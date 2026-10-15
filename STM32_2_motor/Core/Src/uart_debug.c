#include "uart_debug.h"
#include "usart.h"
#include "motor_config.h"
#include "motor_control.h"   /* Motor_RampStart / Motor_RampStop */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ==================== 串口收发缓冲 ====================

   为什么发送不能再用 HAL_UART_Transmit（阻塞）：115200 下发一行 STATUS（约 110
   字节）要 ~10ms，一整屏 HELP（约 900 字节）要 ~80ms。这些时间全卡在后台主循环
   里，而主循环同时还要推进校准状态机、按 ENCODER_SAMPLE_MS(1ms) 采编码器 ——
   一次 STATUS 就让采样停十几毫秒，测速窗口和堵转检测跟着失真。
   改成「写进环形缓冲 + 中断发送」后，调用方入队即返回。

   为什么接收要改成【行队列】：原来的单缓冲有竞态 —— 主循环先把 rx_line_ready
   清 0 再去解析 rx_buf，而中断此时已经可以往同一个 rx_buf 写下一行了，
   解析到一半的字符串会被就地覆盖（表现出来就是"命令偶尔解析成乱七八糟的东西"）。
   现在中断只把完整的一行推进队列，主循环从队头取，两者互不重叠。 */

/* 单行最大长度。★ 从 64 提到 128：64 时限长 63 字符，超出的部分原来是被
   【静默丢弃】的 —— 实测用 60 个空格 + "M1 STOP"（67 字符）发出去，整条命令
   被截成 "M1 "，回一句"缺少子命令"，电机继续转。停机命令被无声吃掉是最不该
   出错的失败模式。128 让最长指令（STOPALL/TRAJ 约 50 字符）有 2 倍余量。
   代价只有 RAM：rx_buf +64、rx_q 4×64=+256，共 +320 字节（bss 6700/20480）。
   注意 rx_len 是 uint8_t，RX_BUF_SIZE 最大只能到 255。 */
#define RX_BUF_SIZE     128
#define RX_QUEUE_LINES  4
/* ★ 必须 >= 固件里最长的那条消息。
   ★ 2026-10-03 起 HELP 已整段删除（原文见 Debug/HELP.txt），它原来是全场最长
   的一条：1787 字节（中文字符 UTF-8 占 3 字节，千万别按字符数估 —— 按字符数
   是 1127，正好蒙混过关），加开机横幅共 1812，就是上电时串口上实际的字节数。
   当年的回归正是它：原来取 1024，HELP 每发必丢且完全静默，现象是"发 HELP
   什么都不回"。现在最长的是 STATUS 行（约 130 字节），两电机一起打印约 260。
   3072 仍然保留不缩小：RAM 不紧张（bss 7020/20480），而缩小只会重新引入
   "某条消息被静默丢弃"这类失败。以后任何一条消息变长，必须同步核对这里。 */
#define TX_BUF_SIZE     3072

/* ---- 接收 ---- */
static uint8_t  rx_buf[RX_BUF_SIZE];   /* 中断正在拼的当前行（只有中断访问） */
static uint8_t  rx_len;                /* 当前行长度（只有中断访问） */
static uint8_t  rx_byte;               /* HAL 的 1 字节接收目标。★ 不标 volatile：
                                          它只被 HAL 的中断服务函数写、只被本文件
                                          的中断回调读，两端都在中断上下文，不存在
                                          跨上下文可见性问题；而加上 volatile 反而
                                          让 HAL_UART_Receive_IT 的 uint8_t* 形参
                                          报"限定符被丢弃"的警告。 */
static uint8_t  rx_q[RX_QUEUE_LINES][RX_BUF_SIZE];
static volatile uint8_t rx_head;       /* 生产者 = 接收中断：下一个要写的槽 */
static volatile uint8_t rx_tail;       /* 消费者 = 主循环：下一个要读的槽 */

/* ---- 发送 ---- */
static uint8_t  tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head;      /* 生产者 = 主循环 */
static volatile uint16_t tx_tail;      /* 消费者 = 发送完成中断 */
static volatile uint16_t tx_chunk;     /* 本次中断发送的字节数 */
static volatile uint8_t  tx_busy;

uint32_t g_uart_tx_dropped;            /* 因缓冲满而丢掉的字节数（SWD 可读） */

/* 启动一段中断发送。必须在临界区（或中断上下文）内调用：
   tx_busy 的「检查—置位」不能被打断，否则主循环和发送完成中断可能同时
   启动两次 HAL_UART_Transmit_IT。 */
static void UART_TX_Kick(void)
{
    uint16_t chunk;

    if (tx_busy || (tx_head == tx_tail))
        return;

    /* 一次发到缓冲区末尾，减少中断次数 */
    chunk = (tx_head > tx_tail) ? (uint16_t)(tx_head - tx_tail)
                                : (uint16_t)(TX_BUF_SIZE - tx_tail);

    tx_busy  = 1;
    tx_chunk = chunk;

    if (HAL_UART_Transmit_IT(&huart1, &tx_buf[tx_tail], chunk) != HAL_OK)
    {
        /* 外设没接受（极少见）。必须把 tx_busy 和 tail 都收回去，
           否则 tx_busy 永远为 1，之后再也发不出任何东西。 */
        tx_busy  = 0;
        tx_tail  = (uint16_t)((tx_tail + chunk) % TX_BUF_SIZE);
        g_uart_tx_dropped += chunk;
    }
}

/* 入队 n 字节（不阻塞）。放不下就整条丢掉并计数 —— 这里绝不能等待，
   等待就退化成阻塞发送，正是要消掉的东西。 */
static void UART_Debug_Send(const char *s, uint16_t n)
{
    uint16_t used, i;
    uint32_t primask;

    if (n == 0U)
        return;

    primask = __get_PRIMASK();
    __disable_irq();                   /* 与发送完成中断里的 tail 更新互斥 */

    used = (uint16_t)((tx_head - tx_tail + TX_BUF_SIZE) % TX_BUF_SIZE);
    if ((uint16_t)(TX_BUF_SIZE - 1U - used) < n)
    {
        g_uart_tx_dropped += n;

        /* 绝不静默失败：留一个可见的标记。整条丢弃在串口上和"命令没生效"
           长得一模一样 —— HELP 就是这么死的（1127B > 1023B，每次都丢，
           而且毫无迹象）。标记自己也放不下时就只计数，不再递归。 */
        if (used + 12U < TX_BUF_SIZE)
        {
            const char *mark = "\r\n[TX DROP]\r\n";
            for (i = 0; mark[i] != '\0'; i++)
            {
                tx_buf[tx_head] = (uint8_t)mark[i];
                tx_head = (uint16_t)((tx_head + 1U) % TX_BUF_SIZE);
            }
            UART_TX_Kick();
        }

        __set_PRIMASK(primask);
        return;
    }

    for (i = 0; i < n; i++)
    {
        tx_buf[tx_head] = (uint8_t)s[i];
        tx_head = (uint16_t)((tx_head + 1U) % TX_BUF_SIZE);
    }

    UART_TX_Kick();

    __set_PRIMASK(primask);
}

static void UART_Debug_SendString(const char *s)
{
    UART_Debug_Send(s, (uint16_t)strlen(s));
}

/* snprintf 出来的字符串（长度已知）走这里，避免再量一次长度 */
static void UART_Debug_SendBuf(const char *s, int len)
{
    if (len > 0)
        UART_Debug_Send(s, (uint16_t)len);
}

/* 忽略大小写比较 */
static int cmd_eq(const char *a, const char *b)
{
    while (*a && *b)
    {
        char ca = (*a >= 'a' && *a <= 'z') ? (*a - 'a' + 'A') : *a;
        char cb = (*b >= 'a' && *b <= 'z') ? (*b - 'a' + 'A') : *b;
        if (ca != cb) return 0;
        a++; b++;
    }
    return *a == *b;
}

/* 无符号整数转字符串，返回长度 */
static int uart_utoa(char *buf, unsigned int v)
{
    char tmp[12];
    int n = 0, i;

    do { tmp[n++] = (char)('0' + (v % 10)); v /= 10; } while (v);
    for (i = 0; i < n; i++) buf[i] = tmp[n - 1 - i];
    buf[n] = '\0';
    return n;
}

/* 浮点值按 2 位小数写入字符串（整数算法，避免依赖 -u _printf_float） */
static void uart_f2(char *buf, float v)
{
    int neg = (v < 0.0f);
    float av = neg ? -v : v;
    unsigned int ip = (unsigned int)av;
    unsigned int fp = (unsigned int)((av - (float)ip) * 100.0f + 0.5f);

    if (fp >= 100) { fp -= 100; ip += 1; }
    if (neg) *buf++ = '-';
    buf += uart_utoa(buf, ip);
    *buf++ = '.';
    *buf++ = (char)('0' + (fp / 10));
    *buf++ = (char)('0' + (fp % 10));
    *buf = '\0';
}

/* 增益被夹取时统一的提示后缀。
   ★ 为什么三个命令共用一条：64KB 的 flash 已经贴边（见文件尾 HELP 的说明），
   KP / KI / VEL 各写一条中文警告会直接把映像撑爆 —— 实测三条长串加起来约
   200 字节，合并成这一条省掉约 140。被夹到多少，看同一行里回显的增益值就
   知道（请求值和回显值一比即知），所以这里不必再重复一遍数值。 */
static const char UART_WARN_CLAMPED[] = " WARN clamped to safe limit";

/* 解析一个"PID 用得上的"浮点数：整段必须都是数字，且结果必须有限。
   ★ 为什么不能只用 strtof 的返回值判断：strtof("nan") 会【成功消费】整个
     token 并返回一个真正的 NaN —— endp 不指向开头、*endp == '\0'，所以
     "是不是数字"的老判断会放它过去。NaN 随后一路进到调制器和 PID：
     实测 M1 SPD nan 会把 PWM 打成 0.00,0.00,0.00（不是运行中的零矢量
     0.50,0.50,0.50），两秒后触发堵转 ERR=32。同一个洞在 TRAJ（amax=NaN
     让制动力曲线整个变 NaN）、VEL 的 ki/kd、CALSPIN/RAMP/ALIGN 的参数上
     都存在，所以收成一个入口，而不是逐个打补丁。
   ★ 为什么用界比较而不是 isfinite()：NaN 与 inf 都过不了这两条界比较
     （NaN 两条都假，±inf 各假一条），而这两个比较在软浮点下编译成
     __ltsf2/__gtsf2 —— 本文件里到处都在用，符号早就链进来了，新增 0 字节
     库代码。isfinite() 在老 newlib 软浮点下可能拉进 __fpclassifyf。
     本片 flash 只剩 356 字节，这个差别是实质的。
     ±1e9 这个界只是"明显不是人输的数"，正常量程（V≈12、rad/s≤20、
     ms≤60000、deg≤1e5）离它六个数量级，不会误伤。 */
static uint8_t uart_parse_f(const char *s, float *out)
{
    char *endp = NULL;
    float v;

    if (s == NULL) return 0;
    v = strtof(s, &endp);
    if (endp == s || endp == NULL || *endp != '\0') return 0;
    if (!(v > -1.0e9f) || !(v < 1.0e9f))            return 0;
    *out = v;
    return 1;
}

/* 浮点值按 1 位小数写入字符串。
   为什么需要：这个工程的链接配置没有开 -u _printf_float，
   snprintf 里的 %f / %.2f 会打印成空字符串。
   我新加的 RAMP / DIR 消息就踩了这个坑 —— 串口输出
   "OK RAMP: 0 -> V / 2000ms, elec  rad/s"，两个数字全是空的，
   导致根本看不到实际施加了多少电压。所有浮点输出一律走这两个函数。 */
static void uart_f1(char *buf, float v)
{
    int neg = (v < 0.0f);
    float av = neg ? -v : v;
    unsigned int ip = (unsigned int)av;
    unsigned int fp = (unsigned int)((av - (float)ip) * 10.0f + 0.5f);

    if (fp >= 10) { fp -= 10; ip += 1; }
    if (neg) *buf++ = '-';
    buf += uart_utoa(buf, ip);
    *buf++ = '.';
    *buf++ = (char)('0' + fp);
    *buf = '\0';
}

/* 浮点值按 3 位小数写入字符串。
   为什么需要：速度环积分增益的合适量级是 0.005 这种，uart_f2 只有 2 位小数，
   会把 0.005 显示成 0.01 —— 看起来和实际写入值不一致，整定时会误判。
   （实测：M2 VEL 0.05 0.005 0 回显成 "ki=0.01"，其实写进去的是 0.005。） */
static void uart_f3(char *buf, float v)
{
    int neg = (v < 0.0f);
    float av = neg ? -v : v;
    unsigned int ip = (unsigned int)av;
    unsigned int fp = (unsigned int)((av - (float)ip) * 1000.0f + 0.5f);

    if (fp >= 1000) { fp -= 1000; ip += 1; }
    if (neg) *buf++ = '-';
    buf += uart_utoa(buf, ip);
    *buf++ = '.';
    *buf++ = (char)('0' + (fp / 100));
    *buf++ = (char)('0' + ((fp / 10) % 10));
    *buf++ = (char)('0' + (fp % 10));
    *buf = '\0';
}

static void UART_Debug_PrintStatus(Motor_t *motor, uint8_t id)
{
    char buf[200];
    char angle_s[16], eangle_s[16], vel_s[16], target_s[16];
    char du_s[16], dv_s[16], dw_s[16], vtrv_s[16], vref_s[16];
    const char *state_str;
    float target, verify_travel;
    int len;

    switch (motor->state)
    {
        case MOTOR_STATE_DISABLED:    state_str = "DISABLED";    break;
        case MOTOR_STATE_INIT:        state_str = "INIT";        break;
        case MOTOR_STATE_CALIBRATION: state_str = "CALIBRATION"; break;
        case MOTOR_STATE_VERIFY:      state_str = "VERIFY";      break;
        case MOTOR_STATE_READY:       state_str = "READY";       break;
        case MOTOR_STATE_RUNNING:     state_str = "RUNNING";     break;
        case MOTOR_STATE_ERROR:       state_str = "ERROR";       break;
        default:                      state_str = "UNKNOWN";     break;
    }

    target = (motor->control_mode == MOTOR_CONTROL_POSITION)
             ? motor->target_angle * RAD_TO_DEG
             : motor->target_velocity;

    uart_f2(angle_s,  motor->mechanical_angle * RAD_TO_DEG);
    uart_f2(eangle_s, motor->electrical_angle * RAD_TO_DEG);
    uart_f2(vel_s,    motor->velocity);
    uart_f2(target_s, target);
    uart_f2(du_s,     motor->duty_u);
    uart_f2(dv_s,     motor->duty_v);
    uart_f2(dw_s,     motor->duty_w);

    /* 上次 VERIFY 自检的带符号行程（折算到「θe 增大方向为正」）。
       换相基准正确时它应明显为正（1V×300ms 下通常 ≫0.01rad）；
       反向 = 换相极性反了，几乎为 0 = 矢量打在 d 轴上（只发热不出力矩）。
       这是判断"基准到底对不对"最直接的证据，所以放进 STATUS。
       追加在行尾：上位机脚本的正则都止于 ERR=，不会受影响。 */
    verify_travel = motor->verify_travel
                  * (((motor->direction * motor->pole_pairs) >= 0.0f) ? 1.0f : -1.0f);
    uart_f2(vtrv_s, verify_travel);

    /* 位置环的【命令】速度 (rad/s)。VEL 是实测速度，这个是内环给定 ——
       整定制动力曲线时必须同时看这两个：给定应当是一条梯形，实测落后给定
       多少就是速度环的跟随误差，也就是残余过冲的来源。
       与 VTRV 一样追加在行尾：上位机脚本的正则止于 ERR=，不受影响。 */
    uart_f2(vref_s, motor->target_velocity);

    len = snprintf(buf, sizeof(buf),
        "M%d STATE=%s ANGLE=%s EANGLE=%s VEL=%s TARGET=%s PWM=%s,%s,%s ERR=%lu VTRV=%s VREF=%s\r\n",
        id, state_str, angle_s, eangle_s, vel_s, target_s, du_s, dv_s, dw_s,
        (unsigned long)motor->error, vtrv_s, vref_s);

    UART_Debug_SendBuf(buf, len);
}

/* 启动类命令的统一反馈。
   Motor_Enable() 只在 READY 状态才真正动作；如果电机处于 ERROR（编码器故障、
   校准失败等），它会静默返回，用户会以为「命令没生效」却查不出原因。
   所以这里把实际结果回报出来。 */
static void UART_Debug_ReportStart(Motor_t *motor)
{
    Motor_Enable(motor);

    if (motor->state == MOTOR_STATE_RUNNING)
    {
        UART_Debug_SendString("OK 电机已启动\r\n");
    }
    else if (motor->state == MOTOR_STATE_ERROR)
    {
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
                           "ERR 电机处于故障态，无法启动 (error=0x%lX)\r\n",
                           (unsigned long)motor->error);
        UART_Debug_SendBuf(buf, len);
    }
    else if (motor->state == MOTOR_STATE_CALIBRATION || motor->state == MOTOR_STATE_INIT
             || motor->state == MOTOR_STATE_VERIFY)
    {
        UART_Debug_SendString("WAIT 正在校准，请稍后再发 STATUS 查看\r\n");
    }
    else
    {
        UART_Debug_SendString("WARN 电机未进入 RUNNING，发送 STATUS 查看状态\r\n");
    }
}

/* ★ 帮助文本已按要求【整段删除】（2026-10-03），回收约 1250 字节 flash ——
   它是 6444 字节 .rodata 里最大的一块字符串。
   原文备份在 Debug/HELP.txt：本工程没有版本控制，删掉就找不回来了。
   删除后 HELP / ? 命令不存在，上电也不再打印命令表。
   各命令的用法提示仍然保留在它们各自的错误回复里（例如
   "ERR: SPD 需要数值，例如 M2 SPD 6"），所以敲错时仍能看到正确写法。 */

/* 命令解析。支持：M1/M2 + START|STOP|SPD|POS|VOL|KP|KI|LIMIT|STATUS|ZERO，
   以及全局的 STATUS / STOPALL / HELP。
   注意：把「无法解析的数字」按 0 处理是危险的（比如误输入 "SPD abc" 会变成停机），
   所以这里对参数做校验，非法就报错、不改动电机。 */
static void UART_Debug_Parse(char *line)
{
    char idstr[12] = {0};
    char cmd[16] = {0};
    char *tok;
    char *arg;
    /* 参数数组：args[0] 是子命令后的第 1 个参数，依次类推。
       多参数命令一律按下标读，不再各自 strtok。 */
    #define MAX_ARGS 6
    char *args[MAX_ARGS];
    int   nargs = 0;
    char *arg1 = NULL, *arg2 = NULL;
    float val = 0.0f;
    int   have_val = 0;
    /* ★ 所有参数在这里一次性解析完，处理器只读数组。
       为什么集中，而不是每个参数各自 strtof + 校验：
       1) -O0 下每次 uart_parse_f 调用要 30~40 字节的取参/调用/回读序列，
          12 个调用点就是 400 多字节 —— 本片 flash 只剩几百字节，这是实打实的
          代价（第一次这么写直接溢出 136 字节）。读数组只要十几字节。
       2) 校验语义只有一个来源，不会出现"这条命令挡了 nan、那条忘了挡"——
          而"某个增益/参数忘了夹"正是 KP→KI→VEL 一路开后门的同一个模式。 */
    float   av[MAX_ARGS];
    uint8_t av_ok[MAX_ARGS];
    Motor_t *motor = NULL;
    uint8_t id = 0;

    tok = strtok(line, " \t\r\n");
    if (!tok) return;

    /* 全局命令优先。注意这里【没有 HELP】—— 帮助文本已整段删除以回收 flash，
       见文件上方那段说明；"HELP" 现在会落到下面的"未知命令"。 */
    if (cmd_eq(tok, "STOPALL"))
    {
        Motor_Disable(&motor1);
        Motor_Disable(&motor2);
        UART_Debug_SendString("OK STOPALL: 两个电机已停机\r\n");
        return;
    }
    if (cmd_eq(tok, "STATUS"))
    {
        UART_Debug_PrintStatus(&motor1, 1);
        UART_Debug_PrintStatus(&motor2, 2);
        return;
    }

    strncpy(idstr, tok, sizeof(idstr) - 1);

    if (cmd_eq(idstr, "M1"))      { motor = &motor1; id = 1; }
    else if (cmd_eq(idstr, "M2")) { motor = &motor2; id = 2; }
    else
    {
        UART_Debug_SendString("ERR: 未知命令\r\n");
        return;
    }

    tok = strtok(NULL, " \t\r\n");
    if (!tok)
    {
        UART_Debug_SendString("ERR: 缺少子命令\r\n");
        return;
    }
    strncpy(cmd, tok, sizeof(cmd) - 1);

    /* 一次性把所有剩余参数收进数组。
       为什么不继续用 strtok：原来每个多参数命令各自 strtok(NULL,...) 取参数，
       而通用解析器已经消费过第 1 个，导致"取几个"和"消费顺序"强耦合 ——
       实测出现过电压/电角速度错位、以及莫名报"参数不足"的情况。
       收进数组后每个处理器只按固定下标读，顺序不再有歧义。 */
    {
        char *p;
        nargs = 0;
        while (nargs < MAX_ARGS && (p = strtok(NULL, " \t\r\n")) != NULL)
            args[nargs++] = p;
    }

    arg  = (nargs >= 1) ? args[0] : NULL;
    arg1 = (nargs >= 2) ? args[1] : NULL;
    arg2 = (nargs >= 3) ? args[2] : NULL;
    /* 最长的命令 RAMP 只用 3 个参数（args[0..2]），所以没有 arg3。
       原先声明了 arg3 却没人用，-Wall 下会报"赋值但未使用"。 */

    /* 一次解析全部参数。av_ok[i] 只在 i < nargs 时被写；下面每个读取都先由
       arg1/arg2/nargs 判断保证下标存在，所以不需要给数组清零。 */
    {
        int i;
        for (i = 0; i < nargs; i++)
            av_ok[i] = uart_parse_f(args[i], &av[i]);
        if (nargs >= 1) { have_val = av_ok[0]; val = av[0]; }
    }

    if (cmd_eq(cmd, "START"))
    {
        UART_Debug_ReportStart(motor);
    }
    else if (cmd_eq(cmd, "STOP"))
    {
        /* 同时清掉【本电机】的开环/斜坡参数，避免下次使能时又自动跑起来。
           ★ 原来是文件级全局，这条 STOP 会把另一个电机的斜坡一起清掉 ——
             若另一个电机正在开环拖动，它的电压会当场掉到 0。 */
        motor->openloop_vel  = 0.0f;
        motor->openloop_volt = 0.0f;
        Motor_RampStop(motor);
        Motor_Disable(motor);
        UART_Debug_SendString("OK STOP\r\n");
    }
    /* 把「当前位置」定义为位置环的 0 度（POS 0 就是这儿）。
       ★ 只动 position_offset，绝不碰 zero_angle。后者是换相基准，
         而此刻转子并不在 d 轴对齐点上，把它写成当前机械角 = 让电角度凭空偏
         7×δ，轻则没力矩、重则负转矩。实测过：改之前发一条 M1 ZERO 再 SPD 1，
         TARGET=+1.00 而电机以约 -10 rad/s 反向飞车。
         详见 motor.h 的 position_offset 说明。 */
    else if (cmd_eq(cmd, "ZERO"))
    {
        motor->position_offset = motor->mechanical_angle;
        motor->target_angle    = motor->mechanical_angle;
        UART_Debug_SendString("OK ZERO\r\n");
    }
    else if (cmd_eq(cmd, "SPD"))
    {
        if (!have_val) { UART_Debug_SendString("ERR: SPD 需要数值，例如 M2 SPD 6\r\n"); return; }
        motor->control_mode = MOTOR_CONTROL_VELOCITY;
        motor->target_velocity = val;
        motor->mode_start_tick = HAL_GetTick();
        UART_Debug_ReportStart(motor);
    }
    else if (cmd_eq(cmd, "POS"))
    {
        if (!have_val) { UART_Debug_SendString("ERR: POS 需要数值(度)，例如 M2 POS 90\r\n"); return; }
        motor->control_mode = MOTOR_CONTROL_POSITION;
        /* 相对于【位置环零点 position_offset】，不是换相基准 zero_angle。
           校准完成时两者相等，所以语义与拆分前一致；ZERO 之后零点就跟着走了。 */
        motor->target_angle = motor->position_offset + val * DEG_TO_RAD;
        UART_Debug_ReportStart(motor);
    }
    else if (cmd_eq(cmd, "VOL"))
    {
        if (!have_val) { UART_Debug_SendString("ERR: VOL 需要数值(V)，例如 M2 VOL 3\r\n"); return; }
        /* ★ 必须先把开环强制旋转停掉。Motor_OpenLoop 只要 openloop_vel != 0，
           就每个周期把 target_voltage 覆写成 openloop_volt（motor_control.c:894-902），
           于是 OLTEST/RAMP 之后紧跟的 VOL 被静默忽略 —— 实测 OLTEST 6 1 之后发
           VOL 3，PWM 偏差仍是 1V 的 ~0.08，而不是 3V 的 ~0.25。
           STOP 早就这么做（同处清 openloop_vel/volt + Motor_RampStop），VOL 漏了。 */
        motor->openloop_vel  = 0.0f;
        motor->openloop_volt = 0.0f;
        Motor_RampStop(motor);
        motor->control_mode = MOTOR_CONTROL_OPEN;
        motor->target_voltage = val;
        UART_Debug_ReportStart(motor);
    }
    /* 单独设 kp。★ 必须和 VEL 命令一样夹到 MOTOR_VEL_KP_MAX：
       原先这里直接赋值、不夹取，于是"M1 KP 0.2"能把安全带整个绕过去 ——
       正是这条安全带要防的"手滑打了个大数"。实测 kp=0.15 已经打出
       ±60 rad/s 的极限环（且 ANGLE 平均列看不见它，只有固件 VEL 的
       min/max 暴露得出来），所以这条不是理论风险。 */
    else if (cmd_eq(cmd, "KP"))
    {
        char buf[112];
        int  len;
        char kp_s[16];
        uint8_t clamped = 0;

        if (!have_val) { UART_Debug_SendString("ERR: KP 需要数值\r\n"); return; }
        if (val > MOTOR_VEL_KP_MAX)  { val = MOTOR_VEL_KP_MAX;  clamped = 1; }
        if (val < -MOTOR_VEL_KP_MAX) { val = -MOTOR_VEL_KP_MAX; clamped = 1; }
        motor->velocity_pid.kp = val;
        motor->velocity_pid.integral = 0.0f;   /* 改增益必须清积分，否则旧积分会冲击输出 */

        uart_f3(kp_s, motor->velocity_pid.kp);
        len = snprintf(buf, sizeof(buf), "OK KP=%s (integral cleared)%s\r\n",
                       kp_s, clamped ? UART_WARN_CLAMPED : "");
        UART_Debug_SendBuf(buf, len);
    }
    /* 静摩擦前馈整定： M2 FF [V]  —— 不带参数则查询当前值。
       为什么需要它：低速（≲1 rad/s）粘滑不是机械问题，是速度环在"用积分去
       发现"一个已知的物理常数（起转阈值约 1.5V），而积分要爬好几秒。
       把已知量直接前馈掉，PID 就只剩粘性阻尼和惯性要管。
       0 = 关闭该电机的前馈（回到纯 PID，用于做 A/B 对比）。
       ★ 前馈量由曲线的平滑部分给出：v=0 时是 0，v≫KNEE 时才接近满值，
         所以查到的值在小目标速度下实际只用到一部分，属于预期行为。 */
    else if (cmd_eq(cmd, "FF"))
    {
        char buf[80];
        int  len;
        char v_s[16];

        if (!have_val)
        {
            uart_f3(v_s, motor->friction_ff_volt);
            len = snprintf(buf, sizeof(buf), "M%d FF=%s V (0=关闭前馈)\r\n", id, v_s);
            UART_Debug_SendBuf(buf, len);
            return;
        }

        if (val < 0.0f) val = -val;      /* 方向由目标速度决定，这里只存幅值 */
        if (val > motor->velocity_volt_limit) val = motor->velocity_volt_limit;

        motor->friction_ff_volt = val;

        /* 改前馈不必清积分：前馈在 PID 外面，PID 的状态没被改动。
           （改 kp/ki 就必须清，因为积分是按旧增益累积的。） */
        uart_f3(v_s, motor->friction_ff_volt);
        len = snprintf(buf, sizeof(buf), "OK M%d FF=%s V\r\n", id, v_s);
        UART_Debug_SendBuf(buf, len);
    }
    /* 位置环制动力曲线整定： M2 TRAJ [amax] 或 M2 TRAJ <vmax> <amax>
       不带参数 = 查询。
       ★ 只给一个数时它是【amax】，不是 vmax —— 顺序故意反过来。
         为什么：vmax 结构上几乎是个常数（位置环输出最后夹在 ±MAX_VELOCITY，
         写超过它只是被白白削平），本项目自始至终真正要整的旋钮是 amax。
         按"vmax 在前"的直觉收单个参数时，"M1 TRAJ 300" 只会把 vmax 夹回 20、
         amax 一动不动，命令只多出一行 WARN —— 很容易被读成"amax 调到 300
         却没效果"，从而得出完全错误的结论。这次实测就真的这样白吃了一轮
         （见 Debug/m1_traj120.txt 的 WARN）。单个参数是 amax 就没有这个坑。
       两个参数时保持 <vmax> <amax> 的老顺序，脚本按两位传参不受影响。
       为什么需要在线整定：amax 是本项目过冲的正主，而"多大才合适"只能实测 ——
       每试一档都重编译+烧录太慢（整定梯子见 motor_config.h 的 MOTOR1_POS_AMAX）。
       ★ 无状态，改完下一拍就生效：限速器每拍都从当前误差重算，
         不需要通知中断"重新规划" —— 这正是不做时基轨迹发生器换来的好处。 */
    else if (cmd_eq(cmd, "TRAJ"))
    {
        char buf[192];
        char v_s[16], a_s[16], m_s[16], d_s[16];
        int  len;
        uint8_t clamped = 0;
        /* 死区的读写单位是【编码器计数】(2π/4096 = 0.001534 rad)：它本来就是
           抗量化噪声的量，用计数最直观；而且 0.0023 rad 走串口的 2 位小数格式
           会打成 "0.00" —— 本工程没开 -u _printf_float，浮点一律走 uart_f1/f2。 */
        const float RAD_PER_COUNT = 0.001534f;

        if (have_val)
        {
            float vmax = motor->traj_vmax;
            float amax = motor->traj_amax;
            float vmin = motor->traj_vmin;
            float dead = motor->traj_deadband;

            if (arg1 == NULL)
            {
                /* 单个参数 = amax（理由见上面那段注释）。其余保持原值不动。 */
                amax = val;
            }
            else
            {
                vmax = val;
                /* "TRAJ 20 abc" 若按 0 处理，会把 amax 悄悄设成 1.0（限速器几乎
                   失效），比报错危险得多；而 "TRAJ 0.5 nan" 更糟 —— amax = NaN
                   让整条制动力曲线变 NaN。av_ok 两个都挡（见 uart_parse_f）。
                   解析失败就保持原值不动，不是置 0。 */
                if (av_ok[1]) amax = av[1];

                if (arg2 != NULL)
                {
                    /* 第 3 个参数 = 最小逼近速度。 */
                    if (av_ok[2]) vmin = av[2];

                    /* 第 4 个参数 = 死区（单位：编码器计数）。上面只声明到 arg2
                       （"没有 arg3"那段注释），所以直接按下标取。 */
                    if (nargs >= 4 && av_ok[3])
                        dead = av[3] * RAD_PER_COUNT;
                }
            }

            /* 夹取理由与 Motor_ApplyConfig 里同一段：vmax 超过 MAX_VELOCITY
               只会让曲线被位置环的输出限幅削平；amax 出现在分母与 sqrt 里，
               0 会让限速器恒为 0（表现为"电机不动、状态却 RUNNING"）。
               vmin 与死区同理由 motor.h 的 traj_vmin 说明：死区小于 1 个编码器
               计数就是在跟量化噪声打架。 */
            if (vmax > MAX_VELOCITY)  { vmax = MAX_VELOCITY; clamped = 1; }
            if (vmax < 0.0f)          { vmax = 0.0f;        clamped = 1; }
            if (amax < 1.0f)          { amax = 1.0f;        clamped = 1; }
            if (vmin > MAX_VELOCITY)  { vmin = MAX_VELOCITY; clamped = 1; }
            if (vmin < 0.0f)          { vmin = 0.0f;        clamped = 1; }
            if (dead < RAD_PER_COUNT) { dead = RAD_PER_COUNT; clamped = 1; }
            if (dead > 0.05f)         { dead = 0.05f;         clamped = 1; }

            motor->traj_vmax     = vmax;
            motor->traj_amax     = amax;
            motor->traj_vmin     = vmin;
            motor->traj_deadband = dead;
        }

        uart_f2(v_s, motor->traj_vmax);
        uart_f2(a_s, motor->traj_amax);
        uart_f2(m_s, motor->traj_vmin);
        uart_f1(d_s, motor->traj_deadband / RAD_PER_COUNT);
        if (have_val && clamped)
        {
            len = snprintf(buf, sizeof(buf),
                           "OK M%d TRAJ vmax=%s amax=%s vmin=%s dead=%s WARN 已夹取\r\n",
                           id, v_s, a_s, m_s, d_s);
        }
        else
        {
            len = snprintf(buf, sizeof(buf),
                           "OK M%d TRAJ vmax=%s amax=%s vmin=%s dead=%s\r\n",
                           id, v_s, a_s, m_s, d_s);
        }
        UART_Debug_SendBuf(buf, len);
    }
    /* 一次设置速度环三个增益： M2 VEL <kp> <ki> <kd>
       为什么需要：原来的 KP / KI 一次只能改一个，而整定速度环必须成套地
       降低增益再逐步加上去，逐条命令既慢又容易把中间状态当成结果。
       例： M2 VEL 0.05 0.5 0     （实测的稳定出厂值，先用它确认能转）
             M2 VEL 0.10 0.5 0     （接近失稳边界；再往上就会振）
       ★ kp 会被夹到 MOTOR_VEL_KP_MAX：实测 0.10 稳定、0.15 就振（±60 rad/s）。
       ★ ki 也不能给太小：这台电机静摩擦约 1.5V，而起步时误差只有目标速度那么大，
         积分得先顶到 1.5V 才起转。ki=0.005 要 300 秒 —— 现象是"走一下停一下"。
         夹住而不是拒绝，是为了让"手滑打了个大数"变成一条能看见的提示，
         而不是一次抖振实验。 */
    else if (cmd_eq(cmd, "VEL"))
    {
        float ki_v = motor->velocity_pid.ki, kd_v = motor->velocity_pid.kd;
        uint8_t clamped = 0, ki_clamped = 0;

        if (!have_val) { UART_Debug_SendString("ERR: VEL <kp> [ki] [kd]，例 M2 VEL 0.05 0.5 0\r\n"); return; }
        /* 次参数"给了就必须是有效数字"，无效则拒绝整条命令。
           ★ 不能只"忽略无效参数、保留旧值"：那样 M1 VEL 0.05 0.5xyz 0 会回 OK，
           而 ki 其实没变 —— 操作者会以为 0.5 已经写进去了。这与单参数的 KP/KI
           不一致（那两条对垃圾输入直接报 ERR），也和被修的缺陷同一性质（原来
           是把 0.5xyz 当成 0.5 写进去）。复用已有的用法提示串，不新增字符串。 */
        if ((arg1 != NULL && !av_ok[1]) || (arg2 != NULL && !av_ok[2]))
        {
            UART_Debug_SendString("ERR: VEL <kp> [ki] [kd]，例 M2 VEL 0.05 0.5 0\r\n");
            return;
        }
        if (arg1 != NULL) ki_v = av[1];
        if (arg2 != NULL) kd_v = av[2];

        if (val > MOTOR_VEL_KP_MAX)  { val = MOTOR_VEL_KP_MAX;  clamped = 1; }
        if (val < -MOTOR_VEL_KP_MAX) { val = -MOTOR_VEL_KP_MAX; clamped = 1; }
        /* ki 和 kp 一样要夹：原先这里只是把 ki_v 直接赋值，于是 VEL 成了
           `Mx KI <大数>` 之外的第二个后门。下限 0（负 ki 是正反馈）。 */
        if (ki_v > MOTOR_VEL_KI_MAX) { ki_v = MOTOR_VEL_KI_MAX; ki_clamped = 1; }
        if (ki_v < 0.0f)             { ki_v = 0.0f;             ki_clamped = 1; }
        /* ★ kd 原本是三个增益里唯一不夹的，补上。kd 作用在被量化过的速度上
           （VELOCITY_WINDOW_MS=8ms，一个编码器计数就是 0.19 rad/s），
           这个工程从未整定过它、出厂值 0 —— 所以给的是 [0, MOTOR_VEL_KD_MAX]
           的小窗口，不是"不限制"。下限同样是 0：负 kd 是正反馈。 */
        if (kd_v > MOTOR_VEL_KD_MAX) { kd_v = MOTOR_VEL_KD_MAX; clamped = 1; }
        if (kd_v < 0.0f)             { kd_v = 0.0f;             clamped = 1; }

        motor->velocity_pid.kp = val;
        motor->velocity_pid.ki = ki_v;
        motor->velocity_pid.kd = kd_v;
        motor->velocity_pid.integral = 0.0f;   /* 改增益必须清积分，否则旧积分会冲击输出 */
        {
            char buf[160];
            char kp_s[16], ki_s[16], kd_s[16];
            int  len;

            uart_f3(kp_s, motor->velocity_pid.kp);
            uart_f3(ki_s, motor->velocity_pid.ki);
            uart_f3(kd_s, motor->velocity_pid.kd);

            /* 夹取时不逐条解释，只挂一句总的 WARN —— kp/ki 的最终值就在本行里，
               跟请求值一比就知道哪个被夹了。比"两个分支各带一条长中文串"省 flash
               （64KB 的片子已经贴边，见文件尾 HELP 的说明）。 */
            len = snprintf(buf, sizeof(buf),
                "OK VEL: kp=%s ki=%s kd=%s (integral cleared)%s\r\n",
                kp_s, ki_s, kd_s,
                (clamped || ki_clamped) ? UART_WARN_CLAMPED : "");
            UART_Debug_SendBuf(buf, len);
        }
    }
    /* 速度环积分增益。作用见上面 VEL 的说明：ki 决定起步要多久才顶得过静摩擦
       （太小 → 走一下停一下），以及会不会进极限环（太大 → 抖振）。
       ★ 修正：这里原来既不夹取、也不清积分，`M1 KI 5` 可以原样写进去 ——
       而 KP 和 VEL 都夹过 kp，等于留了一个后门把整条安全带绕开。
       现在按 KP 的规矩办：夹到 [0, MOTOR_VEL_KI_MAX] 并报 WARN，改完清积分。
       下限是 0 而不是 -MAX：负 ki 是正反馈，结构上必发散，不给。 */
    else if (cmd_eq(cmd, "KI"))
    {
        char buf[96];
        char ki_s[16];
        int  len;
        uint8_t clamped = 0;

        if (!have_val) { UART_Debug_SendString("ERR: KI 需要数值\r\n"); return; }
        if (val > MOTOR_VEL_KI_MAX) { val = MOTOR_VEL_KI_MAX; clamped = 1; }
        if (val < 0.0f)             { val = 0.0f;             clamped = 1; }
        motor->velocity_pid.ki = val;
        motor->velocity_pid.integral = 0.0f;   /* 改增益必须清积分，否则旧积分会冲击输出 */

        uart_f3(ki_s, motor->velocity_pid.ki);
        len = snprintf(buf, sizeof(buf), "OK KI=%s (integral cleared)%s\r\n",
                       ki_s, clamped ? UART_WARN_CLAMPED : "");
        UART_Debug_SendBuf(buf, len);
    }
    /* 速度环输出限幅。
       ★ 上限是 min(母线电压, MOTOR_VEL_VOLT_LIMIT) 而不是 motor->voltage_limit：
         原来允许 "M2 LIMIT 12"，等于把这台电机的堵转保护上限整个绕过去 ——
         电压模式下没有电流采样，12V 堵转就是几安培纯发热（实测烫手）。
         要越过它请改 MOTOR_VEL_VOLT_LIMIT 宏，那是有意为之的动作。 */
    else if (cmd_eq(cmd, "LIMIT"))
    {
        if (!have_val) { UART_Debug_SendString("ERR: LIMIT 需要数值(V)\r\n"); return; }
        if (val < 0.0f) val = -val;
        {
            float vmax = motor->voltage_limit;
            if (vmax > MOTOR_VEL_VOLT_LIMIT) vmax = MOTOR_VEL_VOLT_LIMIT;
            if (val > vmax)
            {
                char buf[128], v_s[16], c_s[16];
                int  len;

                uart_f2(v_s, vmax);
                uart_f2(c_s, val);
                len = snprintf(buf, sizeof(buf),
                               "WARN LIMIT %sV 超过安全上限，已夹到 %sV\r\n", c_s, v_s);
                UART_Debug_SendBuf(buf, len);
                val = vmax;
            }
        }
        motor->velocity_pid.output_max = val;
        motor->velocity_pid.output_min = -val;
        /* ★ 必须同时更新 velocity_volt_limit：静摩擦前馈是加在 PID【外面】的，
           PID 的 output_max 管不到它。若这里只改 PID，用户设 "LIMIT 2" 之后
           总输出仍能到 2 + FF(1.5) = 3.5V —— 限幅被前馈悄悄绕过去了。
           存成同一个字段，前馈的钳位就永远跟着用户的设定走。 */
        motor->velocity_volt_limit = val;
        UART_Debug_SendString("OK LIMIT\r\n");
    }
    /* 旋转式零位校准。
       用法： M2 CALSPIN [电压V] [起始电角速度rad/s]
       例：   M2 CALSPIN          默认 6V / 3.0 rad/s
              M2 CALSPIN 4 2.0    4V / 2.0 rad/s

       流程：开环把转子转起来 → 磁场平滑减速到零 → 转子对齐 → 记录 zero_angle
             → 停机回到 READY。
       为什么需要：旧的静态校准依赖 FOC 输出链本身是准的，而校准恰恰是要确定
       这条链的基准（循环依赖），实际记录的 zero_angle 从来没准过，
       导致闭环时矢量不垂直于转子磁场、只产生保持力，转子被锁死。
       这个版本先让转子转起来，再减速对齐，不依赖任何已有基准。 */
    else if (cmd_eq(cmd, "CALSPIN"))
    {
        float vv = 6.0f, sv = 3.0f;

        if (arg  != NULL && av_ok[0]) vv = av[0];
        if (arg1 != NULL && av_ok[1]) sv = av[1];
        if (vv <= 0.0f) vv = 6.0f;
        if (vv > motor->bus_voltage) vv = motor->bus_voltage;
        if (sv <= 0.0f) sv = 3.0f;
        if (sv > 30.0f) sv = 30.0f;

        /* 只改【本电机】的参数：原来是文件级全局，给电机2 做 CALSPIN 会顺手
           把电机1 的校准电压和拖动速度一起改掉。 */
        motor->align_max_volt = vv;
        motor->calspin_vel    = sv;
        motor->control_mode    = MOTOR_CONTROL_ALIGNZERO;
        motor->mode_start_tick = HAL_GetTick();
        UART_Debug_ReportStart(motor);
        UART_Debug_SendString("OK CALSPIN: spin up, decel to zero, then record zero_angle\r\n");
    }
    /* 开环拖动 -> 闭环速度 交接启动。
       用法： M2 ALIGN <目标速度 rad/s> [最大电压V] [拖动秒数]
       例：   M2 ALIGN 1.0            目标 1.0 rad/s，默认 6V / 3 秒
              M2 ALIGN 2.0 6 4       目标 2.0 rad/s，6V，拖动 4 秒

       为什么需要它：闭环速度环冷启动会死锁（PID 输出小 -> 静摩擦起不来 ->
       编码器不动 -> 误差不减小 -> 输出一直小），最后被堵转保护掐断。
       这里先用缓慢升压的开环旋转磁场把转子拖起来，实测转速跟上目标后
       自动交给速度 PID 接管。 */
    else if (cmd_eq(cmd, "ALIGN"))
    {
        float vmax = 6.0f, secs = 3.0f;

        if (!have_val)
        {
            UART_Debug_SendString("ERR: ALIGN <目标速度rad/s> [最大电压V] [拖动秒数]，例 M2 ALIGN 1.0\r\n");
            return;
        }
        if (arg1 != NULL && av_ok[1]) vmax = av[1];
        if (arg2 != NULL && av_ok[2]) secs = av[2];
        if (vmax <= 0.0f) vmax = 6.0f;
        if (vmax > motor->bus_voltage) vmax = motor->bus_voltage;
        if (secs < 0.5f) secs = 0.5f;
        if (secs > 30.0f) secs = 30.0f;

        motor->align_max_volt = vmax;
        motor->align_drag_ms  = (uint32_t)(secs * 1000.0f);

        /* 必须把第一个参数存成目标速度 —— 漏掉这一行的后果是 ALIGN 全程
           施加零电压（三相恒 50%），转子一动不动，而且状态显示 RUNNING，
           很容易误判成"算法又不行"。 */
        motor->target_velocity = val;
        motor->control_mode   = MOTOR_CONTROL_ALIGN;
        motor->mode_start_tick = HAL_GetTick();
        UART_Debug_ReportStart(motor);
        {
            char buf[160];
            char v_s[16], s_s[16], loop_s[16];
            int  len;

            uart_f2(v_s, motor->align_max_volt);
            uart_f1(s_s, secs);
            /* 把闭环限幅一起报出来：交接时 PID 输出会被它夹住，
               若开环电压高于闭环限幅，交接瞬间电压必然掉一截。
               提前说清楚，省得把它当成"疑似 bug"再查一遍。 */
            uart_f2(loop_s, motor->velocity_pid.output_max);
            len = snprintf(buf, sizeof(buf),
                           "OK ALIGN: target set, drag up to %sV over %ss then hand over "
                           "to PID (closed-loop limit %sV)\r\n",
                           v_s, s_s, loop_s);
            UART_Debug_SendBuf(buf, len);
        }
    }
    /* 换相方向 / 编码器极性。
       为什么要这条命令：FOC 要求「编码器计数增大」与「磁场正转」方向一致。
       该符号是硬件相关的（取决于编码器装向 + 三相接线顺序），无法从代码推断，
       必须实测确定。错了的话 vq>0 会产生负转矩 → 越加电压锁得越死。
       实现：翻转 pole_pairs 的符号。因为 θe = direction×pole_pairs×(θm−zero)，
       反转 pole_pairs 即让 θe 随 θm 反向变化，等效于把换相方向反过来。
       校准得到的 zero_angle 保持不变。
       ★ 修正：原来不带参数走的是"翻转"，和上面这行用法注释直接矛盾，而且很危险 ——
       `Mx DIR` 长得像查询，实际把换相极性翻了过去，紧接着一条 `Mx SPD` 就会朝反方向
       猛冲（VTRV 符号翻转就是这么触发的）。现在不带参数只读、不改任何状态；
       要改必须写明白：`Mx DIR 1` 或 `Mx DIR -1`。
       用法： M2 DIR        （查询当前方向，只读，不翻转）
              M2 DIR -1     （显式设为反向；之后再发 M2 SPD 1 看是否转动）
              M2 DIR 1      （显式设为正向；误翻之后用它恢复） */
    else if (cmd_eq(cmd, "DIR"))
    {
        char buf[64];
        char pp_s[16];
        int  len;

        if (have_val)
        {
            /* 显式指定：val<0 → 取负，val>0 → 取正 */
            float mag = motor->pole_pairs;
            if (mag < 0.0f) mag = -mag;
            motor->pole_pairs = (val < 0.0f) ? -mag : mag;
        }
        /* else：查询，故意什么都不做 */

        uart_f1(pp_s, motor->pole_pairs);
        len = snprintf(buf, sizeof(buf), "%sM%d DIR: pole_pairs=%s theta_e %s\r\n",
                       have_val ? "OK " : "",
                       id, pp_s,
                       (motor->pole_pairs < 0.0f) ? "REVERSED" : "normal");
        UART_Debug_SendBuf(buf, len);
    }
    /* 开环强制旋转测试，运行时可控。
       用途：绕开编码器与闭环反馈，直接看「旋转磁场能否带动转子」，并确定方向。
       这一步能干净地区分：磁场/驱动链问题  vs  编码器/闭环问题。
       用法： M2 OLTEST <电角速度 rad/s> <电压 V>    例如 M2 OLTEST 2 4
              M2 OLTEST -2 4   反向
              M2 STOP          停止 */
    else if (cmd_eq(cmd, "OLTEST"))
    {
        float volt = 0.0f;

        if (!have_val || arg1 == NULL || !av_ok[1])
        {
            UART_Debug_SendString("ERR: OLTEST <电角速度> <电压>，例如 M2 OLTEST 2 4\r\n");
            return;
        }
        volt = av[1];

        motor->openloop_vel  = val;
        motor->openloop_volt = volt;
        motor->control_mode = MOTOR_CONTROL_OPEN;
        UART_Debug_ReportStart(motor);
        UART_Debug_SendString("OK OLTEST (开环强拖；发 M2 STOP 停止)\r\n");
    }
    /* EN 使能极性：  M2 ENP 1  → 高有效（拉高使能）
                       M2 ENP 0  → 低有效（拉低使能）
       这是排查「软件全对但电机完全不动」的关键开关：极性反了的话，
       驱动板一直处于关断态，但 CCER/MOE/CCR/占空比全都看起来正常。
       切换后请重新发 SPD 命令（需要重新使能才生效）。 */
    else if (cmd_eq(cmd, "ENP"))
    {
        /* 极性直接记在电机对象上，所有 EN 操作（使能/关断/报警/校准/开环测试）
           都读它 —— 不再有两个全局变量加三元判断那种"每处都得记得分支"的写法。 */
        if (have_val)
            motor->en_active_high = (val != 0.0f) ? 1 : 0;

        {
            char buf[128];
            int len = snprintf(buf, sizeof(buf),
                               "OK ENP: EN %s有效 (当前引脚电平=%d)\r\n",
                               (motor->en_active_high ? "高" : "低"),
                               (int)HAL_GPIO_ReadPin(motor->en_port, motor->en_pin));
            UART_Debug_SendBuf(buf, len);
        }
        UART_Debug_SendString("   （需重新发 SPD 命令使新极性生效）\r\n");
    }
    /* 开环电压软启动斜坡： M2 RAMP <目标电压V> <电角速度rad/s> <升压时长ms>
       例： M2 RAMP 6 1.5 3000
           —— 3 秒内电压从 0 线性升到 6V，同时磁场以 1.5 rad/s 缓慢旋转。

       为什么要它：一步把电压给到限幅，转子会被"顶"住而不是被"带着转" ——
       静摩擦 + 齿槽让低电压启动失败，而高电压瞬间冲击又会锁死发热。
       缓慢升压 + 磁场慢速旋转，转子是被"逐渐增强的旋转磁场"带起来的。

       停止： M2 STOP（并会把斜坡参数清零） */
    else if (cmd_eq(cmd, "RAMP"))
    {
        /* args[0] 是电压（已解析进 val），args[1] 电角速度，args[2] 升压 ms。
           都从 av[] 读，顺序不再有歧义。 */
        if (!have_val || arg1 == NULL || arg2 == NULL)
        {
            UART_Debug_SendString("ERR: RAMP <电压V> <电角速度rad/s> <升压ms>，例 M2 RAMP 6 1.5 3000\r\n");
            return;
        }

        if (!av_ok[1] || !av_ok[2])
        {
            UART_Debug_SendString("ERR: RAMP 的第 2/3 个参数必须是数字\r\n");
            return;
        }
        if (val <= 0.0f || av[1] == 0.0f)
        {
            UART_Debug_SendString("ERR: 电压需 >0，电角速度需非 0（正负决定转向）\r\n");
            return;
        }
        if (av[2] < 0.0f) av[2] = 0.0f;
        if (val > motor->bus_voltage) val = motor->bus_voltage;

        Motor_RampStart(motor, val, av[1], (uint32_t)av[2]);
        motor->control_mode = MOTOR_CONTROL_OPEN;
        UART_Debug_ReportStart(motor);
        {
            char buf[128];
            char v_s[16], vel_s[16];
            int  len;

            uart_f2(v_s, motor->ramp_volt);
            uart_f2(vel_s, motor->ramp_vel);
            len = snprintf(buf, sizeof(buf),
                           "OK RAMP: 0 -> %sV / %lums, elec %s rad/s\r\n",
                           v_s, (unsigned long)motor->ramp_ms, vel_s);
            UART_Debug_SendBuf(buf, len);
        }
    }
    /* 关掉堵转保护：排查期间用。
       正常运行时它是安全网（电压模式下没有电流采样，堵转时电流全部变热）。
       但排查"电机为什么不起转"时它会误判 —— 低电压下转子还没被带动，
       2 秒就到点跳闸，把正在观察的状态掐断。
       用法： M2 GUARD 0  关；  M2 GUARD 1  开（默认）。
       注意：关掉后请自己在旁边盯着，电机堵转会发热。 */
    else if (cmd_eq(cmd, "GUARD"))
    {
        /* 每电机独立：原来关掉电机2 的堵转保护会连电机1 的一起关掉 */
        if (have_val)
            motor->stall_guard_enable = (val != 0.0f) ? 1 : 0;
        {
            char buf[64];
            int len = snprintf(buf, sizeof(buf), "OK GUARD: 堵转保护 = %s\r\n",
                               motor->stall_guard_enable ? "开" : "关");
            UART_Debug_SendBuf(buf, len);
        }
    }
    /* 清除故障锁存： M2 CLR
       热跳闸后 thermal_i2t 必须已经自然泄放到 MOTOR_THERMAL_CLEAR_MAX 以下
       才允许清除 —— 否则就是"清一下再试"，会把电机反复烧热。 */
    else if (cmd_eq(cmd, "CLR"))
    {
        if (motor->thermal_tripped && motor->thermal_i2t > MOTOR_THERMAL_CLEAR_MAX)
        {
            char buf[128];
            char i2t_s[16], lim_s[16];
            uart_f1(i2t_s, motor->thermal_i2t);
            uart_f1(lim_s, MOTOR_THERMAL_CLEAR_MAX);
            int len = snprintf(buf, sizeof(buf),
                               "ERR: motor still hot (i2t=%s > %s), cool down before clearing\r\n",
                               i2t_s, lim_s);
            UART_Debug_SendBuf(buf, len);
            return;
        }
        motor->error          = MOTOR_ERROR_NONE;
        motor->state          = MOTOR_STATE_READY;
        motor->thermal_i2t    = 0.0f;
        motor->thermal_tripped = 0;
        motor->driver_enabled = 0;
        /* CLR 不走 Motor_Disable，所以积分要在这里单独清一次。不清的话，
           报警停机时积分器停在故障前的位置，清完故障重新 SPD 会在第一个
           周期就把那份陈旧积分加到输出上（与 STOP 甩尾是同一个根因）。 */
        motor->velocity_pid.integral = 0.0f;
        UART_Debug_SendString("OK CLR: 故障已清除，可重新发 SPD/OLTEST\r\n");
    }
    else if (cmd_eq(cmd, "STATUS"))
    {
        UART_Debug_PrintStatus(motor, id);
    }
    else
    {
        UART_Debug_SendString("ERR: 未知子命令\r\n");
    }
}

void UART_Debug_Init(void)
{
    rx_len  = 0;
    rx_head = 0;
    rx_tail = 0;

    UART_Debug_SendString("\r\nSTM32_2_motor ready\r\n");
    /* 原来这里打印整张命令表（UART_Debug_Help）。命令表已删除以回收约 1250
       字节 flash，原文见 Debug/HELP.txt，所以上电现在只有这一行短横幅。
       此时 TX_BUF_SIZE 其实可以大幅缩小（它原来必须 >= 最长消息 = HELP 的
       1787 字节，现在最长的是 STATUS 行约 130 字节），但 RAM 并不紧张
       （bss 7020/20480），缩小只增加"某条长消息被静默丢弃"的风险，
       所以刻意不动它。 */

    /* 开启接收中断：每收到 1 字节进 rx_byte，由 HAL_UART_RxCpltCallback 处理 */
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/* USART1 接收完成回调（中断上下文）：只把字节收进缓冲，绝不做耗时操作 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t ch;

    if (huart->Instance != USART1)
        return;

    ch = rx_byte;

    if (ch == '\n' || ch == '\r')
    {
        if (rx_len > 0)
        {
            uint8_t next = (uint8_t)((rx_head + 1U) % RX_QUEUE_LINES);

            /* 队列满就丢掉这一行，绝不覆盖还没被解析的行 */
            if (next != rx_tail)
            {
                rx_buf[rx_len] = '\0';
                memcpy(rx_q[rx_head], rx_buf, (size_t)rx_len + 1U);
                rx_head = next;
            }
            rx_len = 0;
        }
    }
    else if (rx_len < RX_BUF_SIZE - 1U)
    {
        rx_buf[rx_len++] = ch;
    }
    /* 行超长时静默丢弃多余字符（不覆盖已收内容），下一行照常接收 */

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);   /* 重新开启接收，避免中断只触发一次 */
}

/* USART1 发送完成回调（中断上下文）：推进读指针，若有积压就接着发下一段 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
        return;

    tx_tail = (uint16_t)((tx_tail + tx_chunk) % TX_BUF_SIZE);
    tx_busy = 0;
    UART_TX_Kick();     /* 中断上下文里调用，天然与主循环互斥 */
}

/* 串口错误（溢出/帧错误等）：清标志并重新开启接收，避免 RX 卡死 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
        return;

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/* 后台任务：把队列里积压的完整行全部解析掉（可能发 STATUS，属耗时操作，
   不能在中断里做），再处理周期性状态上报。 */
void UART_Debug_Process(void)
{
    /* 用 while 而不是 if：一次主循环里可能已经攒了好几行，
       每行都要处理，否则积压的命令会延迟好几毫秒才生效。 */
    while (rx_tail != rx_head)
    {
        UART_Debug_Parse((char *)rx_q[rx_tail]);
        rx_tail = (uint8_t)((rx_tail + 1U) % RX_QUEUE_LINES);
    }

    /* 周期性状态上报：连上串口就能直接看到数据，不必反复敲 STATUS。
       只报「正在运行的电机」，都停着时保持安静，避免刷屏。
       周期由 UART_TELEMETRY_MS 控制（motor_config.h），设为 0 即关闭。 */
    {
        static uint32_t last_tx = 0;

        if (UART_TELEMETRY_MS > 0U)
        {
            uint32_t now = HAL_GetTick();

            if ((now - last_tx) >= UART_TELEMETRY_MS)
            {
                last_tx = now;

                if (motor1.state == MOTOR_STATE_RUNNING)
                    UART_Debug_PrintStatus(&motor1, 1);
                if (motor2.state == MOTOR_STATE_RUNNING)
                    UART_Debug_PrintStatus(&motor2, 2);
            }
        }
    }
}
