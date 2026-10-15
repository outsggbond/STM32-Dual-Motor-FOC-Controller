#include "motor_control.h"
#include "motor_config.h"
#include "as5600.h"
#include "tim.h"
#include "i2c.h"
#include "uart_debug.h"
#include <math.h>   /* fabsf：速度环里的静摩擦前馈要用 */

/* 静态函数原型 */
static void Motor_ApplyConfig(Motor_t *motor,
                              float pole_pairs, float direction,
                              float bus_voltage, float voltage_limit,
                              float pos_kp, float pos_ki, float pos_kd,
                              float pos_vmax, float pos_amax,
                              float pos_vmin, float pos_deadband,
                              float vel_kp, float vel_ki, float vel_kd,
                              float friction_ff_volt);
static float AngleNormalize(float angle);
static void Motor_ReadEncoder(Motor_t *motor);
static void Motor_UpdateAngle(Motor_t *motor);
static void Motor_UpdateVelocity(Motor_t *motor);
static void Motor_CalibrationOutput(Motor_t *motor);
static void Motor_OpenLoop(Motor_t *motor);
static void Motor_VelocityControl(Motor_t *motor);
static void Motor_PositionControl(Motor_t *motor);
static void Motor_FOC_Output(Motor_t *motor);
static void Motor_AutoSpinTask(void);
static void Motor2_SpinNRevsTask(void);
static void Motor2_OpenLoopTestTask(void);
static float AngleWrapPi(float a);
static float AngularDifference(float a, float b);
static void Motor_CalibrationVerify(Motor_t *motor);

/* ---- 调试诊断：把 AS5600 内部自检寄存器和 I2C 错误码落到固定 RAM，便于 SWD 直读 ----
   动机：只读 Motor_t 里缓存的 mechanical_angle 无法区分「编码器坏了」和
   「软件根本没去读」。这里绕过缓存，直接问芯片三个关键问题：
     STATUS(0x0B)：磁铁在不在（MD）、太弱（ML）、太强（MH）
     AGC   (0x1A)：自动增益，健康气隙大约在 0x80 附近；0x00 通常表示根本没磁铁
     RAW   (0x0C)：原始角度
   另外记录 HAL 返回码，区分「I2C 通信失败」和「通信成功但磁铁有问题」。 */
typedef struct
{
    uint32_t magic;        /* 固定为 0xA5A50000+序号，用于确认诊断确实在跑 */
    uint32_t loop_count;
    /* [0] = motor1 / I2C1, [1] = motor2 / I2C2 */
    uint32_t read_status[2];   /* HAL 返回码：0=HAL_OK 1=ERROR 2=BUSY 3=TIMEOUT */
    uint8_t  status_reg[2];    /* AS5600 0x0B：bit5 MD, bit4 ML, bit3 MH */
    uint8_t  agc[2];           /* AS5600 0x1A */
    uint16_t raw_angle[2];     /* AS5600 0x0C/0x0D */
    uint32_t i2c_sr1[2];       /* 读完后 I2C 的 SR1，看是否有 BERR/AF/ARLO */
    uint32_t i2c_sr2[2];

    /* —— 电机实时量（每次诊断周期从 Motor_t 拷一份，用于避开结构体偏移不确定的问题）——
       这些字段让「电角度是否在推进」「电压是否真的加上了」可以直接读，不用猜偏移。 */
    float    el_angle[2];      /* 电角度 (rad)：开环时应匀速推进；卡住则说明矢量是静止的 */
    float    mech_angle[2];    /* 机械角 (rad) */
    float    velocity[2];      /* 速度 (rad/s) */
    float    tgt_voltage[2];   /* 输出电压 (V) */
    float    duty_u[2];
    float    duty_v[2];
    float    duty_w[2];
    uint8_t  state[2];         /* MotorState_t */
    uint8_t  control_mode[2];  /* MotorControlMode_t */
    uint8_t  encoder_ok[2];
    uint32_t tim_ccr[2][3];    /* 三个比较寄存器实际值，确认 PWM 真的在输出 */
} DiagBlock_t;

DiagBlock_t g_diag;   /* 非 static：便于在 map 文件里定位其地址 */

static void Diag_ReadAs5600(I2C_HandleTypeDef *hi2c, uint8_t addr, int idx)
{
    uint8_t v = 0;

    g_diag.read_status[idx] =
        (uint32_t)HAL_I2C_Mem_Read(hi2c, (uint16_t)(addr << 1), 0x0B,
                                   I2C_MEMADD_SIZE_8BIT, &v, 1, 50U);
    g_diag.status_reg[idx] = v;

    v = 0;
    (void)HAL_I2C_Mem_Read(hi2c, (uint16_t)(addr << 1), 0x1A,
                           I2C_MEMADD_SIZE_8BIT, &v, 1, 50U);
    g_diag.agc[idx] = v;

    {
        uint8_t d[2] = {0, 0};
        (void)HAL_I2C_Mem_Read(hi2c, (uint16_t)(addr << 1), 0x0C,
                               I2C_MEMADD_SIZE_8BIT, d, 2, 50U);
        g_diag.raw_angle[idx] = (uint16_t)((((uint16_t)d[0] << 8) | d[1]) & 0x0FFF);
    }

    g_diag.i2c_sr1[idx] = hi2c->Instance->SR1;
    g_diag.i2c_sr2[idx] = hi2c->Instance->SR2;
}

static void Diag_Task(void)
{
    static uint32_t last = 0;

    if (HAL_GetTick() - last < DIAG_PERIOD_MS)
        return;
    last = HAL_GetTick();

    Diag_ReadAs5600(&hi2c1, MOTOR1_ENCODER_ADDR, 0);
    Diag_ReadAs5600(&hi2c2, MOTOR2_ENCODER_ADDR, 1);

    /* 拷一份实时量，便于 SWD 直读（诊断周期 500ms，足够观察趋势） */
    {
        Motor_t *m[2];
        int i;
        m[0] = &motor1;
        m[1] = &motor2;
        for (i = 0; i < 2; i++)
        {
            g_diag.el_angle[i]    = m[i]->electrical_angle;
            g_diag.mech_angle[i]  = m[i]->mechanical_angle;
            g_diag.velocity[i]    = m[i]->velocity;
            g_diag.tgt_voltage[i] = m[i]->target_voltage;
            g_diag.duty_u[i]      = m[i]->duty_u;
            g_diag.duty_v[i]      = m[i]->duty_v;
            g_diag.duty_w[i]      = m[i]->duty_w;
            g_diag.state[i]       = (uint8_t)m[i]->state;
            g_diag.control_mode[i]= (uint8_t)m[i]->control_mode;
            g_diag.encoder_ok[i]  = m[i]->encoder_ok;
            g_diag.tim_ccr[i][0]  = __HAL_TIM_GET_COMPARE(m[i]->pwm_timer, m[i]->pwm_u_channel);
            g_diag.tim_ccr[i][1]  = __HAL_TIM_GET_COMPARE(m[i]->pwm_timer, m[i]->pwm_v_channel);
            g_diag.tim_ccr[i][2]  = __HAL_TIM_GET_COMPARE(m[i]->pwm_timer, m[i]->pwm_w_channel);
        }
    }

    g_diag.magic = 0xA5A50000u | (g_diag.loop_count & 0xFFFFu);
    g_diag.loop_count++;
}

/* I2C 总线恢复。
   背景：AS5600 是 I2C 从机，一旦总线被打断（带电插拔、噪声、从机半途掉电），
   从机可能停在「正在输出某一位」的状态死按住 SDA，导致 SDA 永远为低。
   此时 STM32 的 I2C 外设会一直报错、甚至把 PE 位清掉，之后每次读都失败。

   而原固件的「故障恢复」只在**读取成功**的分支里执行，
   总线卡死时读取永远失败 → 恢复代码永远进不去 → 电机永久停在 ERROR，
   只能靠断电重启。这里补上主动恢复：
     1) 关外设并 DeInit（同时会按 MspDeInit 把 SCL/SDA 释放成输入）
     2) 把 SCL 当普通 GPIO 手动打最多 9 个时钟脉冲。
        这是在给「卡在半途的从机」补时钟，让它把剩余位发完、松开 SDA。
     3) 补一个 STOP 条件，把从机状态机复位
     4) 重新 Init 外设（MspInit 会把引脚恢复成复用开漏）
     5) 若 SDA 已被释放则再打一个 STOP
   全程注意 SCL/SDA 是开漏，只能「拉低/释放」，不能主动驱动高电平。 */
static void I2C_BusRecover(I2C_HandleTypeDef *hi2c)
{
    GPIO_TypeDef *port;
    uint16_t     scl_pin;
    uint16_t     sda_pin;
    GPIO_InitTypeDef gpio = {0};
    uint8_t i;

    if (hi2c->Instance == I2C1)      { port = GPIOB; scl_pin = GPIO_PIN_8;  sda_pin = GPIO_PIN_9;  }
    else if (hi2c->Instance == I2C2) { port = GPIOB; scl_pin = GPIO_PIN_10; sda_pin = GPIO_PIN_11; }
    else                             { return; }

    /* 1) 关外设 → 引脚由 MspDeInit 释放为输入 */
    HAL_I2C_DeInit(hi2c);

    /* 2) 引脚配成开漏输出，手动发时钟脉冲 */
    gpio.Pin   = scl_pin | sda_pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(port, &gpio);

    HAL_GPIO_WritePin(port, sda_pin, GPIO_PIN_SET);   /* 释放 SDA */

    for (i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(port, scl_pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(port, scl_pin, GPIO_PIN_SET);   /* 开漏「高」= 释放，靠上拉拉高 */
        HAL_Delay(1);

        /* 从机松开 SDA 就说明它已经复位，可以提前结束 */
        if (HAL_GPIO_ReadPin(port, sda_pin) == GPIO_PIN_SET)
            break;
    }

    /* 3) 补一个 STOP：SCL 高时把 SDA 由低拉高 */
    HAL_GPIO_WritePin(port, sda_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(port, scl_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(port, sda_pin, GPIO_PIN_SET);
    HAL_Delay(1);

    /* 4) 重新初始化外设（MspInit 会把 SCL/SDA 恢复成复用开漏）。
       注意：STM32F1 的 HAL 是 I2Cv1，没有 HAL_I2C_GenerateSTOP()，
       所以 STOP 条件只能靠上面那几步 GPIO 时序来补，这里不要再调外设 API。 */
    HAL_I2C_Init(hi2c);
}

/* 应用电机配置（硬件映射 + PID 参数），两电机参数独立 */
static void Motor_ApplyConfig(Motor_t *motor,
                              float pole_pairs, float direction,
                              float bus_voltage, float voltage_limit,
                              float pos_kp, float pos_ki, float pos_kd,
                              float pos_vmax, float pos_amax,
                              float pos_vmin, float pos_deadband,
                              float vel_kp, float vel_ki, float vel_kd,
                              float friction_ff_volt)
{
    motor->pole_pairs = pole_pairs;
    motor->direction = direction;
    motor->bus_voltage = bus_voltage;
    motor->voltage_limit = voltage_limit;

    /* 运行时参数出厂值。★ 必须每电机写一份：这些量原本是文件级全局，
       电机1 和电机2 共用同一份，任何一条 "M2 RAMP/GUARD/ALIGN/CALSPIN"
       都会顺手改掉电机1 正在用的参数。 */
    motor->en_active_high     = 1;      /* 高有效；由 ENP 命令按板子实测切换 */
    motor->stall_guard_enable = 1;      /* 堵转保护默认开 */
    motor->openloop_vel       = 0.0f;   /* 0 = 不强制推进开环角度 */
    motor->openloop_volt      = 0.0f;
    motor->ramp_volt          = 0.0f;   /* 0 = 斜坡未启用 */
    motor->ramp_vel           = 0.0f;
    motor->ramp_ms            = 0;
    motor->ramp_start_tick    = 0;
    motor->align_max_volt     = MOTOR_ALIGN_DEFAULT_VOLT;
    motor->align_drag_ms      = MOTOR_ALIGN_DEFAULT_DRAG_MS;
    motor->calspin_vel        = MOTOR_CALSPIN_DEFAULT_VEL;

    /* 速度环 Kp 上限：出厂配置值和串口 VEL 命令都用它兜底。
       这台电机上 kp=0.5 实测振荡到 ±38 rad/s，而原来出厂值正是 0.5/1.0 ——
       "上电 START 必然抖"。夹住是为了让任何一次误配都不会变成抖振实验。 */
    if (vel_kp > MOTOR_VEL_KP_MAX)  vel_kp = MOTOR_VEL_KP_MAX;
    if (vel_kp < -MOTOR_VEL_KP_MAX) vel_kp = -MOTOR_VEL_KP_MAX;

    /* 速度环输出上限 = 配置值与安全硬上限中的较小者。
       原实现直接用 voltage_limit（12V），于是只要速度环积分把输出顶到限幅，
       堵转的电机就会被灌 12V —— 没有反电动势，功率全变成绕组发热（实测烫手）。
       这里强制再夹一层 MOTOR_VEL_VOLT_LIMIT。 */
    {
        float vlim = voltage_limit;

        if (vlim > MOTOR_VEL_VOLT_LIMIT) vlim = MOTOR_VEL_VOLT_LIMIT;
        if (vlim < 0.0f) vlim = -vlim;

        /* 同一个 vlim 既给 PID 的 output_max，也存进 motor ——
           Motor_VelocityControl 里给前馈做钳位时要用它，两处必须同源。 */
        motor->velocity_volt_limit = vlim;

        PID_Init(&motor->position_pid, pos_kp, pos_ki, pos_kd, -MAX_VELOCITY, MAX_VELOCITY);
        PID_Init(&motor->velocity_pid, vel_kp, vel_ki, vel_kd, -vlim, vlim);
    }

    /* 静摩擦前馈出厂值。每电机一份，装配不同则起转阈值不同。 */
    motor->friction_ff_volt = friction_ff_volt;

    /* 位置环制动力曲线参数。★ 出厂常量本身是安全的，仍然夹一层 ——
       理由同上面的 MOTOR_VEL_KP_MAX：让任何一次误配都绕不过去。
       vmax 上限取 MAX_VELOCITY，因为位置环输出最后就夹在 ±MAX_VELOCITY，
       写更大只会让这条曲线被削平（前馈在跑、速度被限，误差反而堆起来）。
       amax 下限 1.0f 是防空：它出现在分母与 sqrt 里，0 会让限速器恒为 0，
       位置环就完全不输出了（表现为"电机不动、状态却 RUNNING"）。
       不需要设 amax 上限：amax 越大限速越松，趋近于"完全不限速"，
       也就是本次改动之前那套已经跑过的行为，不会引入新的不稳定。 */
    if (pos_vmax > MAX_VELOCITY) pos_vmax = MAX_VELOCITY;
    if (pos_vmax < 0.0f)         pos_vmax = 0.0f;
    if (pos_amax < 1.0f)         pos_amax = 1.0f;
    motor->traj_vmax = pos_vmax;
    motor->traj_amax = pos_amax;

    /* 最小逼近速度。上限取 MAX_VELOCITY：它是内环给定，比巡航速度还大没有意义。
       下限 0 = 关闭该功能（回到纯 P + 制动力曲线），负值无意义。
       ★ 死区下限取【1 个编码器计数】= 2π/4096 = 0.001534 rad。比这更小就是在
         跟量化噪声打架：转子停在计数边界时读数本来就会在两个相邻码之间跳，
         死区比一个码还窄，下限就会随着这个跳动通断。
       上限 0.05 rad（约 2.9°）：再大就等于几乎全程都在施加上限，位置环退化成
         "恒速蠕动"，保持精度无从谈起。它不是安全上限，是防误配。 */
    if (pos_vmin > MAX_VELOCITY) pos_vmin = MAX_VELOCITY;
    if (pos_vmin < 0.0f)         pos_vmin = 0.0f;
    if (pos_deadband < 0.001534f) pos_deadband = 0.001534f;
    if (pos_deadband > 0.05f)     pos_deadband = 0.05f;
    motor->traj_vmin     = pos_vmin;
    motor->traj_deadband = pos_deadband;
}

/* 角度归一化到 [0, 2π) */
static float AngleNormalize(float angle)
{
    while (angle >= TWO_PI) angle -= TWO_PI;
    while (angle < 0.0f)   angle += TWO_PI;
    return angle;
}

/* 整个电机系统初始化：对象、配置、PWM 启动；零位校准由后台任务完成 */
void Motor_System_Init(void)
{
    Motor_Init(&motor1, MOTOR1_PWM_TIMER, MOTOR1_PWM_U, MOTOR1_PWM_V, MOTOR1_PWM_W,
               MOTOR1_EN_PORT, MOTOR1_EN_PIN, MOTOR1_I2C, MOTOR1_ENCODER_ADDR);
    Motor_ApplyConfig(&motor1, MOTOR1_POLE_PAIRS, MOTOR1_DIRECTION,
                      MOTOR1_BUS_VOLTAGE, MOTOR1_VOLTAGE_LIMIT,
                      MOTOR1_POS_KP, MOTOR1_POS_KI, MOTOR1_POS_KD,
                      MOTOR1_POS_VMAX, MOTOR1_POS_AMAX,
                      MOTOR1_POS_VMIN, MOTOR1_POS_DEADBAND,
                      MOTOR1_VEL_KP, MOTOR1_VEL_KI, MOTOR1_VEL_KD,
                      MOTOR1_FRICTION_FF_VOLT);
    motor1.encoder_sign = MOTOR1_ENCODER_SIGN;

    Motor_Init(&motor2, MOTOR2_PWM_TIMER, MOTOR2_PWM_U, MOTOR2_PWM_V, MOTOR2_PWM_W,
               MOTOR2_EN_PORT, MOTOR2_EN_PIN, MOTOR2_I2C, MOTOR2_ENCODER_ADDR);
    Motor_ApplyConfig(&motor2, MOTOR2_POLE_PAIRS, MOTOR2_DIRECTION,
                      MOTOR2_BUS_VOLTAGE, MOTOR2_VOLTAGE_LIMIT,
                      MOTOR2_POS_KP, MOTOR2_POS_KI, MOTOR2_POS_KD,
                      MOTOR2_POS_VMAX, MOTOR2_POS_AMAX,
                      MOTOR2_POS_VMIN, MOTOR2_POS_DEADBAND,
                      MOTOR2_VEL_KP, MOTOR2_VEL_KI, MOTOR2_VEL_KD,
                      MOTOR2_FRICTION_FF_VOLT);
    motor2.encoder_sign = MOTOR2_ENCODER_SIGN;

    /* 编译期开环测试默认值只给电机2（MOTOR2_OPENLOOP_TEST_* 本来就是给它的）。
       原实现在 Motor_OpenLoop 里对两个电机一律套用这两个宏，等于让电机1 也
       继承电机2 的调试参数；电机1 的对应宏不存在，一旦有人开了电机2 的测试，
       电机1 进 OPEN 模式也会跟着自转。 */
    motor2.openloop_vel  = MOTOR2_OPENLOOP_TEST_VEL;
    motor2.openloop_volt = MOTOR2_OPENLOOP_TEST_VOLTAGE;

    /* PWM 启动，EN 保持关闭 */
    Motor_PWM_Start(&motor1);
    Motor_PWM_Start(&motor2);

    UART_Debug_Init();
}

/* while(1) 后台任务：AS5600 读取缓存 + 零位校准 + 串口（非实时） */
void Motor_System_Task(void)
{
    Motor_ReadEncoder(&motor1);
    Motor_ReadEncoder(&motor2);

    Motor_Calibrate(&motor1);
    Motor_Calibrate(&motor2);

    Motor_AutoSpinTask();
    Motor2_SpinNRevsTask();
    Motor2_OpenLoopTestTask();

    Diag_Task();

    UART_Debug_Process();
}

/* 角度归一化到 (-π, π]，用于求最短路径误差 */
static float AngleWrapPi(float a)
{
    while (a > PI)        a -= TWO_PI;
    while (a <= -PI)      a += TWO_PI;
    return a;
}

/* 两角之间的最短距离（绝对值） */
static float AngularDifference(float a, float b)
{
    float d = AngleWrapPi(a - b);
    return (d < 0.0f) ? -d : d;
}

/* 上电自动常转（调试用）：等校准完成进入 READY 后，自动把电机1 切到速度环常转。
   没有它就必须靠串口发 "M1 SPD <rad/s>"，而串口不通时无法验证硬件。
   关掉：把 MOTOR1_AUTO_SPIN_TARGET_VEL 设为 0.0f。 */
static void Motor_AutoSpinTask(void)
{
    static uint8_t auto_started = 0;

    /* 运行时判断而非 #if：C 预处理器不支持浮点常量。
       设为 0.0f 时此分支恒假，等效于关闭。 */
    if (MOTOR1_AUTO_SPIN_TARGET_VEL <= 0.0f)
        return;

    if (auto_started)
        return;

    /* 只在「校准完成 + 编码器正常」时启动，避免在编码器故障下盲目通电 */
    if (motor1.state != MOTOR_STATE_READY || !motor1.encoder_ok)
        return;

    Motor_Enable(&motor1);                              /* READY -> RUNNING，EN 拉高 */
    motor1.control_mode = MOTOR_CONTROL_VELOCITY;       /* 速度环 */
    motor1.target_velocity = MOTOR1_AUTO_SPIN_TARGET_VEL;

    /* 收紧速度环输出限幅（PID 自带抗积分饱和，这样做比事后钳位干净）：
       默认 ±voltage_limit 会把输出打到 ±12V，对 2804 云台电机过于激进。 */
    motor1.velocity_pid.output_max = MOTOR1_AUTO_SPIN_VOLT_LIMIT;
    motor1.velocity_pid.output_min = -MOTOR1_AUTO_SPIN_VOLT_LIMIT;

    auto_started = 1;
}

/* 电机2：正转 MOTOR2_AUTO_SPIN_REVS 圈，然后停机（调试用）。
   关掉：把 MOTOR2_AUTO_SPIN_REVS 设为 0.0f。

   两个要点：
   1) AS5600 是单圈绝对式，mechanical_angle 只在 [0,2π) 内。这里用每周期增量
      （最短路径）累加成「展开角」travelled，才能数出真实圈数。
   2) 用【速度环】而不是位置环。位置环的误差要经过「位置 PID → 目标速度 → 速度 PID」
      两级串联，增益很难配：实测目标角只领先一点点时，位置误差极小 → 速度给定极小
      → 输出只有 0.2V，不足以让转子起动，而误差又涨不上去，形成死锁（电机原地抖动、
      既不转也不停）。速度环是单级控制，与电机1 走的是同一条已验证路径。 */
static void Motor2_SpinNRevsTask(void)
{
    static uint8_t  started  = 0;
    static uint8_t  finished = 0;
    static float    prev_angle = 0.0f;
    static float    travelled  = 0.0f;
    static uint32_t start_tick = 0;

    float total;
    float elapsed_ms;
    float scale;

    if (MOTOR2_AUTO_SPIN_REVS <= 0.0f)
        return;

    total = MOTOR2_AUTO_SPIN_REVS * TWO_PI;

    /* 只在「校准完成 + 编码器正常」时启动，避免在编码器故障下盲目通电 */
    if (!started)
    {
        if (motor2.state != MOTOR_STATE_READY || !motor2.encoder_ok)
            return;

        Motor_Enable(&motor2);                              /* READY -> RUNNING，EN 拉高 */
        motor2.control_mode = MOTOR_CONTROL_VELOCITY;       /* 速度环：与电机1 相同的已验证路径 */
        motor2.target_velocity = 0.0f;                      /* 先给 0，由下面的软启动逐步加上去 */

        /* 收紧速度环输出限幅（该环决定施加电压），保护 2804 电机 */
        motor2.velocity_pid.output_max = MOTOR2_AUTO_SPIN_VOLT_LIMIT;
        motor2.velocity_pid.output_min = -MOTOR2_AUTO_SPIN_VOLT_LIMIT;

        prev_angle = motor2.mechanical_angle;
        travelled  = 0.0f;
        start_tick = HAL_GetTick();
        started = 1;
        return;
    }

    if (finished)
        return;

    /* 软启动：目标速度在 RAMP_MS 内由 0 线性升到设定值。
       直接给满目标速度会让 PID 瞬间把电压推到限幅，起转太猛、也容易激起抖振。 */
    elapsed_ms = (float)(uint32_t)(HAL_GetTick() - start_tick);
    scale = elapsed_ms / (float)MOTOR2_AUTO_SPIN_RAMP_MS;
    if (scale > 1.0f) scale = 1.0f;
    motor2.target_velocity = MOTOR2_AUTO_SPIN_VEL * scale;

    /* 用最短路径增量累积真实转角（AS5600 是单圈绝对式，必须靠增量才能数圈） */
    travelled += AngleWrapPi(motor2.mechanical_angle - prev_angle);
    prev_angle = motor2.mechanical_angle;

    /* 转够圈数、或超过最长通电时间兜底 → 停机。
       圈数判定在转子被卡住时永远不成立，所以必须有时间兜底。 */
    if (travelled >= total || elapsed_ms >= (float)MOTOR2_AUTO_SPIN_MAX_MS)
    {
        Motor_Disable(&motor2);                             /* 归零 PWM / EN / target */
        finished = 1;
    }
}

/* 开环强制旋转测试：只负责「启动 / 使能 / 超时停机」。
   角度推进与 PWM 输出都在 TIM3 中断路径的 Motor_OpenLoop + Motor_FOC_Output 里完成，
   这里绝不能自己写 PWM —— 中枢每 1ms 就会覆盖一次，两边打架只会得到垃圾波形。 */
static void Motor2_OpenLoopTestTask(void)
{
    static uint8_t  started  = 0;
    static uint8_t  finished = 0;
    static uint32_t start_tick = 0;

    float elapsed_ms;

    if (MOTOR2_OPENLOOP_TEST_VEL <= 0.0f)
        return;

    if (finished)
        return;

    if (!started)
    {
        /* 直接把状态置为 RUNNING + OPEN 模式，跳过依赖编码器的校准步骤。
           开环测试本来就不需要编码器零位，校准反而可能因读失败而中断测试。
           ★ 但 EN 仍必须按极性写：原来这里写死 GPIO_PIN_SET，
             低有效板子上等于把驱动关断 —— 测试看起来"完全不动"，
             很容易被误判成"开环也带不动，硬件有问题"。 */
        Motor_WriteEN(&motor2, 1);
        motor2.driver_enabled = 1;
        motor2.state = MOTOR_STATE_RUNNING;
        motor2.control_mode = MOTOR_CONTROL_OPEN;
        motor2.electrical_angle = 0.0f;
        motor2.target_voltage = MOTOR2_OPENLOOP_TEST_VOLTAGE;
        start_tick = HAL_GetTick();
        started = 1;
        return;
    }

    elapsed_ms = (float)(uint32_t)(HAL_GetTick() - start_tick);

    if (elapsed_ms >= (float)MOTOR2_OPENLOOP_TEST_MAX_MS)
    {
        Motor_Disable(&motor2);
        finished = 1;
    }
}

/* 控制周期入口（TIM3 1 kHz，仅实时控制，无阻塞 I2C） */
void Motor_Control_Loop(void)
{
    Motor_Update(&motor1);
    Motor_Update(&motor2);
}

/* 读取 AS5600 并更新缓存（仅后台任务调用，非实时）：
   连续失败达到阈值才判故障；故障恢复后自动重新零位校准 */
static void Motor_ReadEncoder(Motor_t *motor)
{
    /* 把阻塞式 I2C 读取节流到 ENCODER_SAMPLE_MS；本次未到采样时刻时保留上次缓存值，
       使采样节奏与 1 kHz 控制周期一致，速度环的 dt 才有意义。
       ★ 时间戳必须每电机独立：两个电机的主循环调用是背靠背的（同一毫秒内），
       共用一个时间戳时后调用的那个永远被跳过 —— 它的 mechanical_angle 不再更新，
       速度恒为 0、堵转保护立刻跳闸、ALIGN 的「已跟上目标」判据永不成立。 */
    uint32_t now = HAL_GetTick();
    if ((now - motor->encoder_sample_tick) < ENCODER_SAMPLE_MS)
        return;
    motor->encoder_sample_tick = now;

    if (AS5600_ReadRawAngle(motor->i2c, motor->encoder_addr, &motor->encoder_raw) != HAL_OK)
    {
        motor->encoder_ok = 0;

        if (motor->state != MOTOR_STATE_ERROR)
        {
            motor->encoder_error_count++;

            /* 连续失败到阈值：先主动做一次 I2C 总线恢复。
               必须在直接判故障【之前】做，否则一旦进了 ERROR 就再也不会离开
               （恢复代码原本只在读取成功时执行，而总线卡死时读取永远失败）。 */
            if (motor->encoder_error_count >= ENCODER_ERROR_LIMIT)
            {
                if (!motor->bus_recovered && motor->bus_recover_count < I2C_RECOVER_MAX)
                {
                    I2C_BusRecover(motor->i2c);
                    motor->bus_recovered = 1;
                    motor->bus_recover_count++;
                    motor->encoder_error_count = 0;   /* 给恢复后的总线一个干净的观察窗口 */
                    motor->state = MOTOR_STATE_INIT;  /* 重新走校准 */
                    return;
                }

                if (motor->bus_recover_count >= I2C_RECOVER_MAX)
                {
                    /* 反复恢复仍失败 → 判硬件故障，不再无谓地复位总线 */
                    Motor_SetError(motor, MOTOR_ERROR_ENCODER);
                }
                else
                {
                    /* 已在本轮恢复过一次仍失败：这一轮判定失败，下次读取会再触发恢复 */
                    Motor_SetError(motor,
                        (motor->state == MOTOR_STATE_CALIBRATION)
                        ? MOTOR_ERROR_CALIBRATION : MOTOR_ERROR_ENCODER);
                }
            }
        }
        return;
    }

    motor->encoder_error_count = 0;
    motor->bus_recovered = 0;      /* 读取成功，允许将来再次触发总线恢复 */
    motor->encoder_ok = 1;
    /* 按实测的编码器计数方向折算机械角。
       ★ 这里的取反不是"顺手反一下"：AS5600 的计数正方向由磁铁装向决定，
       与电机的换相正方向不一定一致。不一致却直接用，就会得到
       「换相基准看着对、速度环却成了正反馈」的组合 —— 一发 SPD/ALIGN 就飞车。
       取反后归一到 [0, 2π)。zero_angle 在校准时也是从这里取的，自动同约定。 */
    {
        float a = (float)motor->encoder_raw / 4096.0f * TWO_PI;
        if (motor->encoder_sign < 0.0f)
        {
            a = -a;
            if (a < 0.0f) a += TWO_PI;
        }
        motor->mechanical_angle = a;
    }
    motor->encoder_stamp = now;   /* 记录本次成功采样时刻，供速度环算真实 dt */

    /* 编码器/校准故障恢复：读恢复后自动重新零位校准 */
    if (motor->state == MOTOR_STATE_ERROR)
    {
        uint32_t recoverable = MOTOR_ERROR_ENCODER | MOTOR_ERROR_CALIBRATION;
        if ((motor->error & ~recoverable) == 0)
        {
            motor->error = MOTOR_ERROR_NONE;
            motor->state = MOTOR_STATE_INIT;
        }
    }
}

/* VERIFY 阶段：施加小幅 +q 轴电压，确认「换相正确时确实产生转矩」。

   ★ 判据必须是【带符号】的，不能只看"动没动"：
     vq>0 时转子的期望方向是使 θe 增大，即
         sign(Δθm) = sign(direction × pole_pairs)
     因为 θe = k·(θm − zero)（k = direction×pole_pairs），vq>0 表示矢量超前
     估计 d 轴 90°，正确换相下转子就朝 θe 增大的方向加速。
     只判 |Δθ| 会漏掉最危险的一类错误：基准差 180° 时转矩反号，转子【照样会动】，
     自检通过、闭环却必然跑飞。反向必须报错，绝不能继续运行。

   行程用「每拍带符号增量累加」而不是首尾两点求差：单圈绝对式编码器首尾差
   必须回绕，转过半圈以上就会被算成反向；逐拍累加没有这个问题，
   顺带得到一个可直接观察的诊断量（STATUS 里的 VTRV）。

   反向时先自动翻一次换相极性重试（这就是 DIR -1 的效果），
   两种极性都试过仍反向 → 判基准/硬件故障。用 polarity_tried 位掩码保证
   最多重试一次，不会来回死循环。 */
static void Motor_CalibrationVerify(Motor_t *motor)
{
    uint32_t elapsed;
    float    expect, travel;

    /* 进入 VERIFY 的第一拍：清行程，ISR 开始输出 +q 电压 */
    if (!motor->verify_armed)
    {
        motor->verify_armed      = 1;
        motor->verify_travel     = 0.0f;
        motor->verify_last_angle = motor->mechanical_angle;
        motor->calib_start_tick  = HAL_GetTick();
        motor->control_mode      = MOTOR_CONTROL_OPEN;
        return;
    }

    /* 累加本拍带符号位移（最短路径增量，不受 0/2π 跳变影响） */
    motor->verify_travel +=
        AngleWrapPi(motor->mechanical_angle - motor->verify_last_angle);
    motor->verify_last_angle = motor->mechanical_angle;

    elapsed = HAL_GetTick() - motor->calib_start_tick;
    if (elapsed < CALIBRATION_VERIFY_MS)
        return;

    motor->verify_armed = 0;
    Motor_Disable(motor);   /* 先断电，再判结果：PWM 0 + EN LOW + READY */

    /* 折算到「θe 增大方向为正」 */
    expect = ((motor->direction * motor->pole_pairs) >= 0.0f) ? 1.0f : -1.0f;
    travel = motor->verify_travel * expect;

    if (travel >= CALIBRATION_VERIFY_RAD)
    {
        motor->error = MOTOR_ERROR_NONE;   /* 方向对、幅度够 ⇒ 换相基准有效 */
        return;
    }

    if (travel <= -CALIBRATION_VERIFY_RAD)
    {
        /* 明确反向（不是"没动"）⇒ 换相极性的符号反了，翻一次再校准。
           翻 pole_pairs 只改变「θm → θe」的符号约定，不改动物理过程：
           重新对齐后 zero_angle 仍是同一个机械位置，而自检的期望方向随之反过来，
           于是物理方向不变、判据变对。 */
        uint8_t bit   = (motor->pole_pairs >= 0.0f) ? 0x01u : 0x02u;
        uint8_t other = (motor->pole_pairs >= 0.0f) ? 0x02u : 0x01u;

        if ((motor->polarity_tried & other) == 0u)
        {
            motor->polarity_tried |= bit;
            motor->pole_pairs = -motor->pole_pairs;
            motor->error = MOTOR_ERROR_NONE;
            motor->state = MOTOR_STATE_INIT;   /* 重走 校准 → 自检 */
            return;
        }
    }

    /* 几乎没动（矢量打在 d 轴上：只发热不出力矩），
       或两种极性都试过仍反向 ⇒ 不是极性问题，是基准/驱动问题 */
    Motor_SetError(motor, MOTOR_ERROR_CALIBRATION);
}

/* 零位校准（后台任务驱动）：对齐 → 确认静止 → 记录 zero_angle → VERIFY → READY。
   与旧实现的区别：
     1) 不再「等固定时间就记录」，而是等转子真正静止（连续 SETTLE_MS 内位移极小）
     2) 超时仍未静止 → 明确报校准失败，不记录一个随机位置当基准
     3) 对齐用 d 轴固定矢量，zero_angle = 机械角，不做 π/pole_pairs 补偿
     4) 记录后追加 VERIFY（+q 轴电压）：判据是【带符号】的 —— 转子必须朝
        θe 增大的方向转。反向 ⇒ 换相极性符号反了，自动翻转一次重走校准；
        几乎不动 ⇒ 矢量打在 d 轴上（只发热不出力矩），直接报错。
   详见 motor_config.h 的「零位校准（换相基准）」一节。 */
void Motor_Calibrate(Motor_t *motor)
{
    if (motor->state == MOTOR_STATE_INIT)
    {
        /* 进入校准：使能驱动，ISR 输出固定电压矢量拉转子对齐。
           EN 走统一的 Motor_WriteEN()，极性由 motor->en_active_high 决定 ——
           原来这里写死 GPIO_PIN_SET，ENP 0（低有效）的板子在校准期间反而是
           把驱动关断的，转子根本不会被拉。 */
        Motor_WriteEN(motor, 1);
        motor->driver_enabled = 1;
        motor->state = MOTOR_STATE_CALIBRATION;
        motor->calib_start_tick = HAL_GetTick();

        /* 兜底清掉「对齐用 d 轴」标志：万一上一次 CALSPIN 阶段3 被 STOP/故障
           打断，标志会残留，之后 VERIFY 或速度环就会把电压打到 d 轴上（只发热）。 */
        motor->align_use_vd = 0;

        /* 复用 stall_* 作为「静止检测」窗口（RUNNING 态才归堵转保护用，此处空闲） */
        motor->stall_tick        = HAL_GetTick();
        motor->stall_angle       = motor->mechanical_angle;
        motor->stall_last_angle  = motor->mechanical_angle;
        motor->stall_travel      = 0.0f;
    }
    else if (motor->state == MOTOR_STATE_CALIBRATION)
    {
        uint32_t now     = HAL_GetTick();
        uint32_t elapsed = now - motor->calib_start_tick;

        /* 转子是否已静止？把「静止窗口起点」不断前移，一旦窗口满且位移够小即认为到位 */
        if (AngularDifference(motor->mechanical_angle, motor->stall_angle)
            >= CALIBRATION_SETTLE_RAD)
        {
            /* 还在动：重置窗口 */
            motor->stall_tick  = now;
            motor->stall_angle = motor->mechanical_angle;
        }
        else if ((now - motor->stall_tick) >= CALIBRATION_SETTLE_MS)
        {
            /* 连续 SETTLE_MS 内几乎没动 ⇒ 转子已对齐并静止，可以记录基准 */
            motor->zero_angle     = motor->mechanical_angle;
            /* 位置环零点跟随校准基准初始化：POS <度> 是「相对校准零点」的角度，
               与拆分前的语义完全一致。两者分开存的原因见 motor.h 的 position_offset。 */
            motor->position_offset = motor->zero_angle;
            motor->encoder_stamp  = now;
            motor->electrical_angle = 0.0f;
            motor->velocity       = 0.0f;
            motor->velocity_window_stamp = motor->encoder_stamp;
            motor->velocity_window_angle = motor->mechanical_angle;

            /* 进入 VERIFY：施加 +q 轴电压，确认真的能产生转矩。
               verify_armed 先清 0：即使上一次自检被中途打断（STOP / 故障），
               这次也一定会重新记录起始角，不会拿旧的行程做判断。 */
            motor->electrical_angle = 0.0f;
            motor->target_voltage   = CALIBRATION_VERIFY_VOLT;
            motor->control_mode     = MOTOR_CONTROL_OPEN;
            motor->state            = MOTOR_STATE_VERIFY;
            motor->verify_armed     = 0;
            Motor_CalibrationVerify(motor);   /* 初始化本阶段 */
            return;
        }

        /* 超时仍未静止 ⇒ 校准电压不足以把转子拉到对齐位置。报错而不是记录错误基准。 */
        if (elapsed >= CALIBRATION_MAX_MS)
        {
            Motor_SetError(motor, MOTOR_ERROR_CALIBRATION);
        }
    }
    else if (motor->state == MOTOR_STATE_VERIFY)
    {
        Motor_CalibrationVerify(motor);
    }
}

/* 电角度：θe = direction × pole_pairs × (θm - zero_angle)，归一化到 [0, 2π) */
float Motor_GetElectricalAngle(Motor_t *motor)
{
    float angle = motor->direction * motor->pole_pairs
                * (motor->mechanical_angle - motor->zero_angle);
    return AngleNormalize(angle);
}

/* 更新电角度（使用缓存机械角，无 I2C） */
static void Motor_UpdateAngle(Motor_t *motor)
{
    motor->electrical_angle = Motor_GetElectricalAngle(motor);
}

/* 更新速度：固定时间窗测速（见 motor.h 里 velocity_window_* 的说明）。
   核心：不在每次编码器采样上微分（1ms 窗口会把 1 个计数的量化抖动放大成
   1.5 rad/s 的假速度），而是等窗口跨越 VELOCITY_WINDOW_MS 后，用窗口两端的
   角度差除以窗口长度。

   ★ 窗口两端锚在【编码器采样时刻】(encoder_stamp)，不是中断观察时刻：
     后台主循环里还夹着阻塞式 I2C 读和串口发送，中断看到"窗口满了"的时刻
     本身在抖，用它会把这个抖动除进速度里。用两次采样时刻之差做分母，
     得到的是真实跨越时间 —— 这才是 encoder_stamp 存在的意义。 */
static void Motor_UpdateVelocity(Motor_t *motor)
{
    uint32_t stamp = motor->encoder_stamp;   /* 最新一次成功采样的时刻 */
    float    delta, vmax;

    if (stamp == motor->velocity_window_stamp)
    {
        /* 窗口起点之后还没有新的采样点：不能重算（分母没变、分子也没变，
           重算只会把同一个值反复写一遍），更不能把"这次其实没读到新数据"
           的时间算进 dt —— 那会让速度被系统性低估。
           但也不能一直冻着上一个值：编码器读失败时 stamp 不再更新，
           速度会永远停在最后一次的值上，堵转保护于是以为电机一直在转、
           永不跳闸。超过 VELOCITY_STALE_MS 没有新采样就判 0。 */
        if ((HAL_GetTick() - stamp) > VELOCITY_STALE_MS)
            motor->velocity = 0.0f;
        return;
    }

    if ((stamp - motor->velocity_window_stamp) < VELOCITY_WINDOW_MS)
        return;                              /* 窗口未满，保持上次速度值 */

    delta = AngleWrapPi(motor->mechanical_angle - motor->velocity_window_angle);

    motor->velocity = delta / ((float)(stamp - motor->velocity_window_stamp) * 0.001f);

    /* 混叠上限：AS5600 是单圈绝对式，窗口内转过半圈以上就无法分辨转了多少圈，
       此时算出来的值是无意义的。可分辨上限 = π/窗口时长；越界就夹住，
       宁可给一个"到顶"的值，也不要让垃圾值进 PID（那会被积分放大成飞车）。
       VELOCITY_WINDOW_MS 已按此约束取得足够短（8ms → 393 rad/s）。 */
    vmax = PI / ((float)VELOCITY_WINDOW_MS * 0.001f);
    if (motor->velocity >  vmax) motor->velocity =  vmax;
    if (motor->velocity < -vmax) motor->velocity = -vmax;

    motor->velocity_window_stamp = stamp;
    motor->velocity_window_angle = motor->mechanical_angle;
}

/* 校准输出：固定电压矢量（α 轴），拉转子到固定电角位置 */
static void Motor_CalibrationOutput(Motor_t *motor)
{
    float va = CALIBRATION_VOLTAGE / motor->bus_voltage;

    FOC_SVPWM(va, 0.0f, &motor->duty_u, &motor->duty_v, &motor->duty_w);
    Motor_SetPWM(motor, motor->duty_u, motor->duty_v, motor->duty_w);
}

/* EN 使能极性（1 = 高有效）现在在 Motor_t 里（motor->en_active_high），
   默认 1，可由串口 "M2 ENP 0/1" 切换。
   为什么要做成运行时变量：极性搞反时所有寄存器看起来都正常
   （CCER/MOE/CCR 全对，PWM 引脚也能量到正确波形），
   但驱动板实际关断、电机零转矩。默认值得靠实测确定。
   ★ 原来分两个全局变量（motor1_en_active_high / motor2_en_active_high），
     由 "motor == &motor1 ? ... : ..." 的三元表达式挑一个 —— 这种写法每多一处
     EN 操作就多一次漏判的机会（Motor_SetError 和 Motor_Init 就都漏了，
     在低有效板子上"报警停机"反而把驱动使能了）。现在统一走 Motor_WriteEN()。 */

/* 堵转保护开关（串口 M2 GUARD 0/1）现在是 motor->stall_guard_enable，默认开。
   正常运行时它是安全网；排查"电机为什么不起转"时要能关掉它，
   否则低电压下转子还没被带动，2 秒就跳闸，把观察过程掐断。 */

/* 启动/停止开环电压斜坡。集中在这里设置起始时刻，避免把内部状态暴露出去。 */
void Motor_RampStart(Motor_t *motor, float volt, float elec_vel, uint32_t ramp_ms)
{
    motor->ramp_volt = volt;
    motor->ramp_vel  = elec_vel;
    motor->ramp_ms   = ramp_ms;
    motor->ramp_start_tick = HAL_GetTick();
}

void Motor_RampStop(Motor_t *motor)
{
    motor->ramp_volt = 0.0f;
    motor->ramp_vel  = 0.0f;
    motor->ramp_ms   = 0;
}

/* 开环强制旋转测试（在 TIM3 中断路径里执行）。
   关键点：不重写输出逻辑，只把 electrical_angle 换成「自己推进的角度」，
   后面的 Motor_FOC_Output 就会照这个角度输出电压矢量 —— 与闭环共用同一条输出链，
   唯一的差别就是角度来源。这样判别才干净：输出链若有问题两边都会抖。
   参数全部取自本电机（motor->openloop_* / motor->ramp_*），互不影响。 */
static void Motor_OpenLoop(Motor_t *motor)
{
    float vel, volt;

    /* 斜坡模式优先：电压 0 → 目标值线性上升 */
    if (motor->ramp_volt != 0.0f)
    {
        uint32_t elapsed = HAL_GetTick() - motor->ramp_start_tick;
        float    frac;

        if (motor->ramp_ms == 0) frac = 1.0f;
        else if (elapsed >= motor->ramp_ms) frac = 1.0f;
        else frac = (float)elapsed / (float)motor->ramp_ms;

        vel  = motor->ramp_vel;
        volt = motor->ramp_volt * frac;
    }
    else
    {
        /* 没有斜坡就走 OLTEST 的固定值；两者都为 0 时不推进角度，
           于是 FOC 会按当前 electrical_angle 输出一个【静止矢量】——
           这正是串口 VOL 命令想要的行为（开环固定电压）。 */
        vel  = motor->openloop_vel;
        volt = motor->openloop_volt;
    }

    if (vel == 0.0f)
        return;                        /* 不推进角度：保持 VOL 设定的固定矢量 */

    /* 允许负速度（用于反向强制旋转实验） */
    motor->electrical_angle += vel * CONTROL_DT;
    while (motor->electrical_angle >= TWO_PI) motor->electrical_angle -= TWO_PI;
    while (motor->electrical_angle < 0.0f)    motor->electrical_angle += TWO_PI;

    motor->target_voltage = volt;
}

/* 旋转式零位校准：把转子转起来，再让磁场平滑减速到零，读取 zero_angle。

   原理：转子一旦被旋转磁场带动，就与磁场同步。把电角速度线性降到 0，
   转子跟着减速并最终停在与磁场对齐的位置。此时电角度恰为 2π 的整数倍，
   于是 zero_angle = 机械角。

   为什么不能用"从静止施加固定矢量"的做法：那需要 FOC 输出链本身是准的，
   而校准恰恰是要去确定这条链的基准 —— 循环依赖。先转起来就绕开了这个问题。

   ★ 换相极性（θe 随 θm 的符号）也在这里定出来，这是唯一能可靠定它的地方：
     拖动阶段磁场【始终】朝 θe 增大方向转（vel_elec 恒为正），转子被同步带着走，
     所以 θe 也在增大；而 θe = direction×pole_pairs×θm + 常数，于是必有
         sign(Δθm) = sign(direction × pole_pairs)
     Δθm<0 就说明编码器计数方向与磁场旋转方向相反，必须把 pole_pairs 取负
     （等效串口 DIR -1）。这一步不依赖 zero_angle、也不需要任何先验极性，
     因为开环拖动本身就提供了参照系 —— 只有"从静止拉一下"是做不到的
     （从静止拉，转子朝哪个电气方向走取决于它当时停在哪里，方向无意义）。

   ★ 对齐用纯 d 轴电压（align_use_vd）：d 轴就是转子磁场方向的定义，
     vd 打在哪里磁极就停在哪里，所以「估计 d 轴 ≡ 真实 d 轴」自动成立，
     zero_angle = 机械角，不需要任何补偿。
     原先用 q 轴对齐再补 π/pole_pairs 是错的：q 轴与 d 轴差 90°，
     补 180° 和补 90° 不是一回事，而 |cos(90°)| = |cos(270°)| = 0 ——
     两种都对应"转矩恒为 0、电流全在 d 轴上、只发热不转"。

   参数由 motor->align_max_volt（电压）和 motor->calspin_vel（起始电角速度）控制，
   都由串口 CALSPIN 命令按电机单独设置。 */
#define CALSPIN_RAMP_UP_MS   1500U   /* 升压 + 升速阶段 */
#define CALSPIN_DECEL_MS     3000U   /* 线性减速阶段 */
#define CALSPIN_SETTLE_MS    400U    /* 减速到零后等待稳定 */
#define CALSPIN_SETTLE_RAD   0.02f   /* 稳定判定阈值 (rad) */
/* 拖动行程下限 (rad)：小于它说明转子压根没被带起来，此时的"静止"只是
   因为它一直没动 —— 记录下来的会是随机位置的假基准（旧实现的老毛病），
   必须报错而不是记录。0.2rad≈11.5° 机械角，远高于编码器量化噪声
   （1 计数 = 0.0015rad），慢速 CALSPIN 也够。 */
#define CALSPIN_MIN_DRAG_RAD 0.2f

static void Motor_AlignZeroControl(Motor_t *motor)
{
    uint32_t elapsed = HAL_GetTick() - motor->mode_start_tick;
    float    vel_elec, volt, frac;

    /* 新一次 CALSPIN：mode_start_tick 由命令处理里重设，据此复位本次运行的
       累加状态（中途被 STOP 打断后再进来也不会沿用上一次的行程）。 */
    if (motor->calspin_run_tick != motor->mode_start_tick)
    {
        motor->calspin_run_tick    = motor->mode_start_tick;
        motor->calspin_last_angle  = motor->mechanical_angle;
        motor->calspin_drag_travel = 0.0f;
        motor->calspin_init        = 0;
        motor->align_use_vd        = 0;   /* 只允许阶段3 置位，新一轮先清 */
    }

    /* 逐拍累加带符号机械角增量。用最短路径增量：AS5600 是单圈绝对式，
       角度在 [0,2π) 内跳变，直接首尾求差会算错；逐拍累加则等价于净行程，
       且任意时刻都可用。 */
    motor->calspin_drag_travel +=
        AngleWrapPi(motor->mechanical_angle - motor->calspin_last_angle);
    motor->calspin_last_angle = motor->mechanical_angle;

    vel_elec = motor->calspin_vel * ((motor->pole_pairs >= 0.0f) ? motor->pole_pairs
                                                                  : -motor->pole_pairs);

    if (elapsed < CALSPIN_RAMP_UP_MS)
    {
        /* 阶段 1：电压线性升到上限，同时磁场已经以满速旋转（先把转子带起来） */
        frac = (float)elapsed / (float)CALSPIN_RAMP_UP_MS;
        volt = motor->align_max_volt * frac;
    }
    else if (elapsed < CALSPIN_RAMP_UP_MS + CALSPIN_DECEL_MS)
    {
        /* 阶段 2：电压保持，电角速度从 calspin_vel 线性降到 0。
           转子跟着磁场一起减速，最终停在对齐位置。 */
        uint32_t t = elapsed - CALSPIN_RAMP_UP_MS;
        frac = 1.0f - (float)t / (float)CALSPIN_DECEL_MS;
        if (frac < 0.0f) frac = 0.0f;
        vel_elec *= frac;
        volt = motor->align_max_volt;
    }
    else
    {
        /* 阶段 3：磁场已停。用【纯 d 轴】电压把转子稳定锁在基准上（vd 而不是 vq），
           等它连续 CALSPIN_SETTLE_MS 不动，即可确认对齐。 */
        float drag = motor->calspin_drag_travel;
        if (drag < 0.0f) drag = -drag;

        motor->align_use_vd = 1;
        volt = motor->align_max_volt * 0.5f;

        if (!motor->calspin_init)
        {
            motor->calspin_init         = 1;
            motor->calspin_settle_angle = motor->mechanical_angle;
            motor->calspin_settle_tick  = HAL_GetTick();
        }

        if (AngularDifference(motor->mechanical_angle, motor->calspin_settle_angle)
            >= CALSPIN_SETTLE_RAD)
        {
            motor->calspin_settle_angle = motor->mechanical_angle;
            motor->calspin_settle_tick  = HAL_GetTick();
        }
        else if ((HAL_GetTick() - motor->calspin_settle_tick) >= CALSPIN_SETTLE_MS)
        {
            /* 先确认拖动真的带起过转子，再记录基准 */
            if (drag < CALSPIN_MIN_DRAG_RAD)
            {
                motor->align_use_vd = 0;
                motor->calspin_init = 0;
                Motor_SetError(motor, MOTOR_ERROR_CALIBRATION);
                return;
            }

            /* 定换相极性：拖动时 θe 在增大，故 sign(Δθm) 必须等于 sign(k)。 */
            if (motor->calspin_drag_travel < 0.0f)
                motor->pole_pairs = -motor->pole_pairs;

            /* d 轴对齐 ⇒ 估计 d 轴就落在磁极方向上，zero_angle = 机械角。
               极性翻转不会影响这个位置（对齐是 d 轴拉过去的，与 θm→θe 的
               符号约定无关），所以这里不需要重算。 */
            motor->zero_angle       = motor->mechanical_angle;
            motor->position_offset  = motor->zero_angle;   /* 同上：POS 的基准跟着校准走 */
            motor->electrical_angle = 0.0f;
            motor->velocity         = 0.0f;
            motor->velocity_window_stamp = motor->encoder_stamp;
            motor->velocity_window_angle = motor->mechanical_angle;
            motor->calspin_init     = 0;
            motor->align_use_vd     = 0;
            Motor_Disable(motor);      /* PWM 0 + EN 关断 + 回 READY */
            motor->error = MOTOR_ERROR_NONE;
            return;
        }

        /* 阶段 3 不推进角度（固定矢量），直接输出电压 */
        motor->electrical_angle = 0.0f;
        motor->target_voltage = volt;
        return;
    }

    motor->electrical_angle += vel_elec * CONTROL_DT;
    while (motor->electrical_angle >= TWO_PI) motor->electrical_angle -= TWO_PI;
    while (motor->electrical_angle < 0.0f)    motor->electrical_angle += TWO_PI;
    motor->target_voltage = volt;
}

/* 速度控制：速度误差 → Velocity PID → target_voltage */
/* 速度环：PID + 静摩擦前馈。
   前馈项 = FF × v/(|v|+KNEE)，v 是【目标】速度（不是实测）。
   用目标而不是实测有两个理由：
     ① 实测在低速时本身就是被粘滑污染的（停了→0，动了→超调），拿它做前馈
        会跟着一起振；目标速度是干净的给定，方向明确。
     ② 前馈的物理含义就是"抵消已知的、与运动方向相反的摩擦力"，方向由
        【想去哪】决定，不由"此刻在哪"决定。
   式子用 v/(|v|+KNEE) 而不是 sign(v)：前者在 0 附近连续，位置环保持时
   target_velocity 围绕 0 抖动也不会让前馈在 ±FF 之间跳变（那会变成原地咔咔响）。
   详见 motor_config.h 的 MOTOR_FRICTION_FF_*。 */
static void Motor_VelocityControl(Motor_t *motor)
{
    float out = PID_Update(&motor->velocity_pid,
                           motor->target_velocity, motor->velocity,
                           CONTROL_DT);

#if MOTOR_FRICTION_FF_ENABLE
    {
        float v = motor->target_velocity;
        float ff = motor->friction_ff_volt * (v / (fabsf(v) + MOTOR_FRICTION_FF_KNEE));
        out += ff;
    }
#endif

    /* 前馈加在 PID 【外面】，PID 的 output_max 管不到它 —— 所以这里必须
       再夹一次，否则"前馈 + PID 饱和输出"能超过电压上限。
       这是把前馈放在外面的代价，不能省。 */
    if (out >  motor->velocity_volt_limit) out =  motor->velocity_volt_limit;
    if (out < -motor->velocity_volt_limit) out = -motor->velocity_volt_limit;

    motor->target_voltage = out;
}

/* 开环拖动 → 闭环速度 的交接控制。

   死锁问题：速度环冷启动时 PID 输出小，静摩擦让转子起不来，编码器不动，
   误差不减小，PID 输出就一直小 —— 卡死，最后被堵转保护掐断。
   开环 RAMP 能成功是因为它缓慢升压，电压一直涨到把转子拖动。

   所以这个函数做三件事：
     ① 用「缓慢升压」的开环旋转磁场把转子带起来（不依赖任何反馈）
     ② 升压过程本身就是一个电压扫描，不必事先知道"多少伏能拖动"
     ③ 等实测转速达到目标的 DRAG_CATCH_RATIO 后，把 target_voltage 交给
        速度 PID 接管（角度仍用开环推进的角度，交接瞬间连续、无跳变）

   motor->align_max_volt / motor->align_drag_ms 由串口 ALIGN 命令按电机设置。 */

/* 交接时把速度 PID 的输出预置成当前开环电压。
   不预置的话：交接瞬间闭环积分还是 0、输出还是 0，而上一拍还在施加
   align_max_volt —— 电压直接塌到 0，转子当场减速，要等误差重新长起来
   （还要穿过 PID 的上升时间）才补得回去。表现为交接处"掉一下速"，
   重则直接失速被堵转保护掐断。预置之后电压是连续的。
   顺带把「开环电压」和「闭环输出限幅」对齐：预置值会被 PID 的
   output_max/min 夹住，所以即使 align_max_volt 大于闭环限幅，
   交接也不会输出一个闭环后续维持不了的电压。 */
static void Motor_AlignHandover(Motor_t *motor, float volt)
{
    motor->velocity_pid.integral = 0.0f;   /* 先清掉可能残留的旧积分 */
    PID_PresetOutput(&motor->velocity_pid, volt,
                     motor->target_velocity - motor->velocity);
    motor->control_mode = MOTOR_CONTROL_VELOCITY;
}

static void Motor_AlignControl(Motor_t *motor)
{
    uint32_t elapsed = HAL_GetTick() - motor->mode_start_tick;
    float    vel_cmd = motor->target_velocity;
    float    dir     = (vel_cmd >= 0.0f) ? 1.0f : -1.0f;
    float    frac, volt, vel_elec;

    /* 电角速度 = 机械角速度 × 极对数。用 |pole_pairs| 保证符号只由方向决定 */
    vel_elec = vel_cmd * ((motor->pole_pairs >= 0.0f) ? motor->pole_pairs
                                                      : -motor->pole_pairs);

    /* ① 电压线性升到上限；拖动阶段共用 drag_ms 的前 70% 完成升压，
          留 30% 时间在满电压下继续拖动，确保有时间达到目标转速。 */
    frac = (float)elapsed / ((float)motor->align_drag_ms * 0.70f);
    if (frac > 1.0f) frac = 1.0f;
    volt = motor->align_max_volt * (dir >= 0.0f ? frac : -frac);

    /* ② 实测转速是否已跟上目标？ */
    if (elapsed < motor->align_drag_ms)
    {
        float need = vel_cmd * MOTOR_ALIGN_CATCH_RATIO;
        uint8_t caught = (dir > 0.0f) ? (motor->velocity >= need)
                                      : (motor->velocity <= need);
        if (caught)
        {
            Motor_AlignHandover(motor, volt);   /* 交接闭环，输出预置成 volt */
            return;
        }
    }
    else
    {
        /* 拖动超时仍未跟上 —— 不再无限期开环拖下去，交给闭环（由堵转保护兜底） */
        Motor_AlignHandover(motor, volt);
        return;
    }

    /* ③ 开环推进电角度，并施加当前斜坡电压 */
    motor->electrical_angle += vel_elec * CONTROL_DT;
    while (motor->electrical_angle >= TWO_PI) motor->electrical_angle -= TWO_PI;
    while (motor->electrical_angle < 0.0f)    motor->electrical_angle += TWO_PI;
    motor->target_voltage = volt;
}

/* 位置控制：位置误差（回绕处理）→ Position PID → 目标速度 → 速度控制 */
static void Motor_PositionControl(Motor_t *motor)
{
    float error = motor->target_angle - motor->mechanical_angle;
    float v, v_brake;

    /* 取最短路径：回绕到 [-π, π] */
    while (error > PI)       error -= TWO_PI;
    while (error < -PI)      error += TWO_PI;

    /* 直接把「已回绕的误差」交给 PID（以 0 为反馈量），避免原实现
       feedback = target_angle - error 在回绕瞬间给微分项造成 2π/dt 尖峰 */
    v = PID_Update(&motor->position_pid, error, 0.0f, CONTROL_DT);

    /* ---- 制动力曲线限速 ---------------------------------------------------
       纯 P 位置环隐含的命令减速度是 d(KP·e)/dt = KP·de/dt = −KP·v：
       v 到 20 rad/s 时等于 400 rad/s²（KP=20）。这台电机给不出这么大的减速度，
       于是转子在"给定已经停了"之后还在往前走 —— 误差反号，速度环再反向拉。
       实测表现就是 2~4.5° 过冲、随后 1~3.5s 才收回来（速度环 ωn≈2.7 rad/s、
       ζ≈0.14，是个阻尼很弱的振铃模态，见 Debug/posstep.ps1 的实测表）。

       这里把命令速度压到「从当前剩余距离出发、以 amax 减速恰好停在目标」
       所需的速度：v_brake = √(2·amax·|e|)，再不超过 vmax。
       这不是另外加一条曲线，只是把 P 律那句"我还能更快"改成
       "按刹车距离我最多只能这么快" —— 于是减速从一开始就是可实现的。

       ★ 无状态：每拍都从当前误差重算。所以 ZERO / SPD / 中途改目标 /
         模式来回切换全都自动正确，不存在"目标变了要重新规划"这类握手问题。
       ★ 大误差时 v_brake 远大于 P 律输出，所以起步速度与改动前【完全一样】，
         15~30ms 的起步延迟不受影响（v 被夹到 vmax 的时机也不变）。
       ★ 残余：误差小到 P 律重新成为约束（|e| ≲ amax/KP²）之后，最后一段
         仍由 P 项收尾 —— 所以终值精度 ≤0.08°（1 个编码器计数）保持不变。
         预期残余过冲约为原来的 1/3，不是零。 */
    v_brake = sqrtf(2.0f * motor->traj_amax * fabsf(error));
    if (v_brake > motor->traj_vmax) v_brake = motor->traj_vmax;
    if (v >  v_brake) v =  v_brake;
    if (v < -v_brake) v = -v_brake;

    /* ---- 最小逼近速度 -----------------------------------------------------
       制动力曲线治的是"太快停不住"，这一段治的是另一个病："太慢根本不动"。

       位置环逼近目标时 v_cmd → 0，速度环的误差随之 → 0，积分于是【泄放】，
       "顶出静摩擦所需的那约 1.5V"跟着消失，转子停在离目标几个计数处不动。
       实测：卡在离目标 5 个计数、ERR=0、PWM 有输出、命令速度仅 0.15 rad/s ——
       低于起转阈值，而堵转保护不会报（转子确实"没在转"）。同一原因也是每个
       动作结尾 1~2s 爬行的来源。

       给命令速度一个下限，速度环就被按在"持续克服摩擦"的工作点上：积分稳定在
       起转电压附近，而不是随 v_cmd→0 一起泄放掉。

       ★ 放在制动力曲线【之后】：近目标处两者会冲突（曲线要慢以防过冲，下限要
         快以防卡死）。冲突区间很窄 —— 死区 1.5 个计数(0.0023 rad)时，曲线在
         死区边缘允许 √(2×60×0.0023) = 0.53 rad/s，出厂下限 0.5 不超过它，
         两者在出厂值下并不打架。往上调 vmin 时要留意这条边界。
       ★ 死区内不加下限，交回纯 P 保持 —— 终值精度仍是 ≤1 个计数，
         最小可控步长不会因此变差。
       ★ 方向取【误差】的符号而不是 v 的符号：此处两者理论上同号，但模式切换等
         路径会清 PID 状态，用误差更稳。
       ★ 线性增益为 0：这只是个常数偏置，不像摩擦前馈那样抬高回路增益。
         对照实测：M1 FF 1.5 时前馈在 v≈0 的小信号增益是 F/K = 1.5/0.2 =
         7.5 V/(rad/s)，比速度环自己的 kp=0.05 大 150 倍，直接把位置环推进
         极限环（VEL 冲到 −32 rad/s）。下限没有这个问题。 */
    if (motor->traj_vmin > 0.0f && fabsf(error) > motor->traj_deadband)
    {
        if (error > 0.0f)
        {
            if (v < motor->traj_vmin) v = motor->traj_vmin;
        }
        else
        {
            if (v > -motor->traj_vmin) v = -motor->traj_vmin;
        }
    }

    motor->target_velocity = v;
    Motor_VelocityControl(motor);
}

/* FOC 输出：Vd = 0，Vq = target_voltage → 逆 Park → SVPWM → PWM */
static void Motor_FOC_Output(Motor_t *motor)
{
    float va, vb;

    /* ---- 热累积闸（不可关闭）----
       对输出电压平方做积分：out² × dt。持续大电压 → 快速累积；
       输出小或停机 → 按 MOTOR_THERMAL_DECAY 时间常数泄放。
       一旦超限就断电锁存，必须重新上电/清故障才能恢复。
       这一道不依赖堵转保护，也不依赖任何人"记得开保护"。 */
    {
        float out  = motor->target_voltage;
        float step = out * out * CONTROL_DT;

        motor->thermal_i2t += step;
        motor->thermal_i2t -= motor->thermal_i2t * (CONTROL_DT / MOTOR_THERMAL_DECAY);

        if (motor->thermal_i2t >= MOTOR_THERMAL_I2T_LIMIT)
        {
            motor->thermal_tripped = 1;
            Motor_SetError(motor, MOTOR_ERROR_THERMAL);   /* 内部 PWM 0 + EN 关断 */
            return;
        }
    }

    /* 正常控制是 Vd = 0、Vq = target_voltage。
       只有对齐基准时（CALSPIN 阶段3，align_use_vd）才反过来打在 d 轴上：
       d 轴就是「转子磁场方向」的定义，vd 打在哪里磁极就停在哪里，
       于是「估计 d 轴 ≡ 真实 d 轴」自动成立，zero_angle 不需要任何补偿。
       若改用 q 轴去对齐，磁场方向与估计 d 轴差 90°，记录下来的基准就凭空
       差 90°，闭环里 cos(δ)=0 —— 只发热、不出力矩、转子完全不动。 */
    {
        float out = motor->target_voltage;

        if (out >  motor->voltage_limit) out =  motor->voltage_limit;
        if (out < -motor->voltage_limit) out = -motor->voltage_limit;

        /* 调制比钳位：SVPWM 线性区上限是 bus/√3 ≈ 0.577×bus。
           请求超过它时 SVPWM 会把某一相顶到 0% 或 100%，此时若转子不动，
           绕组里就是纯直流 —— 只发热不出扭矩。这里提前把请求压回线性区，
           避免"占空比顶格 + 转子不动"这种最坏的发热组合。 */
        {
            float vmax = 0.55f * motor->bus_voltage;
            if (out >  vmax) out =  vmax;
            if (out < -vmax) out = -vmax;
        }

        if (motor->align_use_vd) { motor->vd = out;  motor->vq = 0.0f; }
        else                     { motor->vd = 0.0f; motor->vq = out;  }
    }

    FOC_InvPark(motor->vd, motor->vq, motor->electrical_angle,
                &motor->v_alpha, &motor->v_beta);

    /* 电压 → 标幺（除以母线电压） */
    va = motor->v_alpha / motor->bus_voltage;
    vb = motor->v_beta / motor->bus_voltage;

    FOC_SVPWM(va, vb, &motor->duty_u, &motor->duty_v, &motor->duty_w);
    Motor_SetPWM(motor, motor->duty_u, motor->duty_v, motor->duty_w);
}

/* 堵转保护：只要在「有输出电压」的情况下转子长时间不动，就判堵转并停机断电。
   这条保护是发热问题的直接补丁 —— 之前没有任何机制能发现「电机没转」，
   PID 会把输出顶到限幅并一直保持，堵转电流全部变成绕组热量。
   返回 1 表示已判堵转（调用者不要再输出）。 */
static uint8_t Motor_StallGuard(Motor_t *motor)
{
    float    out, moved, step;
    uint32_t now, elapsed;

    now = HAL_GetTick();

    /* ① 先累加本拍行程 —— 必须排在所有提前返回【之前】。
       原实现把累加语句放在了 `elapsed < TIMEOUT` 的提前返回之后，
       而那个返回在窗口没满时每拍都会命中，于是累加一辈子只执行一次
       （窗口满的那一拍）；紧接着"有位移就清零"又把结果抹掉。
       净效果：stall_travel 永远等于「最后一个窗口的净位移」，
       「真的卡死」和「原地振荡」根本区分不开 —— 而这正是加它的唯一目的。 */
    step = AngleWrapPi(motor->mechanical_angle - motor->stall_last_angle);
    motor->stall_last_angle = motor->mechanical_angle;
    if (step < 0.0f) step = -step;
    /* 丢掉不足 1 个编码器计数的步长：读数没变的那些拍本来就不携带位移信息，
       而 1kHz × 2s = 2000 拍，若把末位抖动也累进去能虚增到 3 rad，
       足以让这个判据彻底失效。 */
    if (step < MOTOR_STALL_STEP_DEADBAND_RAD) step = 0.0f;
    motor->stall_travel += step;

    /* 电压太小不足以产生有意义转矩，不参与判定（避免零输出时误报） */
    out = motor->target_voltage;
    if (out < 0.0f) out = -out;

    /* ② 三种「本轮不判定」的情形，一律重置整个窗口（含行程累加） */
    if (!motor->stall_guard_enable ||
        /* 电压软启动斜坡期间：斜坡本身就是缓慢升压，转子还在被逐渐带动，
           此时判堵转是误报 */
        (motor->ramp_volt != 0.0f &&
         (now - motor->ramp_start_tick) < motor->ramp_ms) ||
        (out < MOTOR_STALL_MIN_VOLT))
    {
        motor->stall_tick       = now;
        motor->stall_angle      = motor->mechanical_angle;
        motor->stall_last_angle = motor->mechanical_angle;
        motor->stall_travel     = 0.0f;
        return 0;
    }

    elapsed = now - motor->stall_tick;
    if (elapsed < MOTOR_STALL_TIMEOUT_MS)
        return 0;                       /* 窗口未满，继续累加行程 */

    /* 窗口满：看净位移 */
    moved = AngleWrapPi(motor->mechanical_angle - motor->stall_angle);
    if (moved < 0.0f) moved = -moved;

    /* 判定堵转需要同时满足：
       ① 净位移小（没有真正转到新位置）
       ② 累计行程也小（不是在原地振荡）
       两个都要，缺一不可：只看 ① 会把"原地振荡"（FOC 反馈极性错时的典型表现）
       误判成堵转；只看 ② 则低速正常旋转会被误伤。 */
    if (moved < MOTOR_STALL_MIN_MOVE_RAD &&
        motor->stall_travel < MOTOR_STALL_TRAVEL_LIMIT)
    {
        Motor_SetError(motor, MOTOR_ERROR_STALL);   /* 里面会 PWM 0 + EN 关断 */
        return 1;
    }

    /* 本窗口内有真实运动 ⇒ 滑动窗口重来：重新丈量接下来的 2 秒。
       （行程累加清零，否则长时间正常运转会把累加值堆到上限，
         之后一旦真的卡死反而判不出来。） */
    motor->stall_tick       = now;
    motor->stall_angle      = motor->mechanical_angle;
    motor->stall_last_angle = motor->mechanical_angle;
    motor->stall_travel     = 0.0f;
    return 0;
}

/* 单电机实时控制周期（TIM3 ISR，仅用缓存角度，无 I2C） */
void Motor_Update(Motor_t *motor)
{
    if (motor->state == MOTOR_STATE_ERROR || motor->state == MOTOR_STATE_DISABLED)
        return;

    switch (motor->state)
    {
        case MOTOR_STATE_INIT:
            Motor_SetPWM(motor, 0.0f, 0.0f, 0.0f);
            break;

        case MOTOR_STATE_CALIBRATION:
            Motor_CalibrationOutput(motor);
            break;

        case MOTOR_STATE_VERIFY:
            /* 校准自检：在已记录的 zero_angle 上施加 +q 轴电压（vd=0 → 矢量超前
               转子 90° 电角度）。换相正确时这一步必然产生转矩、转子会动；
               若打在 d 轴上（基准错误）则只有保持力、转子不动 —— 由
               Motor_CalibrationVerify() 判定并报错。 */
            Motor_UpdateAngle(motor);
            Motor_FOC_Output(motor);
            break;

        case MOTOR_STATE_READY:
            Motor_SetPWM(motor, 0.0f, 0.0f, 0.0f);
            break;

        case MOTOR_STATE_RUNNING:
            /* 「对齐用 d 轴」只属于 CALSPIN 阶段3。离开该模式后必须清掉：
               否则下一次进速度环/位置环会把电压打到 d 轴上 —— 只发热不出力矩。 */
            if (motor->control_mode != MOTOR_CONTROL_ALIGNZERO)
                motor->align_use_vd = 0;

            /* 开环模式必须绕开 Motor_UpdateAngle()。
               这是本项目最隐蔽的一个 bug：Motor_UpdateAngle() 会把
               motor->electrical_angle 用编码器角度覆写一遍，而它排在
               control_mode 分派之前，所以开环里自己累加的 electrical_angle
               每个中断周期都被冲掉 —— 我原以为在施加"静止矢量"，
               实际施加的是一个跟着编码器噪声乱跳的矢量（噪声 × 极对数7）。
               后果：矢量乱摆 = 等效交流，不出净扭矩只发热，
               表现为"低电压完全不动、高电压偶尔抖一下"。 */
            if (motor->control_mode == MOTOR_CONTROL_OPEN)
            {
                Motor_UpdateVelocity(motor);     /* 开环也要测速，方便观察 */
                Motor_OpenLoop(motor);           /* 自己维护 electrical_angle */
                if (Motor_StallGuard(motor))
                    break;
                Motor_FOC_Output(motor);
                break;
            }

            /* ALIGN 同样是「自己维护角度」的模式，必须绕开 Motor_UpdateAngle()，
               理由与开环完全相同（见上面那段注释）。 */
            if (motor->control_mode == MOTOR_CONTROL_ALIGN)
            {
                Motor_UpdateVelocity(motor);
                Motor_AlignControl(motor);       /* 可能在此切到 VELOCITY */
                if (Motor_StallGuard(motor))
                    break;
                Motor_FOC_Output(motor);
                break;
            }

            /* 旋转式零位校准：全程自己维护角度，不依赖旧的 zero_angle */
            if (motor->control_mode == MOTOR_CONTROL_ALIGNZERO)
            {
                Motor_UpdateVelocity(motor);
                Motor_AlignZeroControl(motor);
                if (motor->state != MOTOR_STATE_RUNNING)
                    break;                       /* 校准完成，已停机 */
                Motor_FOC_Output(motor);
                break;
            }

            Motor_UpdateAngle(motor);
            Motor_UpdateVelocity(motor);

            switch (motor->control_mode)
            {
                case MOTOR_CONTROL_VELOCITY: Motor_VelocityControl(motor); break;
                case MOTOR_CONTROL_POSITION: Motor_PositionControl(motor); break;
                default: break;
            }
            /* 先查堵转：一旦判定就停机，不再输出 */
            if (Motor_StallGuard(motor))
                break;

            Motor_FOC_Output(motor);
            break;

        default:
            break;
    }
}

/* TIM3 周期中断回调：执行双电机实时控制 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        Motor_Control_Loop();
    }
}
