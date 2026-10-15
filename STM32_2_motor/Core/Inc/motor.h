#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 数学常量 */
#define PI                 3.1415926536f
#define TWO_PI             6.283185307f
#define DEG_TO_RAD         0.01745329252f
#define RAD_TO_DEG         57.29577951f

typedef enum
{
    MOTOR_STATE_DISABLED = 0,
    MOTOR_STATE_INIT,
    MOTOR_STATE_CALIBRATION,
    MOTOR_STATE_VERIFY,       /* 校准后自检：施加 +q 电压确认转子真的会动 */
    MOTOR_STATE_READY,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_ERROR
} MotorState_t;

typedef enum
{
    MOTOR_CONTROL_OPEN = 0,
    MOTOR_CONTROL_VELOCITY,
    MOTOR_CONTROL_POSITION,

    /* 开环拖动 -> 闭环速度 的交接模式。
       为什么需要它：闭环速度环冷启动会死锁 —— PID 输出小 -> 静摩擦起不来
       -> 编码器不动 -> 误差不减小 -> PID 输出一直小 -> 死锁，最后被堵转保护掐断。
       开环 RAMP 之所以成功，正是因为它缓慢升压，电压一直涨到转子被拖动。
       所以正确做法是：先用开环把转子带起来，等实测转速跟上目标后再交接闭环。
       串口： M2 ALIGN <目标速度 rad/s> [最大电压V] [拖动秒数] */
    MOTOR_CONTROL_ALIGN,

    /* 旋转式零位校准。
       为什么旧的校准不可靠：它从静止开始施加一个固定电压矢量把转子"拉"过来，
       但那个矢量走的是 FOC 输出链，而当时输出链正被"角度被覆写"的 bug 破坏，
       所以拉到的位置是随机的，记录的 zero_angle 从来没准过。

       新做法：先在开环把转子转起来（这一步不依赖 zero_angle），
       再把电角速度平滑降到 0，转子会跟着减速并最终与磁场同步对齐。
       对齐用【纯 d 轴】电压，于是「估计 d 轴 ≡ 真实 d 轴」自动成立，
       zero_angle = 机械角，不做任何补偿。

       这一步还顺带定出【换相极性的符号】：拖动时磁场恒朝 θe 增大方向转、
       转子同步跟随，所以 sign(Δθm) 必须等于 sign(direction×pole_pairs)。
       反了就把 pole_pairs 取负（等效串口 DIR -1）。

       串口： M2 CALSPIN [电压V] [电角速度rad/s] */
    MOTOR_CONTROL_ALIGNZERO
} MotorControlMode_t;

typedef enum
{
    MOTOR_ERROR_NONE        = 0x00,
    MOTOR_ERROR_I2C         = 0x01,
    MOTOR_ERROR_ENCODER     = 0x02,
    MOTOR_ERROR_CALIBRATION = 0x04,
    MOTOR_ERROR_OVERSPEED   = 0x08,
    MOTOR_ERROR_CONTROL     = 0x10,
    MOTOR_ERROR_STALL       = 0x20,  /* 堵转：加了电压但转子长时间不动 */
    MOTOR_ERROR_THERMAL     = 0x40   /* 热累积超限：持续大电压输出，主动断电保护 */
} MotorError_t;

typedef struct
{
    /* 硬件资源 */
    TIM_HandleTypeDef *pwm_timer;
    uint32_t pwm_u_channel;
    uint32_t pwm_v_channel;
    uint32_t pwm_w_channel;
    GPIO_TypeDef *en_port;
    uint16_t en_pin;
    I2C_HandleTypeDef *i2c;
    uint8_t encoder_addr;

    /* 电机参数 */
    float pole_pairs;
    float direction;
    float voltage_limit;
    float bus_voltage;

    /* 编码器计数方向：+1 = 计数增大对应电角度增大方向，-1 = 相反。
       ★ 这是硬件装向决定的，必须实测。测法（开环，安全）：
           M2 OLTEST 2 2  → 磁场朝 θe 增大方向转
         看 STATUS 里的 ANGLE：应当【增大】。若 ANGLE 在减小，
         说明编码器计数方向与换相正向相反，这里就取 -1。

       为什么必须在这一层修、而不能靠翻 pole_pairs（DIR -1）糊过去：
       翻 pole_pairs 只改 θe 的符号约定，同时把【速度反馈】的符号也留在了
       原来的方向，于是换相基准"看起来对了"、速度环却成了正反馈 —— 一给
       目标就飞车。把编码器方向摆正，换相与测速同时正确。
       详见 motor_config.h 的 MOTOR2_ENCODER_SIGN。 */
    float encoder_sign;

    /* 编码器。mechanical_angle 由主循环的 Motor_ReadEncoder() 写、
       由 TIM3 中断里的 Motor_Update() 读，属跨上下文共享，必须 volatile：否则编译器
       可能把读取缓存在寄存器里或在优化时重排，导致速度环读到不一致的角度。
       encoder_raw 不标 volatile：它是 AS5600_ReadRawAngle() 的输出缓冲区，
       加限定符会与函数签名冲突；它在主循环写完、同一函数内即转成角度，中断不读它。 */
    uint16_t encoder_raw;
    volatile float mechanical_angle;
    float electrical_angle;

    /* 换相基准：θe = direction × pole_pairs × (θm − 此值)。
       由校准（CALIBRATION / CALSPIN）写入，表示「编码器读数」与「转子磁极」
       之间的固定物理关系。★ 偏一点 FOC 就废：θe 凭空偏 7×δ（7 极对数），
       δ 小则没力矩、δ≈26°机械角就够变成负转矩。改它必须重新对齐，不能随手写。 */
    float zero_angle;

    /* 位置环零点： POS <度> 以它为零点，即 target_angle = position_offset + 度。
       ★ 为什么必须和 zero_angle 分成两个变量 —— 它们语义完全不同，却曾经共用
         一个 zero_angle，于是「把当前位置设为位置环零点」这条命令顺手把换相
         基准也改了。实测后果：发一条 M1 ZERO 之后再 SPD 1，TARGET=+1.00 而电机
         以约 -10 rad/s 反向飞车（PWM 打到限幅）。因为 ZERO 把 zero_angle 写成
         当前转子位置，而此时转子并不在 d 轴对齐点上，电角度凭空偏了 7×δ。

         分开之后：zero_angle 只归换相，任何用户命令都不该碰；
         position_offset 只是「用户想把哪儿当作 0 度」，纯人机约定，随便改都不
         影响换相。写入时机只有两处 —— 校准完成时随 zero_angle 一起初始化
         （这样 POS 的语义与拆分前完全一致），以及 ZERO 命令。 */
    float position_offset;
    volatile uint32_t encoder_stamp;    /* 上次成功采样的时刻 (ms) */
    volatile uint32_t encoder_error_count; /* 连续读失败计数（故障容错） */
    uint8_t bus_recovered;                 /* 本轮连续失败是否已尝试过总线恢复，避免反复复位 */
    uint32_t bus_recover_count;            /* 累计总线恢复次数，超过上限判硬件故障 */

    /* 采样节流时间戳（主循环上下文专用，ISR 不访问）。必须每电机独立 ——
       原来是文件级 static，主循环里 motor1 先读并更新时间戳，紧接着的 motor2
       就因「同一毫秒内」被跳过：电机2 的 mechanical_angle 长期不更新，
       于是速度恒为 0、堵转保护立刻跳闸、ALIGN 的「已跟上目标」判据永不成立。
       两个电机是分别控制的，采样节流也必须各记各的。 */
    uint32_t encoder_sample_tick;

    /* 测速窗口（固定时间窗，仅 TIM3 中断访问）。
       为什么不在每次编码器采样上做微分：AS5600 是 12 位，1 个计数 = 0.001534 rad，
       而采样间隔仅 1ms，于是「1 个计数的抖动」就产生 1.5 rad/s 的假速度 —— 与目标
       速度同量级，速度环必然被噪声激成振荡（实测速度在 +34~-15 rad/s 间乱跳）。
       改成「每次跨越固定窗口才用窗口两端的角度差算速度」后，量化噪声按 1/窗口
       下降，且天然带低通效果。

       ★ 窗口两端锚在【编码器采样时刻】而不是中断观察时刻：主循环会被阻塞式 I2C
         和串口发送拖住，观察时刻的抖动会直接进分母变成速度噪声。
         窗口起点存 encoder_stamp 的值，于是分母 = 两次采样时刻之差，是真实的。 */
    uint32_t velocity_window_stamp;     /* 本测速窗口起点那次采样的 encoder_stamp */
    float    velocity_window_angle;     /* 本测速窗口起始时的机械角 (rad) */

    /* 堵转保护（仅 TIM3 中断访问）。见 motor_config.h 的 MOTOR_STALL_* 说明。 */
    uint32_t stall_tick;                /* 本轮堵转检测窗口起始时刻 (ms) */
    float    stall_angle;               /* 本轮窗口起始时的机械角 (rad) */
    float    stall_last_angle;          /* 上一拍的机械角 (rad)，用于逐拍累加行程。
                                           少了它就没法"逐拍累加" —— 只能拿窗口
                                           两端求差，那算出来是净位移不是行程。 */
    float    stall_travel;              /* 本轮窗口内累计行程 (rad)，恒为非负。
                                           只看净位移会误判：转子在某个位置来回振荡
                                           （FOC 反馈极性错时正是这样）净位移接近 0，
                                           但实际"动得很厉害"。用累计行程可区分
                                           「真的卡死」和「在振荡」。
                                           ★ 必须每拍累加（见 Motor_StallGuard）：
                                             只在窗口边界累加一次的话，这个量永远等于
                                             "最后一个窗口的净位移"，等于白加。 */

    /* 热累积保护（仅 TIM3 中断访问）。
       为什么需要它：堵转保护看的是"转不转"，但还有一种更隐蔽的发热工况 ——
       转子缓慢蠕动/来回振荡，位移够大骗过了堵转保护，而绕组里始终有大电流。
       这里对输出电压做 I²t 式的积分：只要持续输出大电压就累积，
       输出小或停机就按时间常数泄放。累积到上限直接断电并锁存故障。
       它是最后一道闸，不依赖任何人的"记得开保护"。 */
    float    thermal_i2t;               /* 累积量，上限见 MOTOR_THERMAL_I2T_LIMIT */
    uint8_t  thermal_tripped;           /* 1 = 已因过热锁存，需手动清除 */

    /* 校准 */
    uint32_t calib_start_tick; /* 校准起始时刻 (ms) */

    /* ---- 校准自检（VERIFY）状态。必须每电机独立 ----
       原先是函数级 static：两个电机共用同一份「起始角 / armed」，
       后进入 VERIFY 的电机会把先进入的那个踩掉（跳过自检，或拿另一个
       电机的起始角做比较），等于自检根本没做。 */
    float    verify_last_angle;  /* 上一拍机械角 (rad) */
    float    verify_travel;      /* VERIFY 期间【带符号】累计行程 (rad)：
                                    折算到「θe 增大方向为正」后 ≥ 阈值才算通过。
                                    只看 |Δθ| 是不够的 —— 基准差 180° 时 +q 电压
                                    同样产生转矩（方向相反），转子照样会动。 */
    uint8_t  verify_armed;       /* 1 = 正在施加 +q 电压自检 */
    uint8_t  polarity_tried;     /* 已试过的换相极性位掩码：bit0 = 正,pole_pairs，
                                    bit1 = 负。两种都试过仍反向 → 判硬故障，
                                    保证「自动翻极性重试」最多发生一次，不会死循环。 */

    /* ---- 旋转式零位校准（CALSPIN）状态。同样必须每电机独立 ----
       原先也是 static：电机1 在升速阶段会不停重置电机2 的「静止计时」，
       电机2 的稳定判据于是永远不成立或提前成立。 */
    uint32_t calspin_run_tick;    /* 本次运行对应的 mode_start_tick，用于识别「新一次 CALSPIN」 */
    float    calspin_last_angle;  /* 上一拍机械角 (rad)，用于逐拍累加拖动行程 */
    float    calspin_drag_travel; /* 拖动阶段带符号累计行程 (rad)。
                                     拖动时磁场恒朝 θe 增大方向转、转子同步跟随，
                                     故 sign(Δθm) 就是换相极性 sign(k) —— 用它定符号。 */
    float    calspin_settle_angle;/* 阶段3 静止检测窗口起点角 */
    uint32_t calspin_settle_tick; /* 阶段3 静止检测窗口起点时刻 */
    uint8_t  calspin_init;        /* 阶段3 静止窗口是否已初始化 */

    /* 非 0 时 FOC 输出把 target_voltage 打到 d 轴（vd）而不是 q 轴（vq）。
       只有 CALSPIN 阶段3 的对齐会置位：d 轴就是「转子磁场方向」的定义，
       vd 打在哪里磁极就停在哪里 ⇒「估计 d 轴 ≡ 真实 d 轴」自动成立，
       zero_angle 不需要任何补偿。用 q 轴对齐则磁场与估计 d 轴差 90°，
       记录下来的基准凭空差 90°，闭环里 cos(δ)=0 —— 只发热、不出力矩。 */
    uint8_t  align_use_vd;

    /* ---- 运行时可调参数（串口命令写、中断读）----
       ★ 这些原先都是 motor_control.c 里的文件级全局变量，被两个电机【共用】：
         给电机2 发一条 "M2 RAMP 6 1.5 3000"，电机1 正在跑的斜坡会同时被改掉；
         "M2 GUARD 0" 会连电机1 的堵转保护一起关掉。两个电机是分别控制的，
         参数就必须各记各的。 */
    uint8_t  en_active_high;      /* EN 有效极性：1 = 高有效。放结构体里之后，
                                     所有 EN 操作都读它，不再到处写
                                     (motor == &motor1) ? ... : ... 的三元判断。 */
    uint8_t  stall_guard_enable;  /* 堵转保护开关（默认 1 = 开） */

    /* 静摩擦前馈 (V)，运行时可由串口 "M2 FF 1.5" 整定。
       治低速粘滑：这台电机要约 1.5V 才起转，靠 PID 积分去"发现"它要好几秒，
       现象就是 ≲1 rad/s 时走一下停一下。直接把已知的静摩擦量前馈出去，
       PID 就只剩粘性阻尼和惯性要管。曲线与取值依据见 motor_config.h
       的 MOTOR_FRICTION_FF_* 说明。0 = 关闭该电机的前馈。
       ★ 每电机独立：两台装配不同、起转阈值不同，必须分别整定。 */
    float    friction_ff_volt;

    /* 速度环输出的【有效】电压上限 (V) = min(voltage_limit, MOTOR_VEL_VOLT_LIMIT)。
       为什么不直接用 voltage_limit：它是配置值（12V 母线），不是安全上限。
       Motor_ApplyConfig 里把它夹到 MOTOR_VEL_VOLT_LIMIT 后【同时】喂给
       velocity_pid.output_max 和这里。存成字段是为了让前馈的钳位与 PID 的
       output_max 永远同源 —— 前馈加在 PID 外面，PID 的 output_max 管不到它，
       若这里各写一份表达式，将来改了一处就会出现"前馈 + 饱和 PID"超过上限。 */
    float    velocity_volt_limit;
    float    openloop_vel;        /* 开环强拖电角速度 (rad/s)；0 = 不强制推进角度 */
    float    openloop_volt;       /* 开环强拖电压 (V) */
    float    ramp_volt;           /* 电压斜坡目标 (V)；0 = 斜坡未启用 */
    float    ramp_vel;            /* 斜坡期间的电角速度 (rad/s) */
    uint32_t ramp_ms;             /* 升压时长 (ms) */
    uint32_t ramp_start_tick;     /* 斜坡起始时刻 (ms) */
    float    align_max_volt;      /* ALIGN / CALSPIN 的最大电压 (V) */
    uint32_t align_drag_ms;       /* ALIGN 开环拖动时长 (ms) */
    float    calspin_vel;         /* CALSPIN 起始电角速度 (rad/s) */

    /* 当前控制模式的起始时刻 (ms)。ALIGN 模式用它算斜坡进度。 */
    uint32_t mode_start_tick;

    /* 速度 */
    float velocity;
    float target_velocity;

    /* 位置 */
    float target_angle;

    /* 位置环【制动力曲线限速】的两个每电机参数。
       位置环是纯 P（POS_KD=0），隐含的命令减速度是 KP·v —— v 到 20 rad/s 时
       等于 400 rad/s²，这台电机给不出来，于是转子在"给定已停"之后还在走，
       这就是实测 2~4.5° 过冲 + 1~3.5s 振铃收尾的根因。把命令速度压到
       "从当前剩余距离以 amax 恰好刹停"的曲线上，减速从一开始就可行。
       详见 Motor_PositionControl 里的实现与 Debug/posstep.ps1 的实测表。

       ★ 这两个值只在 TIM3 中断里被读，主循环只写 —— 32 位对齐的读写是原子的，
         而且限速器【无状态】（每拍从当前误差重算），所以不需要 volatile，
         也没有"目标变了要通知中断重新规划"的握手问题。
       ★ traj_vmax 必须 ≤ MAX_VELOCITY：位置环输出最后会夹到 ±MAX_VELOCITY，
         写更大只是让这条曲线被白白削平。 */
    float traj_vmax;      /* 巡航速度上限 (rad/s) */
    float traj_amax;      /* 减速度上限 (rad/s²)，本项目过冲的正主 */

    /* 位置环【最小逼近速度】的两个每电机参数。治的是另一个病：
       位置环逼近目标时 v_cmd → 0，速度环的误差随之 → 0，于是积分【泄放】，
       "顶出静摩擦所需的那约 1.5V"跟着消失，转子就停在离目标几个计数处不动。
       实测：卡在离目标 5 个计数、ERR=0、PWM 有输出、命令速度仅 0.15 rad/s ——
       足够小到破不了起转阈值，而堵转/热保护都不会报（转子确实"没在转"）。
       同一原因也造成每个动作结尾 1~2s 的爬行。

       这里给命令速度一个下限：误差绝对值大于 traj_deadband 时，命令速度不低于
       traj_vmin（方向随误差）。于是速度环被按在一个"持续克服摩擦"的工作点上，
       积分稳定在起转电压附近而不是泄放到 0。它【线性增益为 0】—— 只是个常数
       偏置，不像摩擦前馈那样抬高回路增益（实测 M1 FF 1.5 直接把位置环推进
       极限环，VEL 冲到 −32 rad/s，详见 memory 与 STRATEGY 注释）。

       ★ traj_deadband 必须 ≥ 1 个编码器计数（0.001534 rad）：小于它就是在跟
         量化噪声打架，转子停在计数边界时会来回跳。
       ★ 死区内不加上限，交回纯 P 保持 —— 所以终值精度仍是 ≤1 个计数，
         这一步不会把"最小可控步长"变差。 */
    float traj_vmin;      /* 最小逼近速度 (rad/s)，0 = 关闭该功能 */
    float traj_deadband;  /* 死区半宽 (rad)，其外才施加上限 */

    /* 当前版本控制量 */
    float target_voltage;
    float vd;
    float vq;
    float v_alpha;
    float v_beta;

    /* PWM */
    float duty_u;
    float duty_v;
    float duty_w;

    /* ADC 电流采样扩展预留（当前硬件无采样，不参与控制） */
    uint16_t adc_raw_u;
    uint16_t adc_raw_v;
    uint16_t adc_raw_w;
    float current_offset_u;
    float current_offset_v;
    float current_offset_w;
    float ia;
    float ib;
    float ic;
    float i_alpha;
    float i_beta;
    float id;
    float iq;
    float target_id;
    float target_iq;
    uint8_t current_valid;

    /* PID */
    PID_t position_pid;
    PID_t velocity_pid;

    /* 控制模式：由主循环/串口命令写，TIM3 中断里的 Motor_Update() 读 */
    volatile MotorControlMode_t control_mode;

    /* 状态。同样跨上下文：主循环负责状态迁移，中断按状态选择输出 */
    volatile uint8_t encoder_ok;
    volatile uint8_t driver_enabled;
    volatile MotorState_t state;

    /* 故障 */
    volatile uint32_t error;

} Motor_t;

extern Motor_t motor1;
extern Motor_t motor2;

void Motor_Init(Motor_t *motor,
                TIM_HandleTypeDef *timer,
                uint32_t ch_u, uint32_t ch_v, uint32_t ch_w,
                GPIO_TypeDef *en_port, uint16_t en_pin,
                I2C_HandleTypeDef *i2c, uint8_t encoder_addr);

void Motor_PWM_Start(Motor_t *motor);
void Motor_SetPWM(Motor_t *motor, float duty_u, float duty_v, float duty_w);
/* 按 motor->en_active_high 把 EN 写成「使能 / 关断」电平。
   唯一的 EN 写入口：原先每处各自判断极性，漏掉一处（Motor_SetError、
   Motor_Init 都漏了）就会在低有效板子上把 EN 拉成使能态 —— 报警时反而通电。 */
void Motor_WriteEN(Motor_t *motor, uint8_t enable);
void Motor_Enable(Motor_t *motor);
void Motor_Disable(Motor_t *motor);
void Motor_SetError(Motor_t *motor, uint32_t error);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
