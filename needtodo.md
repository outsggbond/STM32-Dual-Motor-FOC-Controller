**做什么、文件怎么分、数据怎么流、函数怎么写、每个阶段完成什么、最终代码达到什么状态。**

# STM32_2_motor 双电机控制软件设计

## 1. 项目最终目标

工程最终实现：

```text
STM32F103C8T6
│
├── 电机1
│   ├── TIM1 CH1/CH2/CH3
│   ├── PB15 EN
│   ├── I2C1
│   └── AS5600
│
└── 电机2
    ├── TIM2 CH1/CH2/CH3
    ├── PA3 EN
    ├── I2C2
    └── AS5600
```

软件完成：

```text
硬件初始化
    ↓
PWM初始化
    ↓
编码器初始化
    ↓
电机参数初始化
    ↓
编码器校准
    ↓
电角度计算
    ↓
FOC控制
    ↓
三相PWM输出
    ↓
双电机独立控制
    ↓
串口参数控制与状态监视
    ↓
故障检测与保护
```

最终两个电机使用**同一套控制代码**，但每个电机拥有独立参数、独立目标值、独立状态和独立控制结果。

---

# 2. 软件总体架构

整个程序分成 5 层：

```text
┌────────────────────────────────────────────┐
│              应用控制层 Application          │
│                                            │
│ motor_control.c                            │
│ 目标位置 / 目标速度 / FOC / 控制状态         │
└─────────────────────┬──────────────────────┘
                      │
┌─────────────────────▼──────────────────────┐
│                 电机层 Motor                 │
│                                            │
│ motor.c                                    │
│ 电机对象 / PWM / EN / 基本状态               │
└───────────────┬────────────────────────────┘
                │
       ┌────────┴────────┐
       │                 │
┌──────▼───────┐ ┌──────▼────────┐
│  AS5600驱动  │ │  PWM硬件接口   │
│  as5600.c    │ │  TIM1 / TIM2  │
└──────┬───────┘ └───────────────┘
       │
       ▼
   I2C1 / I2C2

同时：

┌────────────────────────────────────────────┐
│             系统任务层                     │
│                                            │
│ timer / uart / fault                       │
│ TIM3控制周期 + USART调试                   │
└────────────────────────────────────────────┘
```

---

# 3. 最终目录结构

在 CubeMX 当前工程基础上增加：

```text
STM32_2_motor/
│
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── motor.h
│   │   ├── motor_config.h
│   │   ├── motor_control.h
│   │   ├── as5600.h
│   │   ├── foc.h
│   │   ├── pid.h
│   │   └── uart_debug.h
│   │
│   └── Src/
│       ├── main.c
│       ├── motor.c
│       ├── motor_control.c
│       ├── as5600.c
│       ├── foc.c
│       ├── pid.c
│       └── uart_debug.c
│
├── Drivers/
│   ├── CMSIS/
│   └── STM32F1xx_HAL_Driver/
│
├── STM32_2_motor.ioc
└── STM32F103C8TX_FLASH.ld
```

CubeMX 生成的：

```text
gpio.c
i2c.c
tim.c
usart.c
stm32f1xx_it.c
```

继续由 CubeMX 管理。

**用户自己编写的电机业务代码全部放在新建模块中。**

---

# 4. 各文件职责

| 文件                  | 职责                            |
| ------------------- | ----------------------------- |
| `main.c`            | 系统启动、外设初始化、启动控制系统             |
| `motor_config.h`    | 两个电机的硬件映射和基础参数                |
| `motor.h`           | Motor 对象、状态、接口定义              |
| `motor.c`           | PWM、EN、Motor对象基础操作            |
| `as5600.h/c`        | AS5600 I2C驱动、角度读取             |
| `foc.h/c`           | Clarke、Park、反Park、SVPWM等FOC算法 |
| `pid.h/c`           | 电流/速度/位置控制器                   |
| `motor_control.h/c` | 双电机控制流程、状态机、控制周期              |
| `uart_debug.h/c`    | 串口命令、参数修改、状态输出                |
| `stm32f1xx_it.c`    | CubeMX生成的中断入口                 |

---

# 5. 电机对象设计

核心数据结构统一成：

```c
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

    /* 编码器 */
    uint16_t encoder_raw;
    float mechanical_angle;
    float electrical_angle;
    float zero_angle;

    /* 速度 */
    float velocity;
    float target_velocity;

    /* 位置 */
    float target_angle;

    /* FOC */
    float ia;
    float ib;
    float ic;

    float i_alpha;
    float i_beta;

    float id;
    float iq;

    float target_id;
    float target_iq;

    float vd;
    float vq;

    float v_alpha;
    float v_beta;

    float duty_u;
    float duty_v;
    float duty_w;

    /* 状态 */
    uint8_t encoder_ok;
    uint8_t driver_enabled;

    /* 电机状态 */
    uint8_t state;

    /* 故障 */
    uint32_t error;

} Motor_t;
```

全局只创建：

```c
Motor_t motor1;
Motor_t motor2;
```

---

# 6. 电机配置设计

`motor_config.h` 定义硬件资源。

## Motor1

```c
#define MOTOR1_PWM_TIMER      (&htim1)

#define MOTOR1_PWM_U          TIM_CHANNEL_1
#define MOTOR1_PWM_V          TIM_CHANNEL_2
#define MOTOR1_PWM_W          TIM_CHANNEL_3

#define MOTOR1_EN_PORT        GPIOB
#define MOTOR1_EN_PIN         GPIO_PIN_15

#define MOTOR1_I2C            (&hi2c1)
#define MOTOR1_ENCODER_ADDR   0x36
```

对应：

```text
PA8  → U
PA9  → V
PA10 → W
PB15 → EN
PB8  → SCL
PB9  → SDA
```

## Motor2

```c
#define MOTOR2_PWM_TIMER      (&htim2)

#define MOTOR2_PWM_U          TIM_CHANNEL_1
#define MOTOR2_PWM_V          TIM_CHANNEL_2
#define MOTOR2_PWM_W          TIM_CHANNEL_3

#define MOTOR2_EN_PORT        GPIOA
#define MOTOR2_EN_PIN         GPIO_PIN_3

#define MOTOR2_I2C            (&hi2c2)
#define MOTOR2_ENCODER_ADDR   0x36
```

对应：

```text
PA0 → U
PA1 → V
PA2 → W
PA3 → EN
PB10 → SCL
PB11 → SDA
```

---

# 7. 电机状态机

每个电机独立维护状态。

```c
typedef enum
{
    MOTOR_STATE_DISABLED = 0,
    MOTOR_STATE_INIT,
    MOTOR_STATE_CALIBRATION,
    MOTOR_STATE_READY,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_ERROR
} MotorState_t;
```

状态流程：

```text
DISABLED
   │
   ▼
 INIT
   │
   ▼
CALIBRATION
   │
   ▼
 READY
   │
   ▼
RUNNING
```

异常：

```text
任何状态
   │
   ▼
 ERROR
   │
   ▼
 PWM关闭
 EN关闭
```

状态含义：

| 状态            | 功能           |
| ------------- | ------------ |
| `DISABLED`    | 电机完全关闭       |
| `INIT`        | 软件对象和外设准备    |
| `CALIBRATION` | 建立机械零位/电角度零位 |
| `READY`       | 初始化完成，等待目标   |
| `RUNNING`     | 正常闭环运行       |
| `ERROR`       | 发生故障，停止输出    |

---

# 8. main.c 设计

`main.c` 只负责系统启动，不负责具体 FOC 算法。

最终结构：

```c
int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();

    MX_I2C1_Init();
    MX_I2C2_Init();

    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

    MX_USART1_UART_Init();

    Motor_System_Init();

    HAL_TIM_Base_Start_IT(&htim3);

    while (1)
    {
        Motor_System_Task();
    }
}
```

其中：

```c
Motor_System_Init();
```

负责整个电机系统初始化。

---

# 9. Motor_System_Init()

初始化流程：

```text
Motor_System_Init()
       │
       ├── Motor1对象初始化
       │
       ├── Motor2对象初始化
       │
       ├── PWM启动
       │
       ├── EN保持关闭
       │
       ├── AS5600检测
       │
       ├── 编码器初始化
       │
       └── 两个电机进入 INIT
```

接口：

```c
void Motor_System_Init(void);
```

内部：

```c
Motor_Init(&motor1, ...);
Motor_Init(&motor2, ...);
```

---

# 10. Motor_Init()

接口设计：

```c
void Motor_Init(
    Motor_t *motor,
    TIM_HandleTypeDef *timer,
    uint32_t ch_u,
    uint32_t ch_v,
    uint32_t ch_w,
    GPIO_TypeDef *en_port,
    uint16_t en_pin,
    I2C_HandleTypeDef *i2c,
    uint8_t encoder_addr
);
```

完成：

```text
创建 Motor 对象
       ↓
保存 TIM
保存 PWM 通道
保存 EN GPIO
保存 I2C
保存 AS5600地址
       ↓
设置初始参数
       ↓
状态 = INIT
       ↓
PWM = 0
       ↓
EN = 0
```

---

# 11. PWM 模块设计

`motor.c` 只处理 PWM。

## 启动

```c
void Motor_PWM_Start(Motor_t *motor);
```

实际执行：

```c
HAL_TIM_PWM_Start(... U ...);
HAL_TIM_PWM_Start(... V ...);
HAL_TIM_PWM_Start(... W ...);
```

---

## 设置三相占空比

```c
void Motor_SetPWM(
    Motor_t *motor,
    float duty_u,
    float duty_v,
    float duty_w
);
```

输入范围：

```text
0.0 ~ 1.0
```

输出：

```text
CCR_U
CCR_V
CCR_W
```

关系：

```text
CCR = duty × ARR
```

---

# 12. EN控制设计

```c
void Motor_Enable(Motor_t *motor);
void Motor_Disable(Motor_t *motor);
```

Enable：

```text
PWM正常
+
EN = HIGH
```

Disable：

```text
PWM = 0
+
EN = LOW
```

任何错误都通过：

```c
Motor_Disable(motor);
```

关闭。

---

# 13. AS5600驱动设计

`as5600.c` 只负责传感器。

不参与：

```text
FOC
PID
PWM
状态机
```

只负责：

```text
I2C
 ↓
AS5600
 ↓
Raw Angle
 ↓
机械角度
```

---

# 14. AS5600接口

定义：

```c
HAL_StatusTypeDef AS5600_ReadRawAngle(
    I2C_HandleTypeDef *hi2c,
    uint8_t addr,
    uint16_t *raw
);
```

以及：

```c
HAL_StatusTypeDef AS5600_ReadAngle(
    I2C_HandleTypeDef *hi2c,
    uint8_t addr,
    float *angle
);
```

角度：

```text
0 ~ 4095
```

转换：

```text
mechanical_angle
=
raw / 4096 × 2π
```

或者保存为角度：

```text
0 ~ 360°
```

---

# 15. 编码器读取流程

每一个控制周期：

```text
Motor_Update()
     │
     ▼
AS5600_ReadRawAngle()
     │
     ▼
encoder_raw
     │
     ▼
mechanical_angle
     │
     ▼
direction
     │
     ▼
zero_angle
     │
     ▼
electrical_angle
```

电角度：

```text
θe = direction × pole_pairs × (θm - zero_angle)
```

然后归一化到：

```text
0 ~ 2π
```

---

# 16. 编码器异常处理

AS5600读取失败：

```text
I2C失败
 ↓
encoder_ok = 0
 ↓
error |= MOTOR_ERROR_ENCODER
 ↓
state = ERROR
 ↓
Motor_Disable()
```

AS5600正常：

```text
encoder_ok = 1
```

---

# 17. FOC模块设计

`foc.c` 专门负责数学运算。

包含：

```text
Clarke Transform
Park Transform
逆Park
PI电流控制
SVPWM
```

结构：

```text
三相电流
    ↓
Clarke
    ↓
αβ
    ↓
Park
    ↓
dq
    ↓
Id/Iq PI
    ↓
Vd/Vq
    ↓
逆Park
    ↓
Vα/Vβ
    ↓
SVPWM
    ↓
Duty U/V/W
```

---

# 18. FOC接口

```c
void FOC_Init(void);
```

---

## Clarke

```c
void FOC_Clarke(
    float ia,
    float ib,
    float ic,
    float *alpha,
    float *beta
);
```

---

## Park

```c
void FOC_Park(
    float alpha,
    float beta,
    float angle,
    float *d,
    float *q
);
```

---

## 逆Park

```c
void FOC_InvPark(
    float d,
    float q,
    float angle,
    float *alpha,
    float *beta
);
```

---

## SVPWM

```c
void FOC_SVPWM(
    float alpha,
    float beta,
    float *duty_u,
    float *duty_v,
    float *duty_w
);
```

---

# 19. 一个完整 FOC 周期

每个电机：

```text
读取 AS5600
      ↓
获取机械角
      ↓
机械角 → 电角度
      ↓
获取电流
      ↓
Clarke
      ↓
Park
      ↓
Id / Iq
      ↓
PI
      ↓
Vd / Vq
      ↓
逆Park
      ↓
SVPWM
      ↓
Duty U/V/W
      ↓
TIM CCR
```

---

# 20. 当前硬件没有电流采样时的控制结构

你的当前硬件配置里只有：

```text
AS5600
```

没有配置：

```text
ADC电流采样
```

因此当前工程的软件数据链不能设计成完整的**电流闭环 FOC**。

当前实际控制结构定义为：

```text
AS5600
 ↓
机械位置
 ↓
电角度
 ↓
位置/速度控制
 ↓
电压矢量
 ↓
SVPWM
 ↓
三相PWM
```

即：

```text
位置控制
       ↓
速度控制
       ↓
Vq
       ↓
FOC空间矢量
```

不包含：

```text
Ia/Ib/Ic
Id/Iq电流反馈
Id/Iq PI闭环
```

因为当前硬件没有对应反馈通道。

---

# 21. 当前版本控制层设计

因此 `motor_control.c` 的核心控制链定义为：

```text
Target
  │
  ▼
位置控制器
  │
  ▼
目标速度
  │
  ▼
速度控制器
  │
  ▼
目标 Vq
  │
  ▼
FOC
  │
  ▼
PWM
```

两个电机分别执行。

---

# 22. PID模块

`pid.c` 负责通用 PID。

结构体：

```c
typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;

    float previous_error;

    float output_min;
    float output_max;

} PID_t;
```

接口：

```c
void PID_Init(
    PID_t *pid,
    float kp,
    float ki,
    float kd,
    float min,
    float max
);
```

计算：

```c
float PID_Update(
    PID_t *pid,
    float target,
    float feedback,
    float dt
);
```

---

# 23. 电机控制参数

每个 Motor 对象拥有：

```c
PID_t position_pid;
PID_t velocity_pid;
```

因此：

```text
Motor1
├── position PID
└── velocity PID

Motor2
├── position PID
└── velocity PID
```

两个参数完全独立。

---

# 24. 电机控制模式

定义：

```c
typedef enum
{
    MOTOR_CONTROL_OPEN = 0,
    MOTOR_CONTROL_VELOCITY,
    MOTOR_CONTROL_POSITION
} MotorControlMode_t;
```

对应：

```text
OPEN
    ↓
直接输出控制量

VELOCITY
    ↓
目标速度
    ↓
速度控制

POSITION
    ↓
目标角度
    ↓
位置控制
    ↓
速度控制
```

最终项目运行模式：

```text
POSITION
VELOCITY
```

均通过同一个 `Motor_t` 对象控制。

---

# 25. Motor_Control_Loop()

这是整个系统的核心。

```c
void Motor_Control_Loop(void)
{
    Motor_Update(&motor1);
    Motor_Update(&motor2);
}
```

---

# 26. Motor_Update()

定义完整处理顺序：

```text
Motor_Update(motor)
        │
        ├── 检查状态
        │
        ├── 读取编码器
        │
        ├── 更新机械角
        │
        ├── 计算电角度
        │
        ├── 更新速度
        │
        ├── 执行控制器
        │
        ├── 计算电压矢量
        │
        ├── SVPWM
        │
        ├── 更新PWM
        │
        └── 故障检查
```

代码框架：

```c
void Motor_Update(Motor_t *motor)
{
    if (motor->state == MOTOR_STATE_ERROR)
        return;

    if (!Motor_UpdateEncoder(motor))
    {
        Motor_SetError(
            motor,
            MOTOR_ERROR_ENCODER
        );
        return;
    }

    Motor_UpdateAngle(motor);

    Motor_UpdateVelocity(motor);

    switch (motor->control_mode)
    {
        case MOTOR_CONTROL_OPEN:
            Motor_OpenLoop(motor);
            break;

        case MOTOR_CONTROL_VELOCITY:
            Motor_VelocityControl(motor);
            break;

        case MOTOR_CONTROL_POSITION:
            Motor_PositionControl(motor);
            break;
    }

    Motor_FOC_Output(motor);
}
```

---

# 27. 位置控制

```text
目标角度
    │
    ▼
位置误差
    │
    ▼
Position PID
    │
    ▼
目标速度
    │
    ▼
Velocity PID
    │
    ▼
目标 Vq
```

实现：

```c
void Motor_PositionControl(Motor_t *motor)
{
    float velocity_target;

    velocity_target =
        PID_Update(
            &motor->position_pid,
            motor->target_angle,
            motor->mechanical_angle,
            CONTROL_DT
        );

    motor->target_velocity =
        velocity_target;

    Motor_VelocityControl(motor);
}
```

---

# 28. 速度控制

当前没有电流反馈，因此速度控制最后转换成电压控制量。

```text
target_velocity
       ↓
velocity error
       ↓
Velocity PID
       ↓
Vq
```

：

```c
void Motor_VelocityControl(Motor_t *motor)
{
    motor->target_iq =
        PID_Update(
            &motor->velocity_pid,
            motor->target_velocity,
            motor->velocity,
            CONTROL_DT
        );
}
```

当前版本中的 `target_iq` 实际作为最终控制量接口，可以命名为：

```text
target_voltage
```

避免与真正的电流 Iq 混淆。

更准确的当前结构是：

```c
motor->target_voltage;
```

---

# 29. 当前版本 FOC 输出

由于没有电流采样：

```text
target_voltage
      ↓
Vq
      ↓
Vd = 0
      ↓
逆Park
      ↓
SVPWM
      ↓
Duty U/V/W
```

即：

```c
vd = 0;
vq = motor->target_voltage;
```

---

# 30. 电角度模块

独立函数：

```c
float Motor_GetElectricalAngle(Motor_t *motor);
```

计算：

```text
mechanical_angle
        ↓
减 zero_angle
        ↓
乘 pole_pairs
        ↓
乘 direction
        ↓
angle normalize
        ↓
electrical_angle
```

---

# 31. 零位校准模块

定义：

```c
HAL_StatusTypeDef Motor_Calibrate(
    Motor_t *motor
);
```

状态：

```text
INIT
 ↓
CALIBRATION
 ↓
READY
```

校准过程中：

```text
关闭正常闭环
 ↓
输出固定电压矢量
 ↓
电机转到指定电角位置
 ↓
等待稳定
 ↓
读取AS5600
 ↓
保存zero_angle
 ↓
恢复PWM正常控制
```

最终：

```c
motor->zero_angle
```

保存本次运行的编码器零点。

---

# 32. 控制周期设计

TIM3：

```text
72 MHz
PSC = 71
→ 1 MHz
```

设计控制周期：

```text
TIM3
 ↓
1 ms
 ↓
1 kHz
```

每次 TIM3 更新中断：

```text
Motor1 Control
+
Motor2 Control
```

即：

```text
1 ms
├── Motor1
└── Motor2
```

---

# 33. TIM3中断设计

CubeMX生成：

```c
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}
```

用户代码只注册：

```c
void HAL_TIM_PeriodElapsedCallback(
    TIM_HandleTypeDef *htim
)
{
    if (htim->Instance == TIM3)
    {
        Motor_Control_Loop();
    }
}
```

---

# 34. while(1)任务设计

`while(1)` 不执行实时 FOC。

定义：

```c
void Motor_System_Task(void);
```

负责：

```text
串口处理
 ↓
命令解析
 ↓
参数更新
 ↓
状态检查
 ↓
调试输出
```

流程：

```text
while(1)
{
    Motor_System_Task();
}
```

---

# 35. 串口模块设计

USART1：

```text
115200
8N1
```

串口模块负责：

```text
发送状态
接收命令
修改目标值
修改参数
查询状态
```

---

# 36. 串口命令设计

统一使用：

```text
M1 / M2
```

例如：

```text
M1 POS 90
M2 POS 180
```

含义：

```text
Motor1 target_angle = 90°
Motor2 target_angle = 180°
```

速度：

```text
M1 SPD 10
M2 SPD 5
```

停止：

```text
M1 STOP
M2 STOP
```

使能：

```text
M1 START
M2 START
```

查询：

```text
M1 STATUS
M2 STATUS
```

---

# 37. 串口输出数据

调试输出统一包含：

```text
Motor
State
Encoder
Mechanical Angle
Electrical Angle
Velocity
Target
PWM U
PWM V
PWM W
Error
```

例如：

```text
M1
STATE=RUNNING
ANGLE=123.40
EANGLE=4.82
VEL=6.31
TARGET=180.00
PWM=0.51,0.33,0.67
ERR=0
```

---

# 38. 故障系统

定义：

```c
typedef enum
{
    MOTOR_ERROR_NONE        = 0x00,
    MOTOR_ERROR_I2C         = 0x01,
    MOTOR_ERROR_ENCODER     = 0x02,
    MOTOR_ERROR_CALIBRATION = 0x04,
    MOTOR_ERROR_OVERSPEED   = 0x08,
    MOTOR_ERROR_CONTROL     = 0x10
} MotorError_t;
```

错误处理统一入口：

```c
void Motor_SetError(
    Motor_t *motor,
    uint32_t error
);
```

执行：

```text
记录错误
 ↓
state = ERROR
 ↓
PWM = 0
 ↓
EN = LOW
```

---

# 39. 双电机独立性设计

系统中不存在：

```text
Motor1控制Motor2
```

两个对象完全独立：

```text
motor1
├── TIM1
├── I2C1
├── AS5600 #1
├── target
├── PID
├── angle
├── speed
└── state

motor2
├── TIM2
├── I2C2
├── AS5600 #2
├── target
├── PID
├── angle
├── speed
└── state
```

控制入口统一：

```c
Motor_Update(&motor1);
Motor_Update(&motor2);
```

所以代码复用，但数据完全独立。

---

# 40. 完整运行时序

系统上电：

```text
STM32 Reset
   ↓
HAL_Init()
   ↓
SystemClock_Config()
   ↓
GPIO
   ↓
I2C1/I2C2
   ↓
TIM1/TIM2/TIM3
   ↓
USART1
   ↓
Motor_System_Init()
```

然后：

```text
Motor1 INIT
Motor2 INIT
      ↓
检测 AS5600
      ↓
CALIBRATION
      ↓
READY
      ↓
启动 TIM3
      ↓
1 kHz 控制循环
```

---

# 41. 每 1 ms 的实际运行过程

```text
TIM3 Update
      │
      ├─────────────────┐
      │                 │
      ▼                 ▼
   Motor1             Motor2
      │                 │
      ▼                 ▼
 AS5600 #1           AS5600 #2
      │                 │
      ▼                 ▼
机械角               机械角
      │                 │
      ▼                 ▼
电角度               电角度
      │                 │
      ▼                 ▼
位置/速度控制        位置/速度控制
      │                 │
      ▼                 ▼
Vd/Vq                Vd/Vq
      │                 │
      ▼                 ▼
逆Park               逆Park
      │                 │
      ▼                 ▼
SVPWM                 SVPWM
      │                 │
      ▼                 ▼
TIM1 CCR             TIM2 CCR
      │                 │
      ▼                 ▼
Motor1               Motor2
```

---

# 42. 两个电机参数实例

程序启动时创建：

```c
Motor_t motor1;
Motor_t motor2;
```

分别初始化：

```text
Motor1
TIM = TIM1
I2C = I2C1
EN = PB15

Motor2
TIM = TIM2
I2C = I2C2
EN = PA3
```

其他参数也独立：

```text
pole_pairs
direction
zero_angle
position PID
velocity PID
voltage limit
target position
target velocity
```

---

# 43. 数据流设计

整个软件最终只有一条主数据流：

```text
AS5600
 ↓
Encoder Data
 ↓
Mechanical Angle
 ↓
Electrical Angle
 ↓
Control
 ↓
Voltage Vector
 ↓
SVPWM
 ↓
PWM Duty
 ↓
SimpleFOCMini
 ↓
BLDC Motor
```

反馈：

```text
BLDC Motor
 ↓
AS5600
 ↓
Mechanical Angle
 ↓
Control
```

形成：

```text
┌─────────────────────────────────────┐
│                                     │
│       Position / Velocity           │
│               ↓                     │
│             Control                 │
│               ↓                     │
│            FOC/SVPWM                │
│               ↓                     │
│             PWM                     │
│               ↓                     │
│          SimpleFOCMini              │
│               ↓                     │
│             Motor                   │
│               │                     │
│               ▼                     │
│            AS5600                   │
│               │                     │
│               └─────────────────────┘
```

---

# 44. 第一阶段代码完成标准

完成：

```text
motor.h
motor.c
motor_config.h
```

实现：

```text
□ Motor对象
□ Motor_Init
□ Motor_PWM_Start
□ Motor_SetPWM
□ Motor_Enable
□ Motor_Disable
```

验证：

```text
TIM1三路PWM正常
TIM2三路PWM正常
EN正常
```

---

# 45. 第二阶段代码完成标准

完成：

```text
as5600.h
as5600.c
```

实现：

```text
□ I2C1读取AS5600
□ I2C2读取AS5600
□ Raw Angle
□ Mechanical Angle
□ I2C错误处理
```

验证：

```text
M1角度能够实时变化
M2角度能够实时变化
```

---

# 46. 第三阶段代码完成标准

完成：

```text
motor_control.c
```

实现：

```text
□ 电机状态机
□ 编码器更新
□ 零位
□ 电角度
□ Motor_Enable
□ Motor_Disable
```

验证：

```text
INIT
 ↓
CALIBRATION
 ↓
READY
```

---

# 47. 第四阶段代码完成标准

完成：

```text
pid.c
pid.h
```

实现：

```text
□ 速度控制
□ 位置控制
□ 两套独立参数
```

验证：

```text
Motor1可以设置目标角度
Motor2可以设置目标角度

Motor1和Motor2可以不同步运行
```

---

# 48. 第五阶段代码完成标准

完成：

```text
foc.c
foc.h
```

实现：

```text
□ 电角度
□ Vd/Vq
□ 逆Park
□ SVPWM
□ PWM更新
```

完整链路：

```text
Target
 ↓
Controller
 ↓
Electrical Angle
 ↓
FOC
 ↓
SVPWM
 ↓
PWM
 ↓
Motor
```

---

# 49. 第六阶段代码完成标准

完成：

```text
uart_debug.c
uart_debug.h
```

实现：

```text
□ START
□ STOP
□ POS
□ SPD
□ STATUS
```

例如：

```text
M1 POS 90
M2 POS 180
```

STM32：

```text
Motor1 → 90°
Motor2 → 180°
```

---

# 50. 最终函数关系

最终主要函数关系固定为：

```text
main()
 │
 ├── Motor_System_Init()
 │       │
 │       ├── Motor_Init(M1)
 │       ├── Motor_Init(M2)
 │       ├── PWM Start
 │       └── Encoder Check
 │
 ├── HAL_TIM_Base_Start_IT(TIM3)
 │
 └── while(1)
         │
         └── Motor_System_Task()
                 │
                 └── UART / Debug
```

TIM3：

```text
TIM3_IRQHandler()
       ↓
HAL_TIM_IRQHandler()
       ↓
HAL_TIM_PeriodElapsedCallback()
       ↓
Motor_Control_Loop()
       │
       ├── Motor_Update(M1)
       │
       └── Motor_Update(M2)
```

Motor_Update：

```text
Motor_Update()
 │
 ├── Encoder
 │
 ├── Mechanical Angle
 │
 ├── Electrical Angle
 │
 ├── Position / Velocity Control
 │
 ├── FOC
 │
 ├── SVPWM
 │
 ├── PWM
 │
 └── Error Check
```

---

# 51. 最终代码职责关系

```text
                    main.c
                       │
                       ▼
             motor_control.c
                       │
          ┌────────────┴────────────┐
          ▼                         ▼
      motor1                     motor2
          │                         │
    ┌─────┼─────┐             ┌─────┼─────┐
    ▼     ▼     ▼             ▼     ▼     ▼
  TIM1  I2C1  AS5600         TIM2  I2C2  AS5600
    │           │               │           │
    ▼           ▼               ▼           ▼
   PWM        Angle             PWM        Angle
    │           │               │           │
    └─────┬─────┘               └─────┬─────┘
          ▼                           ▼
        FOC                         FOC
          │                           │
          ▼                           ▼
       Motor1                      Motor2
```

---

# 52. 最终工程完成后的模块边界

```text
CubeMX层
├── GPIO
├── I2C1
├── I2C2
├── TIM1
├── TIM2
├── TIM3
└── USART1

驱动层
├── AS5600
└── PWM

算法层
├── PID
├── Clarke
├── Park
├── Inverse Park
└── SVPWM

电机层
├── Motor1
├── Motor2
├── 状态机
├── 零位校准
└── 故障处理

应用层
├── 目标位置
├── 目标速度
├── 启停控制
└── UART调参
```

**最终结果就是：CubeMX 只负责 STM32 外设初始化；`as5600.c` 负责传感器；`motor.c` 负责 PWM/EN；`foc.c` 负责数学算法；`pid.c` 负责控制器；`motor_control.c` 负责把这些模块串成一个完整的双电机控制系统；`main.c` 只负责启动系统。**

此外，你当前工程里的 TIM1 为 20 kHz、TIM2 为约 1.1 kHz、TIM3 为 1 kHz，这三个时基会按你给出的现有 CubeMX 配置直接纳入上述软件架构；当前硬件没有 ADC 电流采样，因此本设计的当前版本闭环反馈是 **AS5600 位置反馈，控制输出为电压矢量 + SVPWM**，而不是完整的电流采样闭环 FOC。 
