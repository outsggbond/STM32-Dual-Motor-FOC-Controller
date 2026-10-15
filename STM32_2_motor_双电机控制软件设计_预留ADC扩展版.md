**\*\*做什么、文件怎么分、数据怎么流、函数怎么写、每个阶段完成什么、最终代码达到什么状态。\*\***

**# STM32\_2\_motor 双电机控制软件设计**

**## 1. 项目最终目标**

工程最终实现：

\`\`\`text

STM32F103C8T6

│

├── 电机1

│   ├── TIM1 CH1/CH2/CH3

│   ├── PB15 EN

│   ├── I2C1

│   └── AS5600

│

└── 电机2

    ├── TIM2 CH1/CH2/CH3

    ├── PA3 EN

    ├── I2C2

    └── AS5600

\`\`\`

软件完成：

\`\`\`text

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

\`\`\`

最终两个电机使用**\*\*同一套控制代码\*\***，但每个电机拥有独立参数、独立目标值、独立状态和独立控制结果。

**---**

**# 2. 软件总体架构**

整个程序分成 5 层：

\`\`\`text

┌────────────────────────────────────────────┐

│              应用控制层 Application          │

│                                            │

│ motor\_control.c                            │

│ 目标位置 / 目标速度 / FOC / 控制状态         │

└─────────────────────┬──────────────────────┘

                      │

┌─────────────────────▼──────────────────────┐

│                 电机层 Motor                 │

│                                            │

│ motor.c                                    │

│ 电机对象 / PWM / EN / 基本状态               │

└───────────────┬────────────────────────────┘

                │

       ┌────────┴────────┐

       │                 │

┌──────▼───────┐ ┌──────▼────────┐

│  AS5600驱动  │ │  PWM硬件接口   │

│  as5600.c    │ │  TIM1 / TIM2  │

└──────┬───────┘ └───────────────┘

       │

       ▼

   I2C1 / I2C2

同时：

┌────────────────────────────────────────────┐

│             系统任务层                     │

│                                            │

│ timer / uart / fault                       │

│ TIM3控制周期 + USART调试                   │

└────────────────────────────────────────────┘

\`\`\`

**---**

**# 3. 最终目录结构**

在 CubeMX 当前工程基础上增加：

\`\`\`text

STM32\_2\_motor/

│

├── Core/

│   ├── Inc/

│   │   ├── main.h

│   │   ├── motor.h

│   │   ├── motor\_config.h

│   │   ├── motor\_control.h

│   │   ├── as5600.h

│   │   ├── foc.h

│   │   ├── pid.h

│   │   └── uart\_debug.h

│   │

│   └── Src/

│       ├── main.c

│       ├── motor.c

│       ├── motor\_control.c

│       ├── as5600.c

│       ├── foc.c

│       ├── pid.c

│       └── uart\_debug.c

│

├── Drivers/

│   ├── CMSIS/

│   └── STM32F1xx\_HAL\_Driver/

│

├── STM32\_2\_motor.ioc

└── STM32F103C8TX\_FLASH.ld

\`\`\`

CubeMX 生成的：

\`\`\`text

gpio.c

i2c.c

tim.c

usart.c

stm32f1xx\_it.c

\`\`\`

继续由 CubeMX 管理。

**\*\*用户自己编写的电机业务代码全部放在新建模块中。\*\***

**---**

**# 4. 各文件职责**

| 文件 | 职责 |
| ------------------- | ----------------------------- |
| `main.c` | 系统启动、外设初始化、启动控制系统 |
| `motor_config.h` | 两个电机的硬件映射和基础参数 |
| `motor.h` | Motor 对象、状态、接口定义 |
| `motor.c` | PWM、EN、Motor对象基础操作 |
| `as5600.h/c` | AS5600 I2C驱动、角度读取 |
| `foc.h/c` | 当前版本电角度、逆Park、电压矢量、SVPWM；预留Clarke/Park/电流环接口 |
| `pid.h/c` | 当前版本位置/速度控制器；预留Id/Iq电流PI控制接口 |
| `current_sense.h/c` | ADC电流采样扩展接口（当前硬件不存在，当前版本不启用） |
| `motor_control.h/c` | 双电机控制流程、状态机、控制周期 |
| `uart_debug.h/c` | 串口命令、参数修改、状态输出 |
| `stm32f1xx_it.c` | CubeMX生成的中断入口 |

**当前版本原则：**

```text
CurrentSense / ADC
    = 预留
    = 不初始化
    = 不读取
    = 不参与当前控制闭环
```

后续增加电流采样硬件后，只扩展 `current_sense.c`、ADC配置和电流环控制接口，不改变现有双电机对象和上层控制框架。


**# 5. 电机对象设计**

核心数据结构统一成：

\`\`\`c

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

    /* ADC电流采样扩展预留
       当前硬件没有ADC电流采样，以下字段当前不参与控制 */

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

    /* 状态 */

    uint8_t encoder_ok;

    uint8_t driver_enabled;

    uint8_t state;

    /* 故障 */

    uint32_t error;

} Motor\_t;

\`\`\`

全局只创建：

\`\`\`c

Motor\_t motor1;

Motor\_t motor2;

\`\`\`

**---**

**# 6. 电机配置设计**

\`motor\_config.h\` 定义硬件资源。

**## Motor1**

\`\`\`c

\#define MOTOR1\_PWM\_TIMER      (&htim1)

\#define MOTOR1\_PWM\_U          TIM\_CHANNEL\_1

\#define MOTOR1\_PWM\_V          TIM\_CHANNEL\_2

\#define MOTOR1\_PWM\_W          TIM\_CHANNEL\_3

\#define MOTOR1\_EN\_PORT        GPIOB

\#define MOTOR1\_EN\_PIN         GPIO\_PIN\_15

\#define MOTOR1\_I2C            (&hi2c1)

\#define MOTOR1\_ENCODER\_ADDR   0x36

\`\`\`

对应：

\`\`\`text

PA8  → U

PA9  → V

PA10 → W

PB15 → EN

PB8  → SCL

PB9  → SDA

\`\`\`

**## Motor2**

\`\`\`c

\#define MOTOR2\_PWM\_TIMER      (&htim2)

\#define MOTOR2\_PWM\_U          TIM\_CHANNEL\_1

\#define MOTOR2\_PWM\_V          TIM\_CHANNEL\_2

\#define MOTOR2\_PWM\_W          TIM\_CHANNEL\_3

\#define MOTOR2\_EN\_PORT        GPIOA

\#define MOTOR2\_EN\_PIN         GPIO\_PIN\_3

\#define MOTOR2\_I2C            (&hi2c2)

\#define MOTOR2\_ENCODER\_ADDR   0x36

\`\`\`

对应：

\`\`\`text

PA0 → U

PA1 → V

PA2 → W

PA3 → EN

PB10 → SCL

PB11 → SDA

\`\`\`

**---**

**# 7. 电机状态机**

每个电机独立维护状态。

\`\`\`c

typedef enum

{

    MOTOR\_STATE\_DISABLED = 0,

    MOTOR\_STATE\_INIT,

    MOTOR\_STATE\_CALIBRATION,

    MOTOR\_STATE\_READY,

    MOTOR\_STATE\_RUNNING,

    MOTOR\_STATE\_ERROR

} MotorState\_t;

\`\`\`

状态流程：

\`\`\`text

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

\`\`\`

异常：

\`\`\`text

任何状态

   │

   ▼

 ERROR

   │

   ▼

 PWM关闭

 EN关闭

\`\`\`

状态含义：

\| 状态            | 功能           |

\| ------------- | ------------ |

\| \`DISABLED\`    | 电机完全关闭       |

\| \`INIT\`        | 软件对象和外设准备    |

\| \`CALIBRATION\` | 建立机械零位/电角度零位 |

\| \`READY\`       | 初始化完成，等待目标   |

\| \`RUNNING\`     | 正常闭环运行       |

\| \`ERROR\`       | 发生故障，停止输出    |

**---**

**# 8. main.c 设计**

\`main.c\` 只负责系统启动，不负责具体 FOC 算法。

最终结构：

\`\`\`c

int main(void)

{

    HAL\_Init();

    SystemClock\_Config();

    MX\_GPIO\_Init();

    MX\_I2C1\_Init();

    MX\_I2C2\_Init();

    MX\_TIM1\_Init();

    MX\_TIM2\_Init();

    MX\_TIM3\_Init();

    MX\_USART1\_UART\_Init();

    Motor\_System\_Init();

    HAL\_TIM\_Base\_Start\_IT(&htim3);

    while (1)

    {

        Motor\_System\_Task();

    }

}

\`\`\`

其中：

\`\`\`c

Motor\_System\_Init();

\`\`\`

负责整个电机系统初始化。

**---**

**# 9. Motor\_System\_Init()**

初始化流程：

\`\`\`text

Motor\_System\_Init()

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

\`\`\`

接口：

\`\`\`c

void Motor\_System\_Init(void);

\`\`\`

内部：

\`\`\`c

Motor\_Init(&motor1, ...);

Motor\_Init(&motor2, ...);

\`\`\`

**---**

**# 10. Motor\_Init()**

接口设计：

\`\`\`c

void Motor\_Init(

    Motor\_t \*motor,

    TIM\_HandleTypeDef \*timer,

    uint32\_t ch\_u,

    uint32\_t ch\_v,

    uint32\_t ch\_w,

    GPIO\_TypeDef \*en\_port,

    uint16\_t en\_pin,

    I2C\_HandleTypeDef \*i2c,

    uint8\_t encoder\_addr

);

\`\`\`

完成：

\`\`\`text

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

\`\`\`

**---**

**# 11. PWM 模块设计**

\`motor.c\` 只处理 PWM。

**## 启动**

\`\`\`c

void Motor\_PWM\_Start(Motor\_t \*motor);

\`\`\`

实际执行：

\`\`\`c

HAL\_TIM\_PWM\_Start(... U ...);

HAL\_TIM\_PWM\_Start(... V ...);

HAL\_TIM\_PWM\_Start(... W ...);

\`\`\`

**---**

**## 设置三相占空比**

\`\`\`c

void Motor\_SetPWM(

    Motor\_t \*motor,

    float duty\_u,

    float duty\_v,

    float duty\_w

);

\`\`\`

输入范围：

\`\`\`text

0.0 \~ 1.0

\`\`\`

输出：

\`\`\`text

CCR\_U

CCR\_V

CCR\_W

\`\`\`

关系：

\`\`\`text

CCR = duty × ARR

\`\`\`

**---**

**# 12. EN控制设计**

\`\`\`c

void Motor\_Enable(Motor\_t \*motor);

void Motor\_Disable(Motor\_t \*motor);

\`\`\`

Enable：

\`\`\`text

PWM正常

\+

EN = HIGH

\`\`\`

Disable：

\`\`\`text

PWM = 0

\+

EN = LOW

\`\`\`

任何错误都通过：

\`\`\`c

Motor\_Disable(motor);

\`\`\`

关闭。

**---**


**# 14. ADC电流采样扩展模块（预留，不启用）**

增加：

```text
Core/Inc/current_sense.h
Core/Src/current_sense.c
```

职责：

```text
ADC采样接口
    ↓
ADC原始值
    ↓
零点校准
    ↓
电流换算
    ↓
Ia / Ib / Ic
```

### 当前版本

```text
ADC外设：不配置
ADC采样：不执行
DMA：不配置
电流反馈：不参与控制
```

### 预留接口

```c
typedef struct
{
    uint16_t adc_raw_u;
    uint16_t adc_raw_v;
    uint16_t adc_raw_w;

    float current_u;
    float current_v;
    float current_w;

    float ia;
    float ib;
    float ic;

    float current_offset_u;
    float current_offset_v;
    float current_offset_w;

    uint8_t current_valid;
} CurrentSense_t;

HAL_StatusTypeDef CurrentSense_Init(void);

HAL_StatusTypeDef CurrentSense_Read(
    CurrentSense_t *sense
);

void CurrentSense_CalibrateOffset(
    CurrentSense_t *sense
);
```

**当前接口只作为未来扩展保留，不在当前版本调用。**


**# 14. AS5600驱动设计**

\`as5600.c\` 只负责传感器。

不参与：

\`\`\`text

FOC

PID

PWM

状态机

\`\`\`

只负责：

\`\`\`text

I2C

 ↓

AS5600

 ↓

Raw Angle

 ↓

机械角度

\`\`\`

**---**

**# 15. AS5600接口**

定义：

\`\`\`c

HAL\_StatusTypeDef AS5600\_ReadRawAngle(

    I2C\_HandleTypeDef \*hi2c,

    uint8\_t addr,

    uint16\_t \*raw

);

\`\`\`

以及：

\`\`\`c

HAL\_StatusTypeDef AS5600\_ReadAngle(

    I2C\_HandleTypeDef \*hi2c,

    uint8\_t addr,

    float \*angle

);

\`\`\`

角度：

\`\`\`text

0 \~ 4095

\`\`\`

转换：

\`\`\`text

mechanical\_angle

\=

raw / 4096 × 2π

\`\`\`

或者保存为角度：

\`\`\`text

0 \~ 360°

\`\`\`

**---**

**# 16. 编码器读取流程**

每一个控制周期：

\`\`\`text

Motor\_Update()

     │

     ▼

AS5600\_ReadRawAngle()

     │

     ▼

encoder\_raw

     │

     ▼

mechanical\_angle

     │

     ▼

direction

     │

     ▼

zero\_angle

     │

     ▼

electrical\_angle

\`\`\`

电角度：

\`\`\`text

θe = direction × pole\_pairs × (θm - zero\_angle)

\`\`\`

然后归一化到：

\`\`\`text

0 \~ 2π

\`\`\`

**---**

**# 17. 编码器异常处理**

AS5600读取失败：

\`\`\`text

I2C失败

 ↓

encoder\_ok = 0

 ↓

error |= MOTOR\_ERROR\_ENCODER

 ↓

state = ERROR

 ↓

Motor\_Disable()

\`\`\`

AS5600正常：

\`\`\`text

encoder\_ok = 1

\`\`\`

**---**

**# 18. FOC模块设计**

`foc.c` 负责空间矢量相关数学运算。

## 17.1 当前版本实际使用

当前硬件没有ADC电流采样，因此当前版本不执行电流采样闭环。

```text
电角度
    ↓
Vd / Vq
    ↓
逆Park
    ↓
Vα / Vβ
    ↓
SVPWM
    ↓
Duty U/V/W
```

当前版本：

```c
vd = 0.0f;
vq = motor->target_voltage;
```

## 17.2 ADC电流采样扩展预留

未来增加电流采样硬件后，启用：

```text
Ia / Ib / Ic
    ↓
Clarke
    ↓
Iα / Iβ
    ↓
Park
    ↓
Id / Iq
    ↓
Id/Iq PI
    ↓
Vd / Vq
    ↓
逆Park
    ↓
SVPWM
```

预留接口：

```c
void FOC_Clarke(
    float ia,
    float ib,
    float ic,
    float *alpha,
    float *beta
);

void FOC_Park(
    float alpha,
    float beta,
    float angle,
    float *d,
    float *q
);

void FOC_CurrentControl(
    float target_id,
    float target_iq,
    float actual_id,
    float actual_iq,
    float dt,
    float *vd,
    float *vq
);
```

**以上电流环接口当前只保留，不调用，不依赖ADC硬件。**


**# 19. FOC接口**

\`\`\`c

void FOC\_Init(void);

\`\`\`

**---**

**## Clarke**

\`\`\`c

void FOC\_Clarke(

    float ia,

    float ib,

    float ic,

    float \*alpha,

    float \*beta

);

\`\`\`

**---**

**## Park**

\`\`\`c

void FOC\_Park(

    float alpha,

    float beta,

    float angle,

    float \*d,

    float \*q

);

\`\`\`

**---**

**## 逆Park**

\`\`\`c

void FOC\_InvPark(

    float d,

    float q,

    float angle,

    float \*alpha,

    float \*beta

);

\`\`\`

**---**

**## SVPWM**

\`\`\`c

void FOC\_SVPWM(

    float alpha,

    float beta,

    float \*duty\_u,

    float \*duty\_v,

    float \*duty\_w

);

\`\`\`

**---**

**# 20. 一个完整控制周期**

## 19.1 当前版本

```text
读取 AS5600
      ↓
获取机械角
      ↓
机械角 → 电角度
      ↓
位置控制 / 速度控制
      ↓
目标电压
      ↓
Vd = 0
Vq = target_voltage
      ↓
逆Park
      ↓
SVPWM
      ↓
Duty U/V/W
      ↓
TIM CCR
```

## 19.2 未来启用ADC电流采样后的版本

增加电流采样硬件并完成ADC配置后：

```text
读取 AS5600
      ↓
获取电角度
      ↓
ADC采样
      ↓
Ia / Ib / Ic
      ↓
Clarke
      ↓
Iα / Iβ
      ↓
Park
      ↓
Id / Iq
      ↓
Id/Iq PI
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

当前程序只执行 `19.1`，`19.2` 为未来扩展路径。


**# 21. 当前硬件没有电流采样时的控制结构**

当前硬件没有ADC电流采样，因此当前版本：

```text
ADC电流采样 = 预留
ADC初始化 = 不执行
ADC读取 = 不执行
电流反馈 = 不参与控制
电流PI = 不执行
```

## 20.1 当前实际控制结构

```text
AS5600
  ↓
机械位置
  ↓
电角度
  ↓
位置控制
  ↓
目标速度
  ↓
速度控制
  ↓
target_voltage
  ↓
Vd = 0 / Vq = target_voltage
  ↓
逆Park
  ↓
SVPWM
  ↓
三相PWM
```

## 20.2 未来ADC电流采样扩展

保留以下软件数据链：

```text
ADC
  ↓
ADC原始值
  ↓
零点校准
  ↓
电流换算
  ↓
Ia / Ib / Ic
  ↓
Clarke
  ↓
Iα / Iβ
  ↓
Park
  ↓
Id / Iq
  ↓
Id/Iq PI
  ↓
Vd / Vq
```

未来新增硬件后再确定ADC通道、采样方式、采样电阻、放大倍数、零点电压、电流量程和采样时序。当前工程不虚构这些参数。


**# 22. 当前版本控制层设计**

`motor_control.c` 当前只执行无电流采样控制链：

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
target_voltage
 │
 ▼
Vd = 0
Vq = target_voltage
 │
 ▼
逆Park
 │
 ▼
SVPWM
 │
 ▼
PWM
```

两个电机分别执行。

## 未来ADC电流采样扩展

增加电流采样后，可扩展为：

```text
Target
 │
 ▼
位置/速度控制
 │
 ▼
target_Iq
 │
 ▼
ADC电流采样
 │
 ▼
Clarke / Park
 │
 ▼
实际Id / Iq
 │
 ▼
Id/Iq PI
 │
 ▼
Vd / Vq
 │
 ▼
逆Park
 │
 ▼
SVPWM
 │
 ▼
PWM
```

当前版本禁止把 `target_voltage` 当作真实 `target_iq` 使用。


**# 23. PID模块**

`pid.c` 负责通用PID控制器。

当前使用：

```text
Position PID
Velocity PID
```

未来启用ADC电流采样后扩展：

```text
Id PID
Iq PID
```

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

float PID_Update(
    PID_t *pid,
    float target,
    float feedback,
    float dt
);
```

每个电机当前至少拥有：

```c
PID_t position_pid;
PID_t velocity_pid;
```

未来可扩展：

```c
PID_t current_id_pid;
PID_t current_iq_pid;
```

当前电流PID不参与运行控制。


**# 24. 电机控制参数**

每个 Motor 对象拥有：

\`\`\`c

PID\_t position\_pid;

PID\_t velocity\_pid;

\`\`\`

因此：

\`\`\`text

Motor1

├── position PID

└── velocity PID

Motor2

├── position PID

└── velocity PID

\`\`\`

两个参数完全独立。

**---**

**# 25. 电机控制模式**

定义：

\`\`\`c

typedef enum

{

    MOTOR\_CONTROL\_OPEN = 0,

    MOTOR\_CONTROL\_VELOCITY,

    MOTOR\_CONTROL\_POSITION

} MotorControlMode\_t;

\`\`\`

对应：

\`\`\`text

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

\`\`\`

最终项目运行模式：

\`\`\`text

POSITION

VELOCITY

\`\`\`

均通过同一个 \`Motor\_t\` 对象控制。

**---**

**# 26. Motor\_Control\_Loop()**

这是整个系统的核心。

\`\`\`c

void Motor\_Control\_Loop(void)

{

    Motor\_Update(&motor1);

    Motor\_Update(&motor2);

}

\`\`\`

**---**

**# 27. Motor\_Update()**

定义完整处理顺序：

\`\`\`text

Motor\_Update(motor)

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

\`\`\`

代码框架：

\`\`\`c

void Motor\_Update(Motor\_t \*motor)

{

    if (motor->state == MOTOR\_STATE\_ERROR)

        return;

    if (!Motor\_UpdateEncoder(motor))

    {

        Motor\_SetError(

            motor,

            MOTOR\_ERROR\_ENCODER

        );

        return;

    }

    Motor\_UpdateAngle(motor);

    Motor\_UpdateVelocity(motor);

    switch (motor->control\_mode)

    {

        case MOTOR\_CONTROL\_OPEN:

            Motor\_OpenLoop(motor);

            break;

        case MOTOR\_CONTROL\_VELOCITY:

            Motor\_VelocityControl(motor);

            break;

        case MOTOR\_CONTROL\_POSITION:

            Motor\_PositionControl(motor);

            break;

    }

    Motor\_FOC\_Output(motor);

}

\`\`\`

**---**

**# 28. 位置控制**

\`\`\`text

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

\`\`\`

实现：

\`\`\`c

void Motor\_PositionControl(Motor\_t \*motor)

{

    float velocity\_target;

    velocity\_target =

        PID\_Update(

            &motor->position\_pid,

            motor->target\_angle,

            motor->mechanical\_angle,

            CONTROL\_DT

        );

    motor->target\_velocity =

        velocity\_target;

    Motor\_VelocityControl(motor);

}

\`\`\`

**---**

**# 29. 速度控制**

当前没有电流反馈，因此速度控制最后转换成电压控制量。

\`\`\`text

target\_velocity

       ↓

velocity error

       ↓

Velocity PID

       ↓

Vq

\`\`\`

：

\`\`\`c

void Motor\_VelocityControl(Motor\_t \*motor)

{

    motor->target\_iq =

        PID\_Update(

            &motor->velocity\_pid,

            motor->target\_velocity,

            motor->velocity,

            CONTROL\_DT

        );

}

\`\`\`

当前版本中的 \`target\_iq\` 实际作为最终控制量接口，可以命名为：

\`\`\`text

target\_voltage

\`\`\`

避免与真正的电流 Iq 混淆。

更准确的当前结构是：

\`\`\`c

motor->target\_voltage;

\`\`\`

**---**

**# 30. 当前版本 FOC 输出**

由于没有电流采样：

\`\`\`text

target\_voltage

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

\`\`\`

即：

\`\`\`c

vd = 0;

vq = motor->target\_voltage;

\`\`\`

**---**

**# 31. 电角度模块**

独立函数：

\`\`\`c

float Motor\_GetElectricalAngle(Motor\_t \*motor);

\`\`\`

计算：

\`\`\`text

mechanical\_angle

        ↓

减 zero\_angle

        ↓

乘 pole\_pairs

        ↓

乘 direction

        ↓

angle normalize

        ↓

electrical\_angle

\`\`\`

**---**

**# 32. 零位校准模块**

定义：

\`\`\`c

HAL\_StatusTypeDef Motor\_Calibrate(

    Motor\_t \*motor

);

\`\`\`

状态：

\`\`\`text

INIT

 ↓

CALIBRATION

 ↓

READY

\`\`\`

校准过程中：

\`\`\`text

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

保存zero\_angle

 ↓

恢复PWM正常控制

\`\`\`

最终：

\`\`\`c

motor->zero\_angle

\`\`\`

保存本次运行的编码器零点。

**---**

**# 33. 控制周期设计**

TIM3：

\`\`\`text

72 MHz

PSC = 71

→ 1 MHz

\`\`\`

设计控制周期：

\`\`\`text

TIM3

 ↓

1 ms

 ↓

1 kHz

\`\`\`

每次 TIM3 更新中断：

\`\`\`text

Motor1 Control

\+

Motor2 Control

\`\`\`

即：

\`\`\`text

1 ms

├── Motor1

└── Motor2

\`\`\`

**---**

**# 34. TIM3中断设计**

CubeMX生成：

\`\`\`c

void TIM3\_IRQHandler(void)

{

    HAL\_TIM\_IRQHandler(&htim3);

}

\`\`\`

用户代码只注册：

\`\`\`c

void HAL\_TIM\_PeriodElapsedCallback(

    TIM\_HandleTypeDef \*htim

)

{

    if (htim->Instance == TIM3)

    {

        Motor\_Control\_Loop();

    }

}

\`\`\`

**---**

**# 35. while(1)任务设计**

\`while(1)\` 不执行实时 FOC。

定义：

\`\`\`c

void Motor\_System\_Task(void);

\`\`\`

负责：

\`\`\`text

串口处理

 ↓

命令解析

 ↓

参数更新

 ↓

状态检查

 ↓

调试输出

\`\`\`

流程：

\`\`\`text

while(1)

{

    Motor\_System\_Task();

}

\`\`\`

**---**

**# 36. 串口模块设计**

USART1：

\`\`\`text

115200

8N1

\`\`\`

串口模块负责：

\`\`\`text

发送状态

接收命令

修改目标值

修改参数

查询状态

\`\`\`

**---**

**# 37. 串口命令设计**

统一使用：

\`\`\`text

M1 / M2

\`\`\`

例如：

\`\`\`text

M1 POS 90

M2 POS 180

\`\`\`

含义：

\`\`\`text

Motor1 target\_angle = 90°

Motor2 target\_angle = 180°

\`\`\`

速度：

\`\`\`text

M1 SPD 10

M2 SPD 5

\`\`\`

停止：

\`\`\`text

M1 STOP

M2 STOP

\`\`\`

使能：

\`\`\`text

M1 START

M2 START

\`\`\`

查询：

\`\`\`text

M1 STATUS

M2 STATUS

\`\`\`

**---**

**# 38. 串口输出数据**

调试输出统一包含：

\`\`\`text

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

\`\`\`

例如：

\`\`\`text

M1

STATE=RUNNING

ANGLE=123.40

EANGLE=4.82

VEL=6.31

TARGET=180.00

PWM=0.51,0.33,0.67

ERR=0

\`\`\`

**---**

**# 39. 故障系统**

定义：

\`\`\`c

typedef enum

{

    MOTOR\_ERROR\_NONE        = 0x00,

    MOTOR\_ERROR\_I2C         = 0x01,

    MOTOR\_ERROR\_ENCODER     = 0x02,

    MOTOR\_ERROR\_CALIBRATION = 0x04,

    MOTOR\_ERROR\_OVERSPEED   = 0x08,

    MOTOR\_ERROR\_CONTROL     = 0x10

} MotorError\_t;

\`\`\`

错误处理统一入口：

\`\`\`c

void Motor\_SetError(

    Motor\_t \*motor,

    uint32\_t error

);

\`\`\`

执行：

\`\`\`text

记录错误

 ↓

state = ERROR

 ↓

PWM = 0

 ↓

EN = LOW

\`\`\`

**---**

**# 40. 双电机独立性设计**

系统中不存在：

\`\`\`text

Motor1控制Motor2

\`\`\`

两个对象完全独立：

\`\`\`text

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

\`\`\`

控制入口统一：

\`\`\`c

Motor\_Update(&motor1);

Motor\_Update(&motor2);

\`\`\`

所以代码复用，但数据完全独立。

**---**

**# 41. 完整运行时序**

系统上电：

\`\`\`text

STM32 Reset

   ↓

HAL\_Init()

   ↓

SystemClock\_Config()

   ↓

GPIO

   ↓

I2C1/I2C2

   ↓

TIM1/TIM2/TIM3

   ↓

USART1

   ↓

Motor\_System\_Init()

\`\`\`

然后：

\`\`\`text

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

\`\`\`

**---**

**# 42. 每 1 ms 的实际运行过程**

\`\`\`text

TIM3 Update

      │

      ├─────────────────┐

      │                 │

      ▼                 ▼

   Motor1             Motor2

      │                 │

      ▼                 ▼

 AS5600 #1           AS5600 #2

      │                 │

      ▼                 ▼

机械角               机械角

      │                 │

      ▼                 ▼

电角度               电角度

      │                 │

      ▼                 ▼

位置/速度控制        位置/速度控制

      │                 │

      ▼                 ▼

Vd/Vq                Vd/Vq

      │                 │

      ▼                 ▼

逆Park               逆Park

      │                 │

      ▼                 ▼

SVPWM                 SVPWM

      │                 │

      ▼                 ▼

TIM1 CCR             TIM2 CCR

      │                 │

      ▼                 ▼

Motor1               Motor2

\`\`\`

**---**

**# 43. 两个电机参数实例**

程序启动时创建：

\`\`\`c

Motor\_t motor1;

Motor\_t motor2;

\`\`\`

分别初始化：

\`\`\`text

Motor1

TIM = TIM1

I2C = I2C1

EN = PB15

Motor2

TIM = TIM2

I2C = I2C2

EN = PA3

\`\`\`

其他参数也独立：

\`\`\`text

pole\_pairs

direction

zero\_angle

position PID

velocity PID

voltage limit

target position

target velocity

\`\`\`

**---**

**# 44. 数据流设计**

整个软件最终只有一条主数据流：

\`\`\`text

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

\`\`\`

反馈：

\`\`\`text

BLDC Motor

 ↓

AS5600

 ↓

Mechanical Angle

 ↓

Control

\`\`\`

形成：

\`\`\`text

┌─────────────────────────────────────┐

│                                     │

│       Position / Velocity           │

│               ↓                     │

│             Control                 │

│               ↓                     │

│            FOC/SVPWM                │

│               ↓                     │

│             PWM                     │

│               ↓                     │

│          SimpleFOCMini              │

│               ↓                     │

│             Motor                   │

│               │                     │

│               ▼                     │

│            AS5600                   │

│               │                     │

│               └─────────────────────┘

\`\`\`

**---**

**# 45. 第一阶段代码完成标准**

完成：

\`\`\`text

motor.h

motor.c

motor\_config.h

\`\`\`

实现：

\`\`\`text

□ Motor对象

□ Motor\_Init

□ Motor\_PWM\_Start

□ Motor\_SetPWM

□ Motor\_Enable

□ Motor\_Disable

\`\`\`

验证：

\`\`\`text

TIM1三路PWM正常

TIM2三路PWM正常

EN正常

\`\`\`

**---**

**# 46. 第二阶段代码完成标准**

完成：

\`\`\`text

as5600.h

as5600.c

\`\`\`

实现：

\`\`\`text

□ I2C1读取AS5600

□ I2C2读取AS5600

□ Raw Angle

□ Mechanical Angle

□ I2C错误处理

\`\`\`

验证：

\`\`\`text

M1角度能够实时变化

M2角度能够实时变化

\`\`\`

**---**

**# 47. 第三阶段代码完成标准**

完成：

\`\`\`text

motor\_control.c

\`\`\`

实现：

\`\`\`text

□ 电机状态机

□ 编码器更新

□ 零位

□ 电角度

□ Motor\_Enable

□ Motor\_Disable

\`\`\`

验证：

\`\`\`text

INIT

 ↓

CALIBRATION

 ↓

READY

\`\`\`

**---**

**# 48. 第四阶段代码完成标准**

完成：

\`\`\`text

pid.c

pid.h

\`\`\`

实现：

\`\`\`text

□ 速度控制

□ 位置控制

□ 两套独立参数

\`\`\`

验证：

\`\`\`text

Motor1可以设置目标角度

Motor2可以设置目标角度

Motor1和Motor2可以不同步运行

\`\`\`

**---**

**# 49. 第五阶段代码完成标准**

完成：

\`\`\`text

foc.c

foc.h

\`\`\`

实现：

\`\`\`text

□ 电角度

□ Vd/Vq

□ 逆Park

□ SVPWM

□ PWM更新

\`\`\`

完整链路：

\`\`\`text

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

\`\`\`

**---**

**# 50. 第六阶段代码完成标准**

完成：

\`\`\`text

uart\_debug.c

uart\_debug.h

\`\`\`

实现：

\`\`\`text

□ START

□ STOP

□ POS

□ SPD

□ STATUS

\`\`\`

例如：

\`\`\`text

M1 POS 90

M2 POS 180

\`\`\`

STM32：

\`\`\`text

Motor1 → 90°

Motor2 → 180°

\`\`\`

**---**

**# 51. 最终函数关系**

最终主要函数关系固定为：

\`\`\`text

main()

 │

 ├── Motor\_System\_Init()

 │       │

 │       ├── Motor\_Init(M1)

 │       ├── Motor\_Init(M2)

 │       ├── PWM Start

 │       └── Encoder Check

 │

 ├── HAL\_TIM\_Base\_Start\_IT(TIM3)

 │

 └── while(1)

         │

         └── Motor\_System\_Task()

                 │

                 └── UART / Debug

\`\`\`

TIM3：

\`\`\`text

TIM3\_IRQHandler()

       ↓

HAL\_TIM\_IRQHandler()

       ↓

HAL\_TIM\_PeriodElapsedCallback()

       ↓

Motor\_Control\_Loop()

       │

       ├── Motor\_Update(M1)

       │

       └── Motor\_Update(M2)

\`\`\`

Motor\_Update：

\`\`\`text

Motor\_Update()

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

\`\`\`

**---**

**# 52. 最终代码职责关系**

\`\`\`text

                    main.c

                       │

                       ▼

             motor\_control.c

                       │

          ┌────────────┴────────────┐

          ▼                         ▼

      motor1                     motor2

          │                         │

    ┌─────┼─────┐             ┌─────┼─────┐

    ▼     ▼     ▼             ▼     ▼     ▼

  TIM1  I2C1  AS5600         TIM2  I2C2  AS5600

    │           │               │           │

    ▼           ▼               ▼           ▼

   PWM        Angle             PWM        Angle

    │           │               │           │

    └─────┬─────┘               └─────┬─────┘

          ▼                           ▼

        FOC                         FOC

          │                           │

          ▼                           ▼

       Motor1                      Motor2

\`\`\`

**---**

**# 53. 最终工程完成后的模块边界**

\`\`\`text

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

\`\`\`

**\*\*最终结果就是：CubeMX 只负责 STM32 外设初始化；\`as5600.c\` 负责传感器；\`motor.c\` 负责 PWM/EN；\`foc.c\` 负责数学算法；\`pid.c\` 负责控制器；\`motor\_control.c\` 负责把这些模块串成一个完整的双电机控制系统；\`main.c\` 只负责启动系统。\*\***

此外，你当前工程里的 TIM1 为 20 kHz、TIM2 为约 1.1 kHz、TIM3 为 1 kHz，这三个时基会按你给出的现有 CubeMX 配置直接纳入上述软件架构；当前硬件没有 ADC 电流采样，因此本设计的当前版本闭环反馈是 **\*\*AS5600 位置反馈，控制输出为电压矢量 + SVPWM\*\***，而不是完整的电流采样闭环 FOC。 