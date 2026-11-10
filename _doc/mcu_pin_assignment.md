# STM32F407ZGT6 引脚分配

统计对象：STM32F407ZGT6，LQFP144。GPIO 共 114 个（PA–PG 各 16 个，另加 PH0、PH1）。

数据来源：

- CubeMX 分配：`cubemx/cubemx.ioc`（STM32CubeMX 6.14.1）
- 复用初始化：`cubemx/Src/stm32f4xx_hal_msp.c`、`cubemx/Src/main.c`
- RT-Thread 引脚：`drivers/board.h`
- 时钟：`drivers/drv_clk.c` 调用 CubeMX 的 `SystemClock_Config()`

统计日期：2026-11-10。

## 1. 总览

| 项目 | 数量 |
|---|---|
| GPIO 总数 | 114 |
| CubeMX 已分配 | 49 |
| CubeMX 未分配 | 65 |
| 当前固件启动后实际配置 | 6 |

CubeMX 已分配 49 个引脚，按端口分布如下。

| 端口 | 已分配 | 未分配 |
|---|---|---|
| PA | 3 | 13 |
| PB | 4 | 12 |
| PC | 0 | 16 |
| PD | 12 | 4 |
| PE | 11 | 5 |
| PF | 10 | 6 |
| PG | 7 | 9 |
| PH | 2 | 0 |

## 2. CubeMX 已分配引脚

### 2.1 TIM1 PWM

| 引脚 | 信号 | 说明 |
|---|---|---|
| PA8 | TIM1_CH1 | PWM |
| PA9 | TIM1_CH2 | PWM |
| PA10 | TIM1_CH3 | PWM |

定时器为中心对齐模式，预分频 0，自动重装载 2099，比较值 1000。CH4、互补输出和刹车未使用。

### 2.2 USART1

| 引脚 | 信号 | 说明 |
|---|---|---|
| PB6 | USART1_TX | 发送 |
| PB7 | USART1_RX | 接收 |

参数：460800 8N1，无硬件流控。

### 2.3 I2C1

| 引脚 | 信号 | 说明 |
|---|---|---|
| PB8 | I2C1_SCL | 硬件 I2C，开漏 |
| PB9 | I2C1_SDA | 硬件 I2C，开漏 |

时钟 400 kHz。

### 2.4 HSE

| 引脚 | 信号 | 说明 |
|---|---|---|
| PH0 | RCC_OSC_IN | 外部晶振输入 |
| PH1 | RCC_OSC_OUT | 外部晶振输出 |

HSE 按 25 MHz 配置，PLL：M = 25，N = 336，P = 2，Q = 4，系统时钟 168 MHz。

### 2.5 FSMC SRAM Bank4

16 位数据总线，地址 A0–A18，片选 NE4，写使能打开。共 40 个引脚。

数据总线：

| 信号 | 引脚 | 信号 | 引脚 |
|---|---|---|---|
| D0 | PD14 | D8 | PE11 |
| D1 | PD15 | D9 | PE12 |
| D2 | PD0 | D10 | PE13 |
| D3 | PD1 | D11 | PE14 |
| D4 | PE7 | D12 | PE15 |
| D5 | PE8 | D13 | PD8 |
| D6 | PE9 | D14 | PD9 |
| D7 | PE10 | D15 | PD10 |

地址总线：

| 信号 | 引脚 | 信号 | 引脚 |
|---|---|---|---|
| A0 | PF0 | A10 | PG0 |
| A1 | PF1 | A11 | PG1 |
| A2 | PF2 | A12 | PG2 |
| A3 | PF3 | A13 | PG3 |
| A4 | PF4 | A14 | PG4 |
| A5 | PF5 | A15 | PG5 |
| A6 | PF12 | A16 | PD11 |
| A7 | PF13 | A17 | PD12 |
| A8 | PF14 | A18 | PD13 |
| A9 | PF15 |  |  |

控制信号：

| 信号 | 引脚 | 说明 |
|---|---|---|
| NOE | PD4 | 读使能 |
| NWE | PD5 | 写使能 |
| NBL0 | PE0 | 低字节屏蔽 |
| NBL1 | PE1 | 高字节屏蔽 |
| NE4 | PG12 | Bank4 片选 |

## 3. 未分配引脚

以下引脚未在 CubeMX 中分配。PA13、PA14 复位后默认用作 SWD，PB3、PB4、PA15 复位后默认用作 JTAG。

| 端口 | 未分配引脚 |
|---|---|
| PA | PA0–PA7、PA11–PA15 |
| PB | PB0–PB5、PB10–PB15（PB10、PB11 由软件 I2C 使用，见第 4 节） |
| PC | PC0–PC15 |
| PD | PD2、PD3、PD6、PD7 |
| PE | PE2–PE6 |
| PF | PF6–PF11 |
| PG | PG6–PG11、PG13–PG15 |
| PH | 无 |

## 4. 固件启动后实际配置

`applications/main.c` 覆盖了 `cubemx/Src/main.c` 里的 `__WEAK main()`。因此 `MX_TIM1_Init()`、`MX_I2C1_Init()`、`MX_FSMC_Init()`、`MX_USART1_UART_Init()` 在当前启动路径中不会执行。

上电后真正会配置引脚的路径有三条：

1. `drivers/drv_clk.c` 调用 `SystemClock_Config()`，使用 HSE，占用 PH0、PH1。
2. 控制台设备 `uart1` 打开时，`drivers/drv_usart.c` 按 `drivers/board.h` 把 PB6、PB7 配成 USART1 复用。`HAL_UART_Init()` 进入 CubeMX 的 `HAL_UART_MspInit()` 后，配置的仍是这两脚。
3. `drivers/drv_soft_i2c.c` 在板级初始化（`INIT_BOARD_EXPORT`）时，按 `drivers/board.h` 把 PB10、PB11 配成开漏输出，作为软件 I2C1。`rtconfig.h` 已打开 `RT_USING_I2C` 与 `RT_USING_I2C_BITOPS`。

| 引脚 | 配置来源 | 运行时作用 |
|---|---|---|
| PH0、PH1 | `SystemClock_Config()` | HSE |
| PB6 | `BSP_UART1_TX_PIN`，同时由 `HAL_UART_MspInit()` 配置 | USART1_TX |
| PB7 | `BSP_UART1_RX_PIN`，同时由 `HAL_UART_MspInit()` 配置 | USART1_RX |
| PB10 | `BSP_I2C1_SCL_PIN` | 软件 I2C SCL |
| PB11 | `BSP_I2C1_SDA_PIN` | 软件 I2C SDA |

PA8、PA9、PA10、PB8、PB9 以及第 2.5 节的全部 FSMC 引脚只存在于 CubeMX 工程中，当前启动不会把它们设为对应复用功能。

## 5. CubeMX 与 board.h 的差异

| 功能 | CubeMX | `drivers/board.h` |
|---|---|---|
| USART1 | PB6 TX、PB7 RX | PB6 TX、PB7 RX |
| I2C1 | PB8 SCL、PB9 SDA（硬件 I2C，启动时不初始化） | PB10 SCL、PB11 SDA（软件 I2C） |
| TIM1 CH1–CH3 | PA8、PA9、PA10 | 未定义，启动时不初始化 |

USART1 两边已经一致，PA9、PA10 留给 TIM1。软件 I2C 用的 PB10、PB11 在 CubeMX 中未分配。`board.h` 里时钟源宏仍是 `HSI`，实际时钟初始化走的是 CubeMX 的 HSE 配置。

SPI、QSPI、PWM、ADC、CAN、SDIO、ETH、USB 在 `board.h` 中均未打开。
