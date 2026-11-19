# TrajectoryGenerator 使用说明与代码修改说明

## 一、模块概述

**文件**：`simpleFoc/common/trajectory.h`、`simpleFoc/common/trajectory.cpp`

**作用**：把阶跃的目标角度变成位置、速度、加速度都连续的参考轨迹，交给位置环跟踪，消除起步和停止时的冲击。

**两级结构**：

1. **梯形速度规划**：由"目标 − 规划位置"生成速度 `v1`，受 `vmax`、`amax` 限制。末段刹车使用离散精确解，最后一拍精确落在目标上。
2. **速度滑动平均（FIR）**：对 `v1` 做长度为 `smooth_time` 的盒式平均，得到输出速度，再积分得到位置。
   - 加速度突变被抹成斜坡，加加速度有限。
   - 滤波器增益为 1、积分面积不变，终点位置精确等于目标，不过冲。

**特点**：

- 纯算法类，不依赖硬件和 RT-Thread，可在 PC 上单独测试。
- 逐周期在线递推，运动中改目标、改参数都不需要重新规划。
- 静止且目标未变时，`update()` 立即返回，几乎不占 CPU。

## 二、参数说明

| 参数 | 单位 | 默认值（`FocAxis`） | 含义 |
|---|---|---|---|
| `vmax` | rad/s | 3.0 | 最大速度。建议小于 `motor.velocity_limit`（当前为 5），给位置环留修正余量 |
| `amax` | rad/s² | 10.0 | 最大加速度，输出实测不超过此值 |
| `smooth_time` | s | 0.15 | 平滑时间，越大起停越柔和 |
| `vff` | 0~1 | 1.0 | 速度前馈增益（属于 `FocAxis`，不属于 `TrajectoryGenerator`） |

**参数关系**：

- 峰值加加速度约为 `amax / smooth_time`。没有匀速段的短行程，或运动中反向时，最大为该值的 2 倍。
- 总行程时间 ≈ 梯形规划时间 + `smooth_time`。
- 窗口长度为 `smooth_time / dt`，最大 1023 拍。5 kHz 下 `smooth_time` 上限约 204 ms，超出会被钳位。
- 非法输入会被钳到下限：`vmax` ≥ 0.001，`amax` ≥ 0.01，`smooth_time` ≥ 0。`smooth_time = 0` 等于只做梯形规划。

## 三、类接口

```cpp
TrajectoryGenerator(float vmax = 3.0f, float amax = 10.0f, float smooth_time = 0.15f);

void  setLimits(float vmax, float amax, float smooth_time);  // 可在线调用
void  reset(float pos);          // 以 pos 为起点复位，速度/加速度清零，目标 = pos
void  setTarget(float target);   // 设置最终目标
float update(float dt);          // 推进一个周期，返回参考位置

void  enable(bool en);           // false 时旁路：直接输出阶跃目标，用于对比
float position() const;          // 参考位置
float velocity() const;          // 参考速度（可作速度前馈）
float acceleration() const;      // 参考加速度（差分得到，仅供观察）
bool  done() const;              // 已到位并静止
float target() const;
float vmax() const;
float amax() const;
float smoothTime() const;
bool  enabled() const;
```

## 四、使用方法

**基本用法（周期性调用）**：

```cpp
TrajectoryGenerator traj(3.0f, 10.0f, 0.15f);

traj.reset(current_angle);        // 初始化时必须以实际位置为起点
traj.setTarget(2.0f);             // 任意线程、任意时刻

// 固定周期（如 5 kHz 中断）里：
float ref = traj.update(1.0f / 5000.0f);
motor.feed_forward_velocity = traj.velocity();   // 可选前馈
motor.move(ref);
```

**使用要点**：

1. `dt` 要保持固定，并等于实际调用周期。窗口长度按 `smooth_time / dt` 计算。
2. 初始化后必须调用 `reset(实际位置)`，否则会从 0 开始规划，起步时有一次大跳变。
3. 角度是多圈累计值，不做 0~2π 折算，要和 `motor.shaft_angle` 的口径一致。
4. 线程安全：`update()` 在中断里调用；`setTarget()`、`setLimits()` 可在线程里调用，它们都是单个 float 写入，在 Cortex-M4 上是原子的。
5. 资源：每个实例约 4 KB RAM（滑动平均缓冲区）。多轴时每轴一个实例，互相独立。
6. 单步耗时：约几十个浮点运算加一次 `sqrtf`，5 kHz 下可以忽略。

## 五、运行时行为（已用 PC 仿真验证）

- **到位精度**：终点误差为 0，过冲在 1e-4 rad 以内（浮点误差量级）。
- **运动中改目标**：仍然平滑。
- **反向或来不及刹车**：会越过目标，再平滑折返，不会硬停造成速度突变。
- **旁路切换**：`enable(false)` 后输出阶跃，内部状态跟随目标，随时可切回。
- **已知限制**：
  - 运动中大幅改 `smooth_time`（窗口突然变短）会让输出速度跳变，建议静止时再改。
  - 改目标引起的反向，加加速度可能达到 `amax / smooth_time` 的 2 倍。

## 六、Shell 调试命令

```
traj                         显示当前参数
traj on | off                开关平滑（off 为阶跃，用于 A/B 对比）
traj <vmax> <amax> <smooth_ms> [vff]
                             例：traj 3 10 150 1.0
angle <rad>                  设置目标角度；显示 target / planned / actual
```

`smooth` 在 shell 里以毫秒为单位，C API 里以秒为单位。

## 七、代码修改清单

### 新增

- `simpleFoc/common/trajectory.h`、`trajectory.cpp`：算法实现。位于 Eclipse 工程根目录下，会自动参与编译，无需修改构建文件。

### `simpleFoc/port/foc_port.cpp`

- 新增 `#include "../common/trajectory.h"`。
- 新增常量：
  - 周期：`kLoopFreqHz` = 5000、`kLoopDt`。
  - 默认参数：`kTrajVmax`、`kTrajAmax`、`kTrajSmooth`、`kTrajVff`。
- `FocAxis`：
  - 新增成员 `TrajectoryGenerator traj` 和 `vff_gain`。
  - 删除成员 `target_angle`（原默认值 1.0）。`get_target_angle()`、`set_target_angle()` 改为直接读写 `traj`。
  - 新增接口：
    - `get_planned_angle`、`get_planned_velocity`
    - `set_trajectory`、`get_trajectory`
    - `set_trajectory_enable`、`get_trajectory_enable`
- `init()` 末尾新增 `sensor.update(); traj.reset(motor.shaftAngle());`，轨迹以当前实际位置为起点。
- `isr()`：

  ```cpp
  motor.loopFOC();
  const float ref_angle = traj.update(kLoopDt);
  motor.feed_forward_velocity = vff_gain * traj.velocity();
  motor.move(ref_angle);
  ```

  原来是 `motor.move(target_angle)`。
- 新增 C 包装函数：`foc_get_planned_angle`、`foc_get_planned_velocity`、`foc_set_trajectory`、`foc_get_trajectory`、`foc_set_trajectory_enable`、`foc_get_trajectory_enable`。

### `simpleFoc/port/foc_api.h`

- 声明上述 6 个新函数。

### `applications/app_shell.c`

- 新增 `#include <string.h>`。
- `angle` 命令：输出里的 `measured` 换成 `planned`。原来 `measured_angle` 从未被赋值，一直显示 0。
- 新增 `traj` 命令（见第六节）。

## 八、行为变化

1. **上电不再自动转到 1.0 rad。** 原来 `target_angle` 默认为 1.0，上电后会阶跃转过去。现在初始化后目标等于当前位置，上电保持不动。
2. **位置环给定值变了**：从用户设定的目标，变成规划后的参考值，同时多了规划速度前馈。
3. **`angle` 命令回显**：`planned` 在命令刚下发时仍是旧位置，随后逐渐走向目标，这是正常现象。

## 九、验证情况与注意事项

- **PC 仿真**：在 PC 上用 g++ 测试过，包括 10 种行程、多组参数、运动中改目标与反向、随机压力测试、旁路切换。测试代码放在系统临时目录，**没有保留在仓库里**。
- **未验证**：没有用 ARM 工具链编译整个工程，也没有上板。请先编译，再用 `traj off` 与 `traj on` 做 A/B 对比。
- **速度前馈是额外加入的**，不属于最初需求。如有副作用，可用 `traj <vmax> <amax> <ms> 0` 把 `vff` 设为 0 关闭。
- **双环稳定裕度**：实测中扯动电线引起的振荡，来自位置/速度双环的稳定裕度不足加底座未固定，不属于本模块。调参时请先固定底座，并按"先内环（速度环）、后外环（位置环）"的顺序进行。
