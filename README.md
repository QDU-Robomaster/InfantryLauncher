# InfantryLauncher

步兵发射机构模块：控制两个摩擦轮和一个拨弹盘，支持单发、三连发和连发，按裁判系统热量限制
射频，按回传弹速微调摩擦轮转速，并处理卡弹。

## 工作方式

- 构造时创建线程 `LauncherThread`（栈深 `param.task_stack_depth`，优先级 `param.thread_priority`），
  每轮：刷新电机反馈 → 热量 / 状态机 / 拨弹设定 → PID 输出，然后休眠 2 ms。
- 事件：`GetEvent()` 返回的 `LibXR::Event` 注册了 `InfantryLauncher::LauncherEvent` 的各个值：
  - 摩擦轮模式 `SET_FRICMODE_RELAX` / `SET_FRICMODE_SAFE` / `SET_FRICMODE_READY`：`RELAX` 时三个电机
    `Relax()`；`SAFE` 时摩擦轮目标为 0 且输出缩小为 1/50；`READY` 时两个摩擦轮以同一目标转速运行。
    离开 `READY` 时清除拨弹盘标定。
  - 射击模式 `SET_SHOTMODE_SINGLE`（每次 1 发）、`SET_SHOTMODE_BOOST_3`（每次 3 发）、
    `SET_SHOTMODE_CONTINUE`（按住连发）。
  - CMD 的 `CMD_EVENT_START_CTRL` 切到 `SET_FRICMODE_RELAX`；`CMD_EVENT_LOST_CTRL` 复位全部状态、
    失能拨弹电机并放松摩擦轮。
- 摩擦轮转速：目标转速初值为 `fric1_setpoint_speed` (rpm)。`READY` 下每当裁判系统回传的弹速变化时，
  弹速高于 `target_bullet_speed - bullet_speed_tolerance` 则目标降低 70 rpm，低于
  `target_bullet_speed - 2.2 × bullet_speed_tolerance` 则升高 50 rpm（回传弹速不在 0–30 m/s 时按
  `target_bullet_speed - 2 × bullet_speed_tolerance` 处理）。两个摩擦轮转速都不低于目标 − 200 rpm
  才允许发射。
- 拨弹：`launcher_cmd` 的 `isfire` 在 `READY` 且热量允许时进入发射状态。单发 / 三连发模式下，
  上升沿发射 `shot_count` 发，按住超过 0.5 s 转为连发；连发模式下按住即按当前射频连续发射。
  拨弹盘每发转过固定的 2π/10（代码常量，与 `num_trig_tooth` 无关）。进入 `READY` 后的第一次发射
  以摩擦轮转速相对峰值下降 ≥ 150 rpm 判定出弹，并据此标定拨弹盘的发射位置，之后按该位置分度。
  拨弹为角度环 + 速度环，速度参考限幅 1.5 × 2π × 射频 / `num_trig_tooth`。
- 卡弹：拨弹电机扭矩 > 0.028 N·m 时进入卡弹处理，每 0.1 s 在“后退 0.3 格”与原目标之间往复，
  扭矩恢复且仍在发射时重新对齐到下一格继续。
- 热量：单发热量按 10 计，热量估计取本地累计（按拨弹盘转过的格数计发射数、按冷却值衰减）与裁判系统
  17 mm 枪管热量的较大值。裁判系统热量上限为 0（尚未收到 `launcher_ref`）或剩余热量 < 10 时禁止发射；剩余热量 ≤ 60 时射频从 15 Hz 线性降到
  冷却值 / 10，否则射频为 15 Hz；某次发射会使热量超过上限时不发射。
- 电机下发：`MODE_CURRENT`；电机 `state == 0` 时先 `Enable()`，`state` 不为 0 / 1 时 `ClearError()`。
- 裁判系统 UI：`referee` 非空时，定时任务每 80 ms 在图层 1 上轮流绘制落点圆圈、`FRIC ON` /
  `FRIC OFF` 文字和射击模式文字（`SING` / `BOOST_3` / `CONT`）。

Topic：

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `launcher_cmd` | 订阅 | `CMD::LauncherCMD` | 发射命令 `isfire` |
| `launcher_ref` | 订阅 | `Referee::LauncherPack` | 热量上限、冷却值、17 mm 热量、弹速 |

## 依赖

- `QDU-Robomaster/RMMotor`：摩擦轮与拨弹电机。
- `QDU-Robomaster/Motor`：电机命令与反馈类型。
- `QDU-Robomaster/CMD`：发射命令类型与 CMD 事件。
- `QDU-Robomaster/Referee`：热量 / 弹速数据类型与 UI 绘制。

无外部软件包，仅使用 LibXR。

## 构造接口

```cpp
InfantryLauncher(RMMotor& motor_fric_0,
                 RMMotor& motor_fric_1,
                 RMMotor& motor_trig,
                 CMD& cmd,
                 Referee* referee,
                 const Param& param = {...});
```

依赖：

- `motor_fric_0`、`motor_fric_1`：`RMMotor`，两个摩擦轮电机。
- `motor_trig`：`RMMotor`，拨弹电机。
- `cmd`：`CMD` 实例。
- `referee`：`Referee*`，只用于 UI 绘制；填 `nullptr` 时不绘制 UI。热量与弹速数据来自
  `launcher_ref` Topic，与此参数无关。

配置（`Param`；PID 为 `LibXR::PID<float>::Param`，字段 `k, p, i, d, i_limit, out_limit, cycle`）：

- `task_stack_depth`：线程栈深，默认 4096。
- `pid_param_trig_angle`：拨弹角度环，默认 `k = 1, p = 4000, out_limit = 4000`。
- `pid_param_trig_speed`：拨弹速度环，默认 `k = 1, p = 0.0012, i = 0.0005, i_limit = 1, out_limit = 1`。
- `pid_param_fric_speed_0`、`pid_param_fric_speed_1`：摩擦轮速度环，默认 `k = 1, p = 0.002, out_limit = 1`。
- `launcher_param.fric1_setpoint_speed`：摩擦轮目标转速初值 (rpm)，默认 6500。
- `launcher_param.target_bullet_speed`：目标弹速 (m/s)，默认 25。
- `launcher_param.bullet_speed_tolerance`：弹速容差 (m/s)，默认 1.5。
- `launcher_param.trig_gear_ratio`：拨弹电机减速比，默认 36。
- `launcher_param.num_trig_tooth`：拨弹盘齿数，只用于拨弹速度参考限幅，默认 10。
- `thread_priority`：线程优先级，默认 `HIGH`。

## 使用

```sh
xrobot module add QDU-Robomaster/InfantryLauncher
xrobot setup
xrobot instance add QDU-Robomaster/InfantryLauncher
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把依赖填为已列出的实例 id：

```yaml
modules:
  - module: QDU-Robomaster/InfantryLauncher
    id: infantrylauncher_0
    args:
      - motor_fric_0: motor_fric_0
      - motor_fric_1: motor_fric_1
      - motor_trig: motor_trig
      - cmd: cmd
      - referee: referee
      - param:
          task_stack_depth: '4096'
          pid_param_trig_angle:
            k: 1.0f
            p: 4000.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 4000.0f
            cycle: 'false'
          pid_param_trig_speed:
            k: 1.0f
            p: 0.0012f
            i: 0.0005f
            d: 0.0f
            i_limit: 1.0f
            out_limit: 1.0f
            cycle: 'false'
          pid_param_fric_speed_0:
            k: 1.0f
            p: 0.002f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 1.0f
            cycle: 'false'
          pid_param_fric_speed_1:
            k: 1.0f
            p: 0.002f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 1.0f
            cycle: 'false'
          launcher_param:
            fric1_setpoint_speed: 6500.0f
            target_bullet_speed: 25.0f
            bullet_speed_tolerance: 1.5f
            trig_gear_ratio: 36.0f
            num_trig_tooth: '10'
          thread_priority: LibXR::Thread::Priority::HIGH
```

所有依赖都是其他模块实例的 id，须在本实例之前列出：`motor_fric_0`、`motor_fric_1`、`motor_trig`
为 `QDU-Robomaster/RMMotor` 实例，`cmd` 为 `QDU-Robomaster/CMD` 实例，`referee` 为
`QDU-Robomaster/Referee` 实例（`Referee*` 参数填实例 id 时取其地址；不需要 UI 时填 `nullptr`）。
本例没有需要 BSP 用 `XR_REGISTER` 注册的对象。模式事件通常由 `EventBinder` 从遥控器事件绑定到本实例
（`InfantryLauncher::LauncherEvent::SET_FRICMODE_READY` 等）。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/InfantryLauncher`
（在 BSP 中）打印当前的构造函数。
