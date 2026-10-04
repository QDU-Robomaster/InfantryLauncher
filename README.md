# InfantryLauncher

步兵发射机构模块：控制两个摩擦轮和一个拨弹盘，支持单发、三连发和连发，按裁判系统热量限制射频 / Infantry launcher Module controlling two friction wheels and a trigger disc, with single, three-round and continuous fire and a fire rate limited by the referee system heat

## 1. 模块作用 / Purpose

构造时，InfantryLauncher 创建线程 `LauncherThread`（栈深 `param.task_stack_depth`，优先级 `param.thread_priority`）。线程每轮依次刷新电机反馈、执行热量计算与状态机、设定拨弹目标、计算 PID 输出并下发，然后休眠 2 ms。电机以 `MODE_CURRENT` 下发；电机 `state == 0` 时先 `Enable()`，`state` 不为 0 与 1 时 `ClearError()`。

事件：`GetEvent()` 返回的 `LibXR::Event` 注册了 `InfantryLauncher::LauncherEvent` 的各个值。

- 摩擦轮模式 `SET_FRICMODE_RELAX`、`SET_FRICMODE_SAFE`、`SET_FRICMODE_READY`：`RELAX` 时三个电机 `Relax()`；`SAFE` 时摩擦轮目标转速为 0，输出缩小为 1/50；`READY` 时两个摩擦轮以同一目标转速运行。离开 `READY` 时清除拨弹盘标定。
- 射击模式 `SET_SHOTMODE_SINGLE`（每次 1 发）、`SET_SHOTMODE_BOOST_3`（每次 3 发）、`SET_SHOTMODE_CONTINUE`（按住连发）。
- CMD 的 `CMD_EVENT_START_CTRL` 切换到 `SET_FRICMODE_RELAX`；`CMD_EVENT_LOST_CTRL` 复位全部状态，失能拨弹电机并放松摩擦轮。

摩擦轮转速：目标转速初值为 `fric1_setpoint_speed`（rpm）。`READY` 下每当裁判系统回传的弹速变化时，弹速高于 `target_bullet_speed - bullet_speed_tolerance` 则目标降低 70 rpm，低于 `target_bullet_speed - 2.2 × bullet_speed_tolerance` 则升高 50 rpm；回传弹速不在 0 至 30 m/s 时按 `target_bullet_speed - 2 × bullet_speed_tolerance` 处理。两个摩擦轮的转速都不低于目标减 200 rpm 时才允许发射。

拨弹：`launcher_cmd` 的 `isfire` 在 `READY` 且热量允许时进入发射状态。单发与三连发模式下，上升沿发射 `shot_count` 发，按住超过 0.5 s 转为连发；连发模式下按住即按当前射频连续发射。拨弹盘每发转过 2π/10。进入 `READY` 后的第一次发射，以摩擦轮转速相对峰值下降不小于 150 rpm 判定出弹，并据此标定拨弹盘的发射位置，之后按该位置分度。拨弹由角度环与速度环串联，速度参考限幅为 1.5 × 2π × 射频 / `num_trig_tooth`。

卡弹：拨弹电机扭矩大于 0.028 N·m 时进入卡弹处理，每 0.1 s 在“后退 0.3 格”与原目标之间往复，扭矩恢复且仍在发射时重新对齐到下一格并继续。

热量：单发热量按 10 计，热量估计取本地累计（按拨弹盘转过的格数计发射数，按冷却值衰减）与裁判系统 17 mm 枪管热量的较大值。裁判系统热量上限为 0（尚未收到 `launcher_ref`）或剩余热量小于 10 时禁止发射；剩余热量不超过 60 时射频从 15 Hz 线性降到冷却值 / 10，否则射频为 15 Hz；某次发射会使热量超过上限时，该次发射不执行。

裁判系统 UI：`referee` 非空时，定时任务每 80 ms 在图层 1 上轮流绘制落点圆圈、`FRIC ON` / `FRIC OFF` 文字和射击模式文字（`SING`、`BOOST_3`、`CONT`）。

Upon construction, InfantryLauncher creates the thread `LauncherThread` (stack depth `param.task_stack_depth`, priority `param.thread_priority`). Each iteration refreshes the motor feedback, runs the heat calculation and the state machine, sets the trigger target, computes the PID outputs and sends them, and then sleeps for 2 ms. The motors are sent in `MODE_CURRENT`; a motor with `state == 0` is `Enable()`d first, and `ClearError()` is called when `state` is neither 0 nor 1.

Events: `GetEvent()` returns a `LibXR::Event` on which every value of `InfantryLauncher::LauncherEvent` is registered.

- Friction wheel modes `SET_FRICMODE_RELAX`, `SET_FRICMODE_SAFE` and `SET_FRICMODE_READY`: in `RELAX` the three motors are `Relax()`ed; in `SAFE` the friction wheel target speed is 0 and the output is scaled down to 1/50; in `READY` both friction wheels run at the same target speed. Leaving `READY` clears the trigger disc calibration.
- Shot modes `SET_SHOTMODE_SINGLE` (1 round each time), `SET_SHOTMODE_BOOST_3` (3 rounds each time) and `SET_SHOTMODE_CONTINUE` (continuous fire while held).
- The CMD event `CMD_EVENT_START_CTRL` switches to `SET_FRICMODE_RELAX`; `CMD_EVENT_LOST_CTRL` resets all states, disables the trigger motor and relaxes the friction wheels.

Friction wheel speed: the initial target speed is `fric1_setpoint_speed` (rpm). In `READY`, whenever the bullet speed reported by the referee system changes, a speed above `target_bullet_speed - bullet_speed_tolerance` lowers the target by 70 rpm and a speed below `target_bullet_speed - 2.2 × bullet_speed_tolerance` raises it by 50 rpm; a reported bullet speed outside 0 to 30 m/s is treated as `target_bullet_speed - 2 × bullet_speed_tolerance`. Firing is allowed only when the speeds of both friction wheels are at least the target minus 200 rpm.

Trigger: with `READY` and heat available, `isfire` of `launcher_cmd` enters the firing state. In the single and three-round modes, a rising edge fires `shot_count` rounds and holding for more than 0.5 s switches to continuous fire; in the continuous mode, holding fires continuously at the current fire rate. The trigger disc advances 2π/10 per round. The first shot after entering `READY` is detected as a round leaving when the friction wheel speed falls at least 150 rpm from its peak, which calibrates the firing position of the trigger disc, and later rounds are indexed from that position. The trigger runs an angle loop in series with a speed loop, and the speed reference is limited to 1.5 × 2π × fire rate / `num_trig_tooth`.

Jam: a trigger motor torque above 0.028 N·m enters jam handling, which alternates every 0.1 s between "back off 0.3 tooth" and the original target; once the torque recovers and firing continues, the disc realigns to the next tooth and goes on.

Heat: one round counts as 10 heat, and the heat estimate is the larger of the local accumulation (rounds counted from the teeth the trigger disc advances, decayed by the cooling value) and the referee system 17 mm barrel heat. Firing is disabled when the referee system heat limit is 0 (no `launcher_ref` received yet) or the remaining heat is below 10; when the remaining heat is at most 60, the fire rate decreases linearly from 15 Hz to the cooling value / 10, otherwise the fire rate is 15 Hz; a shot that would push the heat over the limit is not fired.

Referee system UI: when `referee` is not null, a timer task draws in turn, every 80 ms on layer 1, the impact point circle, the `FRIC ON` / `FRIC OFF` text and the shot mode text (`SING`, `BOOST_3`, `CONT`).

## 2. 构造接口 / Constructor

```cpp
InfantryLauncher(RMMotor& motor_fric_0,
                 RMMotor& motor_fric_1,
                 RMMotor& motor_trig,
                 CMD& cmd,
                 Referee* referee,
                 const Param& param = {...});  // 节选 / excerpt
```

依赖：

- `motor_fric_0`、`motor_fric_1`：`RMMotor`，两个摩擦轮电机。
- `motor_trig`：`RMMotor`，拨弹电机。
- `cmd`：`CMD` 实例。
- `referee`：`Referee*`，用于 UI 绘制，为 `nullptr` 时不创建 UI 定时任务；热量与弹速数据来自 `launcher_ref_topic_name` 指定的 Topic。

配置参数（`Param`；PID 为 `LibXR::PID<float>::Param`，字段为 `k, p, i, d, i_limit, out_limit, cycle`）：

- `task_stack_depth`：线程栈深，默认 4096。
- `pid_param_trig_angle`：拨弹角度环，默认 `{.k = 1.0f, .p = 4000.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 4000.0f, .cycle = false}`。
- `pid_param_trig_speed`：拨弹速度环，默认 `{.k = 1.0f, .p = 0.0012f, .i = 0.0005f, .d = 0.0f, .i_limit = 1.0f, .out_limit = 1.0f, .cycle = false}`。
- `pid_param_fric_speed_0`、`pid_param_fric_speed_1`：摩擦轮速度环，默认均为 `{.k = 1.0f, .p = 0.002f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .cycle = false}`。
- `launcher_param.fric1_setpoint_speed`：摩擦轮目标转速初值，单位 rpm，默认 6500。
- `launcher_param.target_bullet_speed`：目标弹速，单位 m/s，默认 25。
- `launcher_param.bullet_speed_tolerance`：弹速容差，单位 m/s，默认 1.5。
- `launcher_param.trig_gear_ratio`：拨弹电机减速比，默认 36。
- `launcher_param.num_trig_tooth`：拨弹盘齿数，用于拨弹速度参考限幅，默认 10。
- `thread_priority`：线程优先级，默认 `LibXR::Thread::Priority::HIGH`。
- `launcher_cmd_topic_name`：订阅的发射控制命令 Topic 名称，默认 `"launcher_cmd"`，与 CMD 的 `launcher_cmd_topic_name` 一致。
- `launcher_ref_topic_name`：订阅的裁判系统发射数据 Topic 名称，默认 `"launcher_ref"`，与 Referee 的 `referee_launcher_tp_name` 一致。

Dependencies:

- `motor_fric_0`, `motor_fric_1`: `RMMotor` objects for the two friction wheel motors.
- `motor_trig`: the `RMMotor` of the trigger motor.
- `cmd`: the `CMD` instance.
- `referee`: `Referee*`, used for UI drawing; the UI timer task is created only when it is not `nullptr`. The heat and bullet speed data come from the Topic named by `launcher_ref_topic_name`.

Configuration parameters (`Param`; the PIDs are `LibXR::PID<float>::Param` with fields `k, p, i, d, i_limit, out_limit, cycle`):

- `task_stack_depth`: thread stack depth, default 4096.
- `pid_param_trig_angle`: trigger angle loop, default `{.k = 1.0f, .p = 4000.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 4000.0f, .cycle = false}`.
- `pid_param_trig_speed`: trigger speed loop, default `{.k = 1.0f, .p = 0.0012f, .i = 0.0005f, .d = 0.0f, .i_limit = 1.0f, .out_limit = 1.0f, .cycle = false}`.
- `pid_param_fric_speed_0`, `pid_param_fric_speed_1`: friction wheel speed loops, both default to `{.k = 1.0f, .p = 0.002f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .cycle = false}`.
- `launcher_param.fric1_setpoint_speed`: initial friction wheel target speed in rpm, default 6500.
- `launcher_param.target_bullet_speed`: target bullet speed in m/s, default 25.
- `launcher_param.bullet_speed_tolerance`: bullet speed tolerance in m/s, default 1.5.
- `launcher_param.trig_gear_ratio`: trigger motor reduction ratio, default 36.
- `launcher_param.num_trig_tooth`: number of trigger disc teeth, used for the trigger speed reference limit, default 10.
- `thread_priority`: thread priority, default `LibXR::Thread::Priority::HIGH`.
- `launcher_cmd_topic_name`: name of the subscribed launcher command Topic, default `"launcher_cmd"`, matching the `launcher_cmd_topic_name` of CMD.
- `launcher_ref_topic_name`: name of the subscribed referee launcher data Topic, default `"launcher_ref"`, matching the `referee_launcher_tp_name` of Referee.

## 3. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `param.launcher_cmd_topic_name`（默认 `launcher_cmd`） | 订阅 | `CMD::LauncherCMD` | 发射命令 `isfire` |
| `param.launcher_ref_topic_name`（默认 `launcher_ref`） | 订阅 | `Referee::LauncherPack` | 热量上限、冷却值、17 mm 热量、弹速 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `param.launcher_cmd_topic_name` (default `launcher_cmd`) | Subscribe | `CMD::LauncherCMD` | Fire command `isfire` |
| `param.launcher_ref_topic_name` (default `launcher_ref`) | Subscribe | `Referee::LauncherPack` | Heat limit, cooling value, 17 mm heat, bullet speed |

## 4. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/InfantryLauncher` 写入的实例，依赖填写为其他模块实例的 id：`motor_fric_0`、`motor_fric_1`、`motor_trig` 取自 `QDU-Robomaster/RMMotor` 实例，`cmd` 取自 `QDU-Robomaster/CMD` 实例，`referee` 取自 `QDU-Robomaster/Referee` 实例的地址（`'&ref'`），它们须在本实例之前列出。模式事件通常由 `EventBinder` 从遥控器事件绑定到本实例（`InfantryLauncher::LauncherEvent::SET_FRICMODE_READY` 等）。

An instance written by `xrobot instance add QDU-Robomaster/InfantryLauncher`, with the dependencies set to the ids of other Module instances: `motor_fric_0`, `motor_fric_1` and `motor_trig` come from `QDU-Robomaster/RMMotor` instances, `cmd` from a `QDU-Robomaster/CMD` instance, and `referee` from the address of a `QDU-Robomaster/Referee` instance (`'&ref'`); they are listed before this instance. The mode events are usually bound from the remote controller events to this instance by `EventBinder` (`InfantryLauncher::LauncherEvent::SET_FRICMODE_READY` and others).

```yaml
modules:
  - module: QDU-Robomaster/InfantryLauncher
    id: launcher
    args:
      - motor_fric_0: motor_fric_front_left
      - motor_fric_1: motor_fric_front_right
      - motor_trig: motor_trig
      - cmd: cmd
      - referee: '&ref'
      - param:
          task_stack_depth: 1536
          pid_param_trig_angle:
            k: 1.0f
            p: 40.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.0f
            cycle: false
          pid_param_trig_speed:
            k: 1.0f
            p: 0.15f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.0f
            cycle: false
          pid_param_fric_speed_0:
            k: 0.8f
            p: 0.0008f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.6f
            cycle: false
          pid_param_fric_speed_1:
            k: 0.8f
            p: 0.0008f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.6f
            cycle: false
          launcher_param:
            fric1_setpoint_speed: 5800.0f
            target_bullet_speed: 21.0f
            bullet_speed_tolerance: 1.0f
            trig_gear_ratio: 36.0f
            num_trig_tooth: 10
          thread_priority: LibXR::Thread::Priority::HIGH
          launcher_cmd_topic_name: "launcher_cmd"
          launcher_ref_topic_name: "launcher_ref"
```

## 5. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/RMMotor`：摩擦轮与拨弹电机。
- `QDU-Robomaster/Motor`：电机命令与反馈类型。
- `QDU-Robomaster/CMD`：发射命令类型与 CMD 事件。
- `QDU-Robomaster/Referee`：热量与弹速数据类型，以及 UI 绘制。
- LibXR。

硬件：两个摩擦轮电机和一个拨弹电机，均通过 `RMMotor` 实例接入。

Dependencies:

- `QDU-Robomaster/RMMotor`: friction wheel and trigger motors.
- `QDU-Robomaster/Motor`: motor command and feedback types.
- `QDU-Robomaster/CMD`: launcher command type and CMD events.
- `QDU-Robomaster/Referee`: heat and bullet speed data types, and UI drawing.
- LibXR.

Hardware: two friction wheel motors and one trigger motor, all attached through `RMMotor` instances.
