# InfantryLauncher

基于 LibXR / XRobot 1.0 的 RoboMaster 步兵发射模块，控制两个摩擦轮和一个拨弹盘，
支持单发、三连发、持续连发、首发弹位校准、卡弹恢复和裁判系统 UI。
主要实现位于 [InfantryLauncher.hpp](InfantryLauncher.hpp)。

当前版本使用拨弹角度环与速度环串级 PID、两路摩擦轮速度 PID，
通过 `Motor::ControlMode::MODE_CURRENT` 输出电流指令。
构造函数已移除 `Sliding*` 参数，模块依赖中也不再包含 `local/Sliding`。

## 依赖与接口

模块 ID 为 `QDU-Robomaster/InfantryLauncher`，使用 C++20。
`MODULE MANIFEST V2` 声明以下依赖，均采用 `same-or-dev` 引用：

- `QDU-Robomaster/CMD`：发射命令和控制连接事件。
- `QDU-Robomaster/RMMotor`：摩擦轮及拨弹电机。
- `QDU-Robomaster/Motor`：电机反馈和控制接口。
- `QDU-Robomaster/Referee`：裁判数据与 UI。

构造接口如下，`param` 的默认初始化值省略，详见头文件：

```cpp
InfantryLauncher(RMMotor& motor_fric_0, RMMotor& motor_fric_1,
                 RMMotor& motor_trig, CMD& cmd, Referee* referee,
                 const Param& param);
```

`CMD::LauncherCMD::isfire` 表示是否按住扳机。命令和裁判数据分别通过
`launcher_cmd_topic_name`、`launcher_ref_topic_name` 指定的 Topic 订阅。
`referee` 指针用于绘制 UI；传入 `nullptr` 时不创建 UI 定时任务，
裁判数据订阅仍由 Topic 名称决定。

电机、CMD 以及需要引用的 Referee 实例应先于发射器构造。
`GetEvent()` 提供事件绑定入口，控制循环由模块自身创建的线程执行。

## XRobot 配置

以下为发射器实例配置片段，参数采用头文件中的默认值。
其中 `motor_fric_0`、`motor_fric_1`、`motor_trig`、`cmd`、`ref` 应替换为
工程中已定义的实例 ID；PID、转速和减速比需按实际机构配置。

```yaml
modules:
  - module: QDU-Robomaster/InfantryLauncher
    id: launcher
    args:
      - motor_fric_0: motor_fric_0
      - motor_fric_1: motor_fric_1
      - motor_trig: motor_trig
      - cmd: cmd
      - referee: '&ref'
      - param:
          task_stack_depth: 4096
          pid_param_trig_angle: '{.k = 1.0f, .p = 4000.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 4000.0f, .cycle = false}'
          pid_param_trig_speed: '{.k = 1.0f, .p = 0.0012f, .i = 0.0005f, .d = 0.0f, .i_limit = 1.0f, .out_limit = 1.0f, .cycle = false}'
          pid_param_fric_speed_0: '{.k = 1.0f, .p = 0.002f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .cycle = false}'
          pid_param_fric_speed_1: '{.k = 1.0f, .p = 0.002f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .cycle = false}'
          launcher_param:
            fric1_setpoint_speed: 6500.0f
            target_bullet_speed: 25.0f
            bullet_speed_tolerance: 1.5f
            trig_gear_ratio: 36.0f
            num_trig_tooth: 10
          thread_priority: LibXR::Thread::Priority::HIGH
          launcher_cmd_topic_name: "launcher_cmd"
          launcher_ref_topic_name: "launcher_ref"
          heat_param:
            offline_heat_limit: 240.0f
            offline_cooling_rate: 40.0f
```

从旧的滑模配置迁移时，删除发射器 `args` 中的 `sliding` 项。
工程中遗留的 Sliding 实例不会再参与本模块的拨弹控制。

`num_trig_tooth` 当前用于拨弹速度参考限幅；每发拨弹角度与热量计数使用的
`launcher::param::TRIG_STEP` 固定为 `2π / 10`（36°）。
仅修改 YAML 中的齿数不会改变这个步进角。

## 模式与扳机行为

通过 `GetEvent()` 绑定 `InfantryLauncher::LauncherEvent`：

| 事件 | 行为 |
| --- | --- |
| `SET_FRICMODE_RELAX` | 放松摩擦轮和拨弹电机，清除校准与延迟观测状态 |
| `SET_FRICMODE_SAFE` | 摩擦轮目标转速设为 0，摩擦轮 PID 输出除以 50，退出拨弹与脱卡流程 |
| `SET_FRICMODE_READY` | 启动摩擦轮，允许按扳机命令和热量条件拨弹 |
| `SET_SHOTMODE_SINGLE` | 每次按下扳机请求一发，长按超过 0.5 秒进入持续拨弹 |
| `SET_SHOTMODE_CONTINUE` | 按住扳机持续拨弹，无需等待长按阈值 |
| `SET_SHOTMODE_BOOST_3` | 每次按下扳机请求三发；长按逻辑同样适用，连续请求仍使用三发步进 |

射击模式选择与摩擦轮模式分别保存。只有摩擦轮处于 `READY`，且速度、热量条件满足时，
才会启动一次正常拨弹。两个摩擦轮的速度绝对值均须达到期望转速减 200 rpm。

单发和三连发使用 `isfire` 从 `false` 变为 `true` 的上升沿触发。
若该次触发时条件不满足，不会排队保存这次短按请求。
连发则在持续按住期间检查：当前拨弹完成，且距上次启动已达到 `1 / trig_freq_` 秒，
才启动下一次拨弹。内部期望射频初值为 `15.0f`，实际节奏还受热量及到位时间影响。

松开扳机后停止发起后续拨弹，正常状态下继续完成当前拨弹目标并保持目标位置；
摩擦轮继续按所选模式运行。连发模式选择会保留，再次按下仍进入连发。
热量不足、模式切换、卡弹或失控会改变这一正常完成流程。

`CMD_EVENT_LOST_CTRL` 会复位状态、禁用拨弹电机并放松摩擦轮；
`CMD_EVENT_START_CTRL` 将摩擦轮模式设为 `RELAX`，需重新选择 `READY`。

## 校准与卡弹恢复

首发拨弹期间记录两个摩擦轮速度绝对值的平均值及其峰值。
当峰值达到就绪门限，且平均转速从峰值下降至少 `FRIC_DROP_RPM = 150 rpm` 时，
判定检测到出弹，记录当前拨弹角度作为弹位基准。
常规发射延迟观测使用同一掉速条件。

拨弹到位容差为 `TRIGGER_SETTLE_ANGLE = 0.2 * TRIG_STEP`，即约 7.2°。
未完成校准就到位时会结束本次校准等待，下一次发射可重新尝试校准。

仅在 `READY` 模式下，拨弹电机反馈扭矩大于 `JAM_TORQUE = 0.028f` 时进入脱卡。
当前判断使用正向扭矩值。脱卡过程以 0.1 秒为间隔，在回退约 0.3 齿与恢复目标之间切换。
松开扳机不会单独取消 `READY` 状态下的脱卡流程。

## 热量管理与发射计数

| 配置字段 | 默认值 | 使用条件 |
| --- | --- | --- |
| `heat_param.offline_heat_limit` | `240.0f` | 首次有效裁判参数到达前的热量上限，必须有限且大于 0 |
| `heat_param.offline_cooling_rate` | `40.0f` | 首次有效裁判参数到达前每秒冷却量，必须有限且不小于 0 |

裁判 Topic 尚未出现时，控制线程使用本地参数继续运行，待 Topic 出现后再订阅。
裁判热量上限必须大于 0，冷却速率和当前热量必须不小于 0，三者均须为有限值；
整组数据通过校验后才更新缓存。收到过有效参数后，继续使用最后一次有效值，
断流时不会自动切回离线参数。冷却速率为 0 是有效值。

`CurrentHeat()` 每轮控制循环结算冷却量和拨弹进度。
当前实现每轮都会将本地热量与缓存的裁判热量取较大值：
若裁判数据停止更新，缓存热量仍会作为本地估计的下限。
`UpdateHeatControl()` 以至少 50 ms 的间隔更新开火许可和射频限制。
每次新拨弹还会检查剩余热量是否足以容纳本次一发或三发请求。

`heat_limit_.launched_num` 表示本轮按拨弹角度计算出的发射数：

1. 初始化之后，每轮进入 `CurrentHeat()` 时先清零。
2. `READY` 状态下累计角度进度 `shot_progress_`，以一齿为一发。
3. 进度达到一发时取整数部分写入 `launched_num`，扣除已结算进度，保留小数部分。
4. 将 `single_heat * launched_num` 加入本地热量；当前 `single_heat = 10.0f`。

因此 `launched_num` 通常只在一个控制周期内为 `1`，下一轮又变为 `0`。
累计热量保存在 `current_heat` 中；累计发射总数需另设变量记录。
该计数根据拨弹运动估算，空转也可能被计入；首发校准成功时还会单独补计一发热量，
并清零角度进度，因此不能直接把 `launched_num` 的累计值当作实际出弹总数。
当前接口没有 `GetOverheatCount()`。

## 调试变量、线程与 UI

以下为调试器可观察的内部变量：

| 变量 | 含义 |
| --- | --- |
| `first_shot_delay_ms_` | 首发拨弹开始到掉速校准成功的耗时，单位 ms |
| `delay_time_` | 首发校准等待的控制循环次数 |
| `real_launch_delay_` | 常规发射计时起点到检测到掉速的耗时，单位 ms |
| `trigger_step_active_` | 当前是否仍在执行一次拨弹 |
| `target_shot_index_` | 弹位目标序号，与校准基准共同确定目标角度 |
| `heat_limit_.current_heat` | 当前本地估计热量 |
| `heat_limit_.launched_num` | 本轮角度进度结算出的发射数 |

常规发射超过 500 ms 仍未检测到出弹时，当前代码会重新设置计时起点。
此时 `real_launch_delay_` 后续记录的是距最近一次计时起点的时间，
不代表从最初拨弹开始累计的全部等待时间。
离开 `READY` 或失控时会清除相关延迟观测状态。

控制线程每轮末尾休眠 2 ms，实际周期还包含执行与调度时间。
命令读取、裁判数据更新和状态机在互斥锁内执行，随后解锁并调用 `Control()` 输出电机指令。
事件回调和 UI 状态快照也使用该锁。

提供有效 `referee` 指针且获得非零机器人 ID 后，UI 在图层 1 上显示
`FRIC ON/OFF`、`SING/CONT/BOOST_3` 和固定位置的落点参考圆圈。
定时任务间隔为 80 ms，三类图形分时处理，因此同一类图形通常每 240 ms 处理一次。
`FRIC ON` 表示选择了 `READY` 模式，实际能否拨弹仍由速度和热量条件决定。

## 在 BSP 工程中使用

在 BSP 的 `Modules/modules.yaml` 中指定模块分支，确认模块源索引指向所需仓库。
模块代码提交并推送后，在 BSP 根目录执行：

```sh
xrobot setup --update QDU-Robomaster/InfantryLauncher
xrobot sync
xrobot format
xrobot format --check
xrobot gen -c User/xrobot.yaml
cmake --preset debug
cmake --build --preset debug
```

上述配置路径与构建预设适用于配套 BSP；其他工程使用各自的配置和工具链。
若更新后报告旧 `sliding` 构造参数不匹配，先删除该参数，再运行同步和生成命令。
模块目录是独立 Git 仓库，源码和本文档应在模块仓库提交；
BSP 的 `xrobot.lock` 由 `xrobot setup` 生成，不手工修改。
