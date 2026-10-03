#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 步兵发射机构模块：控制两个摩擦轮和一个拨弹盘，支持单发、三连发和连发，按裁判系统热量限制射频 / Infantry launcher Module controlling two friction wheels and a trigger disc, with single, three-round and continuous fire and a fire rate limited by the referee system heat
depends:
- id: QDU-Robomaster/CMD
  ref: same-or-dev
- id: QDU-Robomaster/RMMotor
  ref: same-or-dev
- id: QDU-Robomaster/Motor
  ref: same-or-dev
- id: QDU-Robomaster/Referee
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#include "Motor.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "timer.hpp"

namespace launcher::param
{
/// 拨弹盘每发转过的角度 (rad)
/// Angle the trigger disc advances per round (rad)
constexpr float TRIG_STEP = static_cast<float>(LibXR::TWO_PI) / 10.0f;
/// 判定卡弹的拨弹电机扭矩 (N·m)
/// Trigger motor torque that is taken as a jam (N·m)
constexpr float JAM_TORQUE = 0.028f;
/// 卡弹处理中后退与复位的切换间隔 (s)
/// Switching interval between backing off and returning during jam handling (s)
constexpr float JAM_TOGGLE_INTERVAL_SEC = 0.1f;
/// 按住超过该时间转为连发 (s)
/// Holding for longer than this switches to continuous fire (s)
constexpr float LONG_PRESS_THRESHOLD_SEC = 0.5f;
/// 热量控制的更新周期 (s)
/// Update period of the heat control (s)
constexpr float HEAT_TICK_SEC = 0.05f;
/// 发射进度判定的容差
/// Tolerance of the shot progress decision
constexpr float SHOT_PROGRESS_EPSILON = 1e-4f;
/// 拨弹盘到位判定的角度容差 (rad)
/// Angle tolerance for the trigger disc reaching its target (rad)
constexpr float TRIGGER_SETTLE_ANGLE = 0.2f * TRIG_STEP;
/// 允许发射的摩擦轮转速相对目标的余量 (rpm)
/// Margin below the target speed of the friction wheels that still allows firing (rpm)
constexpr float FRIC_READY_RPM_MARGIN = 200.0f;
/// 判定出弹的摩擦轮转速相对峰值的下降量 (rpm)
/// Friction wheel speed drop from the peak that is taken as a round leaving (rpm)
constexpr float FRIC_DROP_RPM = 150.0f;
}  // namespace launcher::param

/**
 * @brief 步兵发射机构模块：控制两个摩擦轮和一个拨弹盘，支持单发、三连发和连发，
 *        按裁判系统热量限制射频。
 *        Infantry launcher Module controlling two friction wheels and a trigger disc,
 *        with single, three-round and continuous fire and a fire rate limited by the
 *        referee system heat.
 */
class InfantryLauncher
{
 public:
  /**
   * @brief 发射状态。
   *        Launcher states.
   */
  enum class LauncherState : uint8_t
  {
    RELAX,   ///< 放松 Relax
    STOP,    ///< 停止发射 Firing stopped
    NORMAL,  ///< 正常发射 Normal firing
    JAMMED,  ///< 卡弹 Jammed
  };

  /**
   * @brief 发射机构事件，数值同时是 `GetEvent()` 上注册的事件 ID。
   *        Launcher events; the values are also the event IDs registered on
   *        `GetEvent()`.
   */
  enum class LauncherEvent : uint8_t
  {
    SET_FRICMODE_RELAX,  ///< 摩擦轮放松 Friction wheels relaxed
    SET_FRICMODE_SAFE,  ///< 摩擦轮安全：目标转速为 0 Friction wheels safe: target speed 0
    SET_FRICMODE_READY,     ///< 摩擦轮就绪 Friction wheels ready
    SET_SHOTMODE_SINGLE,    ///< 单发 Single shot
    SET_SHOTMODE_CONTINUE,  ///< 连发 Continuous fire
    SET_SHOTMODE_BOOST_3,   ///< 三连发 Three-round burst
  };

  /**
   * @brief 拨弹模式。
   *        Trigger modes.
   */
  enum class TrigMode : uint8_t
  {
    RELAX,     ///< 放松 Relax
    SAFE,      ///< 安全：保持当前位置 Safe: hold the current position
    SINGLE,    ///< 单发 Single shot
    CONTINUE,  ///< 连发 Continuous fire
    JAM,       ///< 卡弹处理 Jam handling
  };

  /**
   * @brief 裁判系统回传的发射相关数据。
   *        Launcher-related data reported by the referee system.
   */
  struct RefereeData
  {
    float cooling_rate = 0.0f;     ///< 枪管冷却值 Barrel cooling value
    float heat_limit = 0.0f;       ///< 枪管热量上限 Barrel heat limit
    float current_heat_17 = 0.0f;  ///< 17 mm 枪管当前热量 Current 17 mm barrel heat
    float bullet_speed = 0.0f;     ///< 弹速 (m/s) Bullet speed (m/s)
  };

  /**
   * @brief 发射器参数。
   *        Launcher parameters.
   */
  struct LauncherParam
  {
    float fric1_setpoint_speed;    ///< 摩擦轮目标转速初值 (rpm)
                                   ///< Initial friction wheel target speed (rpm)
    float target_bullet_speed;     ///< 目标弹速 (m/s) Target bullet speed (m/s)
    float bullet_speed_tolerance;  ///< 弹速容差 (m/s) Bullet speed tolerance (m/s)
    float trig_gear_ratio;         ///< 拨弹电机减速比 Trigger motor reduction ratio
    uint8_t num_trig_tooth;        ///< 拨弹盘齿数，用于拨弹速度参考限幅
                                   ///< Number of trigger disc teeth, used for the
                                   ///< trigger speed reference limit
  };

  /**
   * @brief 热量控制状态。
   *        Heat control state.
   */
  struct HeatLimit
  {
    float single_heat;     ///< 单发热量 Heat per round
    float launched_num;    ///< 本次更新判定的发射数 Rounds counted in this update
    float current_heat;    ///< 本地估计的当前热量 Locally estimated current heat
    float heat_threshold;  ///< 开始降低射频的剩余热量，以单发热量为单位
                           ///< Remaining heat at which the fire rate starts to drop, in
                           ///< units of the heat per round
    bool allow_fire;       ///< 是否允许发射 Whether firing is allowed
    float merge;           ///< 热量余量 Heat margin
  };

  /**
   * @brief 步兵发射机构配置参数。
   *        Infantry launcher configuration parameters.
   */
  struct Param
  {
    uint32_t task_stack_depth;                        ///< 线程栈深
                                                      ///< Thread stack depth
    LibXR::PID<float>::Param pid_param_trig_angle;    ///< 拨弹角度环 PID
                                                      ///< Trigger angle-loop PID
    LibXR::PID<float>::Param pid_param_trig_speed;    ///< 拨弹速度环 PID
                                                      ///< Trigger speed-loop PID
    LibXR::PID<float>::Param pid_param_fric_speed_0;  ///< 摩擦轮 0 速度环 PID
                                                      ///< Friction wheel 0 speed-loop PID
    LibXR::PID<float>::Param pid_param_fric_speed_1;  ///< 摩擦轮 1 速度环 PID
                                                      ///< Friction wheel 1 speed-loop PID
    LauncherParam launcher_param;                     ///< 发射器参数
                                                      ///< Launcher parameters
    LibXR::Thread::Priority thread_priority;          ///< 线程优先级
                                                      ///< Thread priority
    const char* launcher_cmd_topic_name;              ///< 订阅的发射控制命令 Topic 名称
                                                      ///< Name of the subscribed launcher
                                                      ///< command Topic
    const char* launcher_ref_topic_name;              ///< 订阅的裁判发射数据 Topic 名称
                                                      ///< Name of the subscribed referee
                                                      ///< launcher data Topic
  };

  /**
   * @brief 构造 InfantryLauncher，创建控制线程与 UI 定时任务并注册事件。
   *        Construct InfantryLauncher, create the control thread and the UI timer task,
   *        and register the events.
   *
   * @param motor_fric_0 摩擦轮 0 电机。
   *                     Friction wheel 0 motor.
   * @param motor_fric_1 摩擦轮 1 电机。
   *                     Friction wheel 1 motor.
   * @param motor_trig 拨弹电机。
   *                   Trigger motor.
   * @param cmd CMD 实例。
   *            CMD instance.
   * @param referee Referee 实例指针，用于 UI 绘制，为 `nullptr` 时不创建 UI 定时任务。
   *                Pointer to a Referee instance for UI drawing; the UI timer task is
   *                created only when it is not `nullptr`.
   * @param param 配置参数。
   *              Configuration parameters.
   */
  InfantryLauncher(
      RMMotor& motor_fric_0,
      RMMotor& motor_fric_1,
      RMMotor& motor_trig,
      CMD& cmd,
      Referee* referee,
      const Param& param = {.task_stack_depth = 4096, .pid_param_trig_angle = {.k = 1.0f, .p = 4000.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 4000.0f, .cycle = false}, .pid_param_trig_speed = {.k = 1.0f, .p = 0.0012f, .i = 0.0005f, .d = 0.0f, .i_limit = 1.0f, .out_limit = 1.0f, .cycle = false}, .pid_param_fric_speed_0 = {.k = 1.0f, .p = 0.002f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .cycle = false}, .pid_param_fric_speed_1 = {.k = 1.0f, .p = 0.002f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .cycle = false}, .launcher_param = {.fric1_setpoint_speed = 6500.0f, .target_bullet_speed = 25.0f, .bullet_speed_tolerance = 1.5f, .trig_gear_ratio = 36.0f, .num_trig_tooth = 10}, .thread_priority = LibXR::Thread::Priority::HIGH, .launcher_cmd_topic_name = "launcher_cmd", .launcher_ref_topic_name = "launcher_ref"})
      : motor_fric_0_(&motor_fric_0),
        motor_fric_1_(&motor_fric_1),
        motor_trig_(&motor_trig),
        pid_trig_angle_(param.pid_param_trig_angle),
        pid_trig_sp_(param.pid_param_trig_speed),
        pid_fric_0_(param.pid_param_fric_speed_0),
        pid_fric_1_(param.pid_param_fric_speed_1),
        param_(param.launcher_param),
        referee_(referee)
  {
    launcher_cmd_topic_name_ = param.launcher_cmd_topic_name;
    launcher_ref_topic_name_ = param.launcher_ref_topic_name;
    thread_.Create(this, ThreadFunc, "LauncherThread", param.task_stack_depth, param.thread_priority);

    if (referee_ != nullptr)
    {
      timer_ui_ = LibXR::Timer::CreateTask(DrawUI, this, UI_REFRESH_PERIOD_MS);
      LibXR::Timer::Add(timer_ui_);
      LibXR::Timer::Start(timer_ui_);
    }

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, InfantryLauncher* self, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          self->mutex_.Lock();
          self->LostCtrl();
          self->mutex_.Unlock();
        },
        this);

    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, InfantryLauncher* self, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          self->mutex_.Lock();
          self->SetMode(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX));
          self->mutex_.Unlock();
        },
        this);

    cmd.GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
    cmd.GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);

    auto event_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, InfantryLauncher* self, uint32_t event_id)
        {
          UNUSED(in_isr);
          self->mutex_.Lock();
          self->SetMode(event_id);
          self->mutex_.Unlock();
        },
        this);
    launcher_event.Register(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX),
                            event_callback);
    launcher_event.Register(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_SAFE),
                            event_callback);
    launcher_event.Register(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_READY),
                            event_callback);
    launcher_event.Register(static_cast<uint32_t>(LauncherEvent::SET_SHOTMODE_SINGLE),
                            event_callback);
    launcher_event.Register(static_cast<uint32_t>(LauncherEvent::SET_SHOTMODE_CONTINUE),
                            event_callback);
    launcher_event.Register(static_cast<uint32_t>(LauncherEvent::SET_SHOTMODE_BOOST_3),
                            event_callback);
  }

  /**
   * @brief 控制线程函数：订阅发射命令与裁判系统数据，每 2 ms 执行一轮更新与控制。
   *        Control thread function that subscribes to the launcher command and the
   *        referee data and runs one update and control iteration every 2 ms.
   *
   * @param self InfantryLauncher 实例指针。
   *             Pointer to the InfantryLauncher instance.
   */
  static void ThreadFunc(InfantryLauncher* self)
  {
    LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> cmd_sub(self->launcher_cmd_topic_name_);
    LibXR::Topic::ASyncSubscriber<Referee::LauncherPack> launcher_ref(
        self->launcher_ref_topic_name_);
    cmd_sub.StartWaiting();
    launcher_ref.StartWaiting();
    self->last_online_time_ = LibXR::Timebase::GetMicroseconds();
    while (true)
    {
      auto now = LibXR::Timebase::GetMicroseconds();
      self->dt_ = (now - self->last_online_time_).ToSecondf();
      self->last_online_time_ = now;

      if (cmd_sub.Available())
      {
        self->launcher_cmd_ = cmd_sub.GetData();
        cmd_sub.StartWaiting();
      }
      if (launcher_ref.Available())
      {
        const auto ref_pack = launcher_ref.GetData();
        self->ref_data_.heat_limit = ref_pack.rs.shooter_heat_limit;
        self->ref_data_.cooling_rate = ref_pack.rs.shooter_cooling_value;
        self->ref_data_.current_heat_17 = ref_pack.ph.launcher_id1_17_heat;
        self->ref_data_.bullet_speed = ref_pack.ld.bullet_speed;
        self->robot_level_ = ref_pack.rs.robot_level;
        launcher_ref.StartWaiting();
      }
      self->mutex_.Lock();
      self->Update();
      self->RunStateMachine();
      self->mutex_.Unlock();
      self->Control();
      LibXR::Thread::Sleep(2);
    }
  }

  /**
   * @brief 刷新电机反馈、累加拨弹盘角度并更新发射状态。
   *        Refresh the motor feedback, accumulate the trigger disc angle and update the
   *        launcher state.
   */
  void Update()
  {
    motor_fric_0_->Update();
    motor_fric_1_->Update();
    motor_trig_->Update();

    param_fric_0_ = motor_fric_0_->GetFeedback();
    param_fric_1_ = motor_fric_1_->GetFeedback();
    param_trig_ = motor_trig_->GetFeedback();

    float current_motor_angle = param_trig_.position;
    float delta_trig_angle = LibXR::CycleValue<float>(current_motor_angle) -
                             LibXR::CycleValue<float>(last_motor_angle_);
    trig_angle_ += delta_trig_angle / param_.trig_gear_ratio;
    last_motor_angle_ = current_motor_angle;

    UpdateLauncherState();
  }

  /**
   * @brief 计算拨弹与摩擦轮的 PID 输出并以 `MODE_CURRENT`
   * 下发；摩擦轮放松时放松全部电机。 Compute the PID outputs of the trigger and the
   * friction wheels and send them in `MODE_CURRENT`; all motors are relaxed when the
   * friction wheels are relaxed.
   */
  void Control()
  {
    float out_fric_0 = 0.0f;
    float out_fric_1 = 0.0f;
    Motor::Feedback trig_fb{};
    Motor::Feedback fric_0_fb{};
    Motor::Feedback fric_1_fb{};
    bool relax = false;

    SetFricTargetByEvent();

    if (launcher_event_ == LauncherEvent::SET_FRICMODE_RELAX)
    {
      relax = true;
    }
    else
    {
      if (trig_mode_ != TrigMode::RELAX)
      {
        TrigControl(out_trig_, target_trig_angle_, dt_);
      }
      FricControl(out_fric_0, out_fric_1, target_rpm_, dt_);
      trig_fb = param_trig_;
      fric_0_fb = param_fric_0_;
      fric_1_fb = param_fric_1_;
    }

    if (relax)
    {
      motor_trig_->Relax();
      motor_fric_0_->Relax();
      motor_fric_1_->Relax();
      return;
    }

    auto cmd_trig = Motor::MotorCmd{
        .mode = Motor::ControlMode::MODE_CURRENT,
        .reduction_ratio = param_.trig_gear_ratio,
        .velocity = out_trig_,
    };
    auto cmd_fric_0 = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                      .reduction_ratio = 1.0f,
                                      .velocity = out_fric_0};
    auto cmd_fric_1 = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                      .reduction_ratio = 1.0f,
                                      .velocity = out_fric_1};

    auto motor_control =
        [&](Motor* motor, const Motor::Feedback& fb, const Motor::MotorCmd& cmd)
    {
      if (fb.state == 0)
      {
        motor->Enable();
      }
      else if (fb.state != 0 && fb.state != 1)
      {
        motor->ClearError();
      }
      else
      {
        motor->Control(cmd);
      }
    };

    motor_control(motor_trig_, trig_fb, cmd_trig);
    motor_control(motor_fric_0_, fric_0_fb, cmd_fric_0);
    motor_control(motor_fric_1_, fric_1_fb, cmd_fric_1);
  }

  /**
   * @brief 切换摩擦轮模式或射击模式。
   *        Switch the friction wheel mode or the shot mode.
   *
   * @param mode `LauncherEvent` 的数值。
   *             Value of `LauncherEvent`.
   */
  void SetMode(uint32_t mode)
  {
    auto event = static_cast<LauncherEvent>(mode);
    switch (event)
    {
      case LauncherEvent::SET_SHOTMODE_SINGLE:
        shot_count_ = 1;
        continue_mode_ = false;
        ui_fire_mode_text_initialized_ = false;
        ui_refresh_tick_ = UI_FIRE_MODE_TEXT_PHASE;
        return;
      case LauncherEvent::SET_SHOTMODE_CONTINUE:
        shot_count_ = 1;
        continue_mode_ = true;
        ui_fire_mode_text_initialized_ = false;
        ui_refresh_tick_ = UI_FIRE_MODE_TEXT_PHASE;
        return;
      case LauncherEvent::SET_SHOTMODE_BOOST_3:
        shot_count_ = 3;
        continue_mode_ = false;
        ui_fire_mode_text_initialized_ = false;
        ui_refresh_tick_ = UI_FIRE_MODE_TEXT_PHASE;
        return;
      case LauncherEvent::SET_FRICMODE_RELAX:
      case LauncherEvent::SET_FRICMODE_SAFE:
      case LauncherEvent::SET_FRICMODE_READY:
        launcher_event_ = event;
        ui_fric_text_initialized_ = false;
        ui_refresh_tick_ = UI_FRIC_TEXT_PHASE;
        if (event != LauncherEvent::SET_FRICMODE_READY)
        {
          calibrated_ = false;
          calibration_pending_ = false;
          target_shot_index_ = 0;
          is_reverse_ = false;
        }
        break;
    }

    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();
  }

  /**
   * @brief 失去控制时复位全部状态，失能拨弹电机并放松摩擦轮。
   *        Reset all states when control is lost, disable the trigger motor and relax the
   *        friction wheels.
   */
  void LostCtrl()
  {
    launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
    launcher_state_ = LauncherState::RELAX;
    trig_mode_ = TrigMode::RELAX;

    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();

    target_trig_angle_ = trig_angle_;
    press_continue_ = false;
    calibrated_ = false;
    calibration_pending_ = false;
    trigger_step_active_ = false;
    target_shot_index_ = 0;
    is_reverse_ = false;
    shot_progress_ = 0.0f;
    launcher_cmd_.isfire = false;
    ui_fric_text_initialized_ = false;
    ui_fire_mode_text_initialized_ = false;
    ui_shot_position_initialized_ = false;
    ui_refresh_tick_ = UI_FRIC_TEXT_PHASE;

    motor_trig_->Disable();
    motor_fric_0_->Relax();
    motor_fric_1_->Relax();
  }

  /**
   * @brief 获取发射机构事件对象，`LauncherEvent` 的各个值注册在其上。
   *        Get the launcher event object on which every value of `LauncherEvent` is
   *        registered.
   *
   * @return 事件对象的引用。
   *         Reference to the event object.
   */
  LibXR::Event& GetEvent() { return launcher_event; }

  CMD::LauncherCMD launcher_cmd_{};  ///< 最近一次发射命令 Latest fire command  // NOLINT
  RefereeData
      ref_data_;  ///< 裁判系统回传的发射数据 Launcher data from the referee system

 private:
  // 发射机构 UI 使用的图层编号
  static constexpr uint8_t UI_LAYER_LAUNCHER = 1;
  // 发射机构 UI 文字共用的线宽和字号
  static constexpr uint16_t UI_CHAR_WIDTH = 2;
  static constexpr uint16_t UI_FONT_SIZE = 20;
  // 摩擦轮状态文字 ON/OFF 的显示位置
  static constexpr uint16_t UI_FRIC_TEXT_X = 160;
  static constexpr uint16_t UI_FRIC_TEXT_Y = 580;
  // 发射模式文字显示位置
  static constexpr uint16_t UI_FIRE_MODE_TEXT_X = 160;
  static constexpr uint16_t UI_FIRE_MODE_TEXT_Y = 540;
  // 实际落点圆圈显示位置
  static constexpr uint16_t UI_SHOT_POSITION_X = 960;
  static constexpr uint16_t UI_SHOT_POSITION_Y = 480;
  static constexpr uint16_t UI_SHOT_POSITION_RADIUS = 18;
  static constexpr uint16_t UI_SHOT_POSITION_WIDTH = 3;
  // 发射机构 UI 的刷新周期和分时重发节奏
  static constexpr uint32_t UI_REFRESH_PERIOD_MS = 80;
  static constexpr uint32_t UI_REFRESH_PHASE_COUNT = 3;
  static constexpr uint32_t UI_SHOT_POSITION_PHASE = 0;
  static constexpr uint32_t UI_FRIC_TEXT_PHASE = 1;
  static constexpr uint32_t UI_FIRE_MODE_TEXT_PHASE = 2;
  static constexpr uint32_t UI_TEXT_READD_DIV = 10;
  static constexpr uint32_t UI_FIGURE_READD_DIV = 60;

  RMMotor* motor_fric_0_;
  RMMotor* motor_fric_1_;
  RMMotor* motor_trig_;
  float last_trig_angle_ = 0.0f;
  Motor::Feedback param_fric_0_{};
  Motor::Feedback param_fric_1_{};
  Motor::Feedback param_trig_{};

  LibXR::PID<float> pid_trig_angle_;
  LibXR::PID<float> pid_trig_sp_;
  LibXR::PID<float> pid_fric_0_;
  LibXR::PID<float> pid_fric_1_;

  LauncherParam param_;
  Referee* referee_ = nullptr;
  LibXR::Event launcher_event;
  const char* launcher_cmd_topic_name_ = nullptr;
  const char* launcher_ref_topic_name_ = nullptr;
  LibXR::Thread thread_;
  LibXR::Timer::TimerHandle timer_ui_{};
  uint8_t robot_level_ = 5;

  float out_trig_ = 0.0f;

  float expect_trig_freq_ = 15.0f;
  float dt_ = 0.0f;
  float target_rpm_ = 0.0f;
  float expect_rpm_ = param_.fric1_setpoint_speed;
  float last_bullet_speed_ = -1.0f;
  float trig_freq_ = 0.0f;
  float trig_angle_ = 0.0f;
  float target_trig_angle_ = 0.0f;
  float last_motor_angle_ = 0.0f;
  float first_shot_angle_ = 0.0f;
  float fric_speed_peak_ = 0.0f;
  float jam_target_angle_ = 0.0f;
  int32_t target_shot_index_ = 0;
  uint8_t shot_count_ = 1;

  bool last_fire_notify_ = false;
  bool continue_mode_ = false;
  bool press_continue_ = false;
  bool is_reverse_ = false;
  bool heat_initialized_ = false;
  bool trigger_step_active_ = false;
  bool calibrated_ = false;
  bool calibration_pending_ = false;
  bool ui_layer_cleared_ = false;
  bool ui_fric_text_initialized_ = false;
  bool ui_fire_mode_text_initialized_ = false;
  bool ui_shot_position_initialized_ = false;
  uint32_t ui_refresh_tick_ = 0;

  float shot_progress_ = 0.0f;

  LibXR::MillisecondTimestamp fire_press_time_ = 0;
  LibXR::MillisecondTimestamp last_trig_time_ = 0;
  LibXR::MillisecondTimestamp last_jam_time_ = 0;
  LibXR::MillisecondTimestamp last_heat_time_ = 0;
  LibXR::MillisecondTimestamp last_check_time_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  LauncherEvent launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
  LauncherState launcher_state_ = LauncherState::RELAX;
  TrigMode trig_mode_ = TrigMode::RELAX;
  TrigMode last_trig_mode_ = TrigMode::RELAX;

  HeatLimit heat_limit_{
      .single_heat = 10.0f,
      .launched_num = 0.0f,
      .current_heat = 0.0f,
      .heat_threshold = 6.0f,
      .allow_fire = true,
      .merge = 0.0f,
  };
  LibXR::Mutex mutex_;

  void UpdateLauncherState()
  {
    if (param_trig_.torque > launcher::param::JAM_TORQUE)
    {
      launcher_state_ = LauncherState::JAMMED;
      return;
    }
    if (launcher_event_ != LauncherEvent::SET_FRICMODE_READY)
    {
      launcher_state_ = LauncherState::RELAX;
      return;
    }

    if (!heat_limit_.allow_fire)
    {
      launcher_state_ = LauncherState::STOP;
      return;
    }

    launcher_state_ = launcher_cmd_.isfire ? LauncherState::NORMAL : LauncherState::STOP;
  }

  void RunStateMachine()
  {
    auto now = LibXR::Timebase::GetMilliseconds();
    CurrentHeat(now);
    UpdateHeatControl(now);
    UpdateLauncherState();
    UpdateTriggerMode(now);
    UpdateTriggerSetpoint(now);

    last_fire_notify_ = launcher_cmd_.isfire;
  }

  void UpdateTriggerMode(LibXR::MillisecondTimestamp now)
  {
    switch (launcher_state_)
    {
      case LauncherState::RELAX:
        trig_mode_ = TrigMode::RELAX;
        press_continue_ = false;
        break;

      case LauncherState::STOP:
        trig_mode_ = TrigMode::SAFE;
        press_continue_ = false;
        break;

      case LauncherState::NORMAL:
        if (continue_mode_)
        {
          press_continue_ = true;
          trig_mode_ = TrigMode::CONTINUE;
        }
        else if (!last_fire_notify_)
        {
          fire_press_time_ = now;
          press_continue_ = false;
          trig_mode_ = TrigMode::SINGLE;
        }
        else
        {
          if (!press_continue_ && (now - fire_press_time_).ToSecondf() >
                                      launcher::param::LONG_PRESS_THRESHOLD_SEC)
          {
            press_continue_ = true;
          }
          trig_mode_ = press_continue_ ? TrigMode::CONTINUE : TrigMode::SINGLE;
        }
        break;

      case LauncherState::JAMMED:
        trig_mode_ = TrigMode::JAM;
        break;
    }
  }

  void UpdateTriggerSetpoint(LibXR::MillisecondTimestamp now)
  {
    const float step = launcher::param::TRIG_STEP;
    const float ready_rpm = expect_rpm_ - launcher::param::FRIC_READY_RPM_MARGIN;
    const float fric_speed =
        (fabsf(param_fric_0_.velocity) + fabsf(param_fric_1_.velocity)) * 0.5f;
    const bool fric_ready = fabsf(param_fric_0_.velocity) >= ready_rpm &&
                            fabsf(param_fric_1_.velocity) >= ready_rpm;

    auto indexed_target = [&]()
    { return first_shot_angle_ + step * static_cast<float>(target_shot_index_); };

    auto next_indexed_target = [&]()
    {
      target_shot_index_ =
          static_cast<int32_t>(ceilf((trig_angle_ - first_shot_angle_) / step));
      return indexed_target();
    };

    auto recover_from_jam = [&]()
    {
      target_trig_angle_ = calibrated_ ? next_indexed_target() : jam_target_angle_;
      is_reverse_ = false;
      trigger_step_active_ = true;
      last_trig_time_ = now;
    };

    if (trigger_step_active_)
    {
      fric_speed_peak_ = std::max(fric_speed_peak_, fric_speed);

      if (calibration_pending_ && !calibrated_ && fric_speed_peak_ >= ready_rpm &&
          fric_speed_peak_ - fric_speed >= launcher::param::FRIC_DROP_RPM)
      {
        calibrated_ = true;
        calibration_pending_ = false;
        first_shot_angle_ = trig_angle_;
        target_trig_angle_ = indexed_target();
        heat_limit_.current_heat =
            std::max(heat_limit_.current_heat, ref_data_.current_heat_17) +
            heat_limit_.single_heat;
        shot_progress_ = 0.0f;
        last_trig_angle_ = trig_angle_;
      }

      float angle_error = fabsf(target_trig_angle_ - trig_angle_);
      if (angle_error <= launcher::param::TRIGGER_SETTLE_ANGLE)
      {
        trigger_step_active_ = false;
        if (calibration_pending_ && !calibrated_)
        {
          calibration_pending_ = false;
        }
      }
    }
    else
    {
      fric_speed_peak_ = fric_speed;
    }

    auto start_shot = [&]()
    {
      if (!fric_ready)
      {
        return;
      }

      const float current_heat =
          std::max(heat_limit_.current_heat, ref_data_.current_heat_17);
      const float shot_heat = heat_limit_.single_heat * static_cast<float>(shot_count_);
      if (ref_data_.heat_limit <= 0.0f ||
          current_heat + shot_heat + heat_limit_.merge > ref_data_.heat_limit)
      {
        return;
      }

      if (calibrated_)
      {
        target_shot_index_ += static_cast<int32_t>(shot_count_);
        target_trig_angle_ = indexed_target();
      }
      else
      {
        calibration_pending_ = true;
        target_shot_index_ = static_cast<int32_t>(shot_count_) - 1;
        fric_speed_peak_ = fric_speed;
        target_trig_angle_ = trig_angle_ + step * static_cast<float>(shot_count_);
      }

      trigger_step_active_ = true;
      last_trig_time_ = now;
    };

    switch (trig_mode_)
    {
      case TrigMode::RELAX:
      case TrigMode::SAFE:
        target_trig_angle_ = trig_angle_;
        trigger_step_active_ = false;
        calibration_pending_ = false;
        is_reverse_ = false;
        break;

      case TrigMode::SINGLE:
        if (last_trig_mode_ == TrigMode::JAM)
        {
          recover_from_jam();
        }
        else if (last_trig_mode_ == TrigMode::SAFE || last_trig_mode_ == TrigMode::RELAX)
        {
          start_shot();
        }
        break;

      case TrigMode::CONTINUE:
      {
        float trig_freq = std::max(trig_freq_, 1e-3f);
        float interval_s = 1.0f / trig_freq;
        float since_last = (now - last_trig_time_).ToSecondf();
        if (last_trig_mode_ == TrigMode::JAM)
        {
          recover_from_jam();
        }
        else if (!trigger_step_active_ && since_last >= interval_s)
        {
          start_shot();
        }
      }
      break;

      case TrigMode::JAM:
      {
        trigger_step_active_ = false;
        if (last_trig_mode_ != TrigMode::JAM)
        {
          jam_target_angle_ = calibrated_ ? indexed_target() : target_trig_angle_;
          is_reverse_ = false;
        }
        if (last_trig_mode_ != TrigMode::JAM ||
            (now - last_jam_time_).ToSecondf() >=
                launcher::param::JAM_TOGGLE_INTERVAL_SEC)
        {
          target_trig_angle_ =
              is_reverse_ ? jam_target_angle_ : trig_angle_ - 0.3f * step;
          is_reverse_ = !is_reverse_;
          last_jam_time_ = now;
        }
      }
      break;
    }

    last_trig_mode_ = trig_mode_;
  }

  void SetFricTargetByEvent()
  {
    switch (launcher_event_)
    {
      case LauncherEvent::SET_FRICMODE_RELAX:
      case LauncherEvent::SET_FRICMODE_SAFE:
        target_rpm_ = 0.0f;
        break;
      case LauncherEvent::SET_FRICMODE_READY:
      {
        // 根据裁判系统回传弹速微调摩擦轮期望转速
        float bullet_speed = ref_data_.bullet_speed;
        if (bullet_speed < 0.0f || bullet_speed > 30.0f)
        {
          bullet_speed =
              param_.target_bullet_speed - 2.0f * param_.bullet_speed_tolerance;
        }

        if (last_bullet_speed_ != bullet_speed)
        {
          if (bullet_speed > param_.target_bullet_speed - param_.bullet_speed_tolerance)
          {
            expect_rpm_ -= 70.0f;
          }

          if (bullet_speed <
              param_.target_bullet_speed - 2.2f * param_.bullet_speed_tolerance)
          {
            expect_rpm_ += 50.0f;
          }
          last_bullet_speed_ = bullet_speed;
        }

        target_rpm_ = expect_rpm_;
        break;
      }
      case LauncherEvent::SET_SHOTMODE_SINGLE:
      case LauncherEvent::SET_SHOTMODE_CONTINUE:
      case LauncherEvent::SET_SHOTMODE_BOOST_3:
        break;
    }
  }

  void UpdateHeatControl(LibXR::MillisecondTimestamp now)
  {
    float delta_time = (now - last_heat_time_).ToSecondf();

    if (delta_time < launcher::param::HEAT_TICK_SEC)
    {
      return;
    }
    last_heat_time_ = now;

    float current_heat = std::max(heat_limit_.current_heat, ref_data_.current_heat_17);
    float residuary_heat = ref_data_.heat_limit - current_heat - heat_limit_.merge;
    heat_limit_.allow_fire =
        ref_data_.heat_limit > 0.0f && residuary_heat >= heat_limit_.single_heat;

    if (!heat_limit_.allow_fire)
    {
      trig_freq_ = 0.0f;
      return;
    }

    if (residuary_heat <= heat_limit_.single_heat * heat_limit_.heat_threshold)
    {
      float safe_freq = ref_data_.cooling_rate / heat_limit_.single_heat;
      float ratio =
          residuary_heat / (heat_limit_.single_heat * heat_limit_.heat_threshold);
      trig_freq_ = ratio * (expect_trig_freq_ - safe_freq) + safe_freq;
      return;
    }

    trig_freq_ = expect_trig_freq_;
  }

  void CurrentHeat(LibXR::MillisecondTimestamp now)
  {
    float delta_time = (now - last_check_time_).ToSecondf();

    if (!heat_initialized_)
    {
      heat_initialized_ = true;
      last_check_time_ = now;
      last_trig_angle_ = trig_angle_;
      return;
    }

    last_check_time_ = now;
    heat_limit_.launched_num = 0.0f;

    if (delta_time > 0.0f)
    {
      heat_limit_.current_heat -= ref_data_.cooling_rate * delta_time;
    }
    if (heat_limit_.current_heat <= 0.0f)
    {
      heat_limit_.current_heat = 0.0f;
    }
    heat_limit_.current_heat =
        std::max(heat_limit_.current_heat, ref_data_.current_heat_17);

    float delta_teeth = (trig_angle_ - last_trig_angle_) / launcher::param::TRIG_STEP;
    last_trig_angle_ = trig_angle_;

    if (launcher_event_ == LauncherEvent::SET_FRICMODE_READY)
    {
      shot_progress_ += delta_teeth;
      if (shot_progress_ < 0.0f)
      {
        shot_progress_ = 0.0f;
      }
    }
    else
    {
      shot_progress_ = 0.0f;
    }

    if (shot_progress_ >= 1.0f - launcher::param::SHOT_PROGRESS_EPSILON)
    {
      heat_limit_.launched_num = floorf(shot_progress_);
      shot_progress_ -= heat_limit_.launched_num;
      heat_limit_.current_heat += heat_limit_.single_heat * heat_limit_.launched_num;
    }
  }

  static void DrawUI(InfantryLauncher* launcher)
  {
    if (launcher->referee_ == nullptr)
    {
      return;
    }

    const uint16_t ROBOT_ID = launcher->referee_->GetRobotID();
    if (ROBOT_ID == 0)
    {
      return;
    }
    const uint16_t CLIENT_ID = launcher->referee_->GetClientID(ROBOT_ID);

    launcher->mutex_.Lock();
    const uint32_t UI_TICK = launcher->ui_refresh_tick_++;
    const uint32_t UI_PHASE = UI_TICK % UI_REFRESH_PHASE_COUNT;
    const bool FORCE_TEXT_READD = (UI_TICK % UI_TEXT_READD_DIV) < UI_REFRESH_PHASE_COUNT;
    const bool FRIC_ENABLED =
        launcher->launcher_event_ == LauncherEvent::SET_FRICMODE_READY;
    const uint8_t SHOT_COUNT = launcher->shot_count_;
    const TrigMode TRIG_MODE = launcher->trig_mode_;
    const bool CONTINUE_MODE = launcher->continue_mode_;
    const bool PRESS_CONTINUE = launcher->press_continue_;
    const bool UI_LAYER_CLEARED = launcher->ui_layer_cleared_;
    const bool UI_FRIC_TEXT_INITIALIZED = launcher->ui_fric_text_initialized_;
    const bool UI_FIRE_MODE_TEXT_INITIALIZED = launcher->ui_fire_mode_text_initialized_;
    const bool UI_SHOT_POSITION_INITIALIZED = launcher->ui_shot_position_initialized_;
    launcher->mutex_.Unlock();

    if (!UI_LAYER_CLEARED)
    {
      if (UI_PHASE != 0)
      {
        return;
      }

      Referee::UILayerDelete ui_del{};
      ui_del.delete_type = static_cast<uint8_t>(Referee::UIDeleteType::UI_DELETE_LAYER);
      ui_del.layer = UI_LAYER_LAUNCHER;
      if (launcher->referee_->SendUILayerDelete(ROBOT_ID, CLIENT_ID, ui_del) !=
          LibXR::ErrorCode::OK)
      {
        return;
      }

      launcher->mutex_.Lock();
      launcher->ui_layer_cleared_ = true;
      launcher->mutex_.Unlock();
      return;
    }

    if (UI_PHASE == UI_SHOT_POSITION_PHASE)
    {
      const bool REBUILD_SHOT_POSITION =
          !UI_SHOT_POSITION_INITIALIZED || (UI_TICK % UI_FIGURE_READD_DIV) == 0;
      if (REBUILD_SHOT_POSITION)
      {
        Referee::UIFigure shot_position_fig{};
        // 绘制实际落点圆圈
        launcher->referee_->FillCircle(
            shot_position_fig, "BPT", Referee::UIFigureOp::UI_OP_ADD, UI_LAYER_LAUNCHER,
            Referee::UIColor::UI_COLOR_YELLOW, UI_SHOT_POSITION_WIDTH, UI_SHOT_POSITION_X,
            UI_SHOT_POSITION_Y, UI_SHOT_POSITION_RADIUS);
        if (launcher->referee_->SendUIFigure(ROBOT_ID, CLIENT_ID, shot_position_fig) ==
            LibXR::ErrorCode::OK)
        {
          launcher->mutex_.Lock();
          launcher->ui_shot_position_initialized_ = true;
          launcher->mutex_.Unlock();
        }
      }
      return;
    }

    Referee::UICharacter char_fig{};
    if (UI_PHASE == UI_FRIC_TEXT_PHASE)
    {
      const bool REBUILD_FRIC_TEXT = !UI_FRIC_TEXT_INITIALIZED || FORCE_TEXT_READD;
      // 绘制发射机构的摩擦轮状态文字
      launcher->referee_->FillCharacter(
          char_fig, "FRC",
          REBUILD_FRIC_TEXT ? Referee::UIFigureOp::UI_OP_ADD
                            : Referee::UIFigureOp::UI_OP_MODIFY,
          UI_LAYER_LAUNCHER,
          FRIC_ENABLED ? Referee::UIColor::UI_COLOR_GREEN
                       : Referee::UIColor::UI_COLOR_ORANGE,
          UI_FONT_SIZE, UI_CHAR_WIDTH, UI_FRIC_TEXT_X, UI_FRIC_TEXT_Y,
          FRIC_ENABLED ? "FRIC ON" : "FRIC OFF");
      if (launcher->referee_->SendUICharacter(ROBOT_ID, CLIENT_ID, char_fig) ==
          LibXR::ErrorCode::OK)
      {
        launcher->mutex_.Lock();
        launcher->ui_fric_text_initialized_ = true;
        launcher->mutex_.Unlock();
      }
      return;
    }

    if (UI_PHASE == UI_FIRE_MODE_TEXT_PHASE)
    {
      const bool REBUILD_FIRE_MODE_TEXT =
          !UI_FIRE_MODE_TEXT_INITIALIZED || FORCE_TEXT_READD;
      // 绘制当前发射模式文字
      launcher->referee_->FillCharacter(
          char_fig, "FRM",
          REBUILD_FIRE_MODE_TEXT ? Referee::UIFigureOp::UI_OP_ADD
                                 : Referee::UIFigureOp::UI_OP_MODIFY,
          UI_LAYER_LAUNCHER, GetFireModeColor(SHOT_COUNT, TRIG_MODE, CONTINUE_MODE),
          UI_FONT_SIZE, UI_CHAR_WIDTH, UI_FIRE_MODE_TEXT_X, UI_FIRE_MODE_TEXT_Y,
          GetFireModeText(SHOT_COUNT, TRIG_MODE, CONTINUE_MODE, PRESS_CONTINUE));
      if (launcher->referee_->SendUICharacter(ROBOT_ID, CLIENT_ID, char_fig) ==
          LibXR::ErrorCode::OK)
      {
        launcher->mutex_.Lock();
        launcher->ui_fire_mode_text_initialized_ = true;
        launcher->mutex_.Unlock();
      }
      return;
    }
  }

  static const char* GetFireModeText(uint8_t shot_count, TrigMode trig_mode,
                                     bool continue_mode, bool press_continue)
  {
    if (trig_mode == TrigMode::CONTINUE || continue_mode || press_continue)
    {
      return "CONT";
    }
    if (shot_count >= 3)
    {
      return "BOOST_3";
    }
    return "SING";
  }

  static Referee::UIColor GetFireModeColor(uint8_t shot_count, TrigMode trig_mode,
                                           bool continue_mode)
  {
    if (trig_mode == TrigMode::CONTINUE || continue_mode)
    {
      return Referee::UIColor::UI_COLOR_CYAN;
    }
    if (shot_count >= 3)
    {
      return Referee::UIColor::UI_COLOR_YELLOW;
    }
    return Referee::UIColor::UI_COLOR_WHITE;
  }

  void TrigControl(float& out_trig, float target_trig_angle, float dt)
  {
    float plate_omega_ref = pid_trig_angle_.Calculate(
        target_trig_angle, trig_angle_, param_trig_.omega / param_.trig_gear_ratio, dt);
    float omega_limit =
        static_cast<float>(1.5f * LibXR::TWO_PI * trig_freq_ / param_.num_trig_tooth);
    float motor_omega_ref = std::clamp(plate_omega_ref, -omega_limit, omega_limit);
    out_trig = pid_trig_sp_.Calculate(motor_omega_ref,
                                      param_trig_.omega / param_.trig_gear_ratio, dt);
  }

  void FricControl(float& out_fric_0, float& out_fric_1, float target_rpm, float dt)
  {
    out_fric_0 = pid_fric_0_.Calculate(target_rpm, param_fric_0_.velocity, dt);
    out_fric_1 = pid_fric_1_.Calculate(target_rpm, param_fric_1_.velocity, dt);

    if (launcher_event_ == LauncherEvent::SET_FRICMODE_SAFE)
    {
      out_fric_0 /= 50.0f;
      out_fric_1 /= 50.0f;
    }
  }
};
