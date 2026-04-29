#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args: []
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <initializer_list>

#include "CMD.hpp"
#include "Motor.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "app_framework.hpp"
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

namespace launcher::param {
constexpr float TRIGSTEP = static_cast<float>(LibXR::TWO_PI) / 10;
constexpr float JAM_TOR = 0.1f;
constexpr float DELTA_RPM = 150.0f;  // rpm
constexpr float BOOST_FIRE_FREQ = 20.0f;

struct Burst5sLevel {
  uint16_t heat_limit;
  uint16_t heat_cooling;
  float burst_freq;
};

constexpr Burst5sLevel BURST_5S_TABLE[] = {
    {170, 5, 3.9f},  {180, 7, 4.3f},  {190, 9, 4.7f},  {200, 11, 5.1f},
    {210, 12, 5.4f}, {220, 13, 5.7f}, {230, 14, 6.0f}, {240, 16, 6.4f},
    {250, 18, 6.8f}, {260, 20, 7.2f},
};
}  // namespace launcher::param

class InfantryLauncher {
 public:
  enum class FireMode : uint8_t {
    SINGLE,
    CONTINUE,
    BOOST,
  };

  enum class FireModeEvent : uint8_t {
    SET_FIREMODE_SINGLE = 0x10,
    SET_FIREMODE_CONTINUE,
    SET_FIREMODE_BOOST,
  };

  enum class LauncherState : uint8_t {
    RELAX,
    STOP,
    NORMAL,
  };
  /*控制摩擦轮*/
  enum class LauncherEvent : uint8_t {
    SET_FRICMODE_RELAX,
    SET_FRICMODE_SAFE,
    SET_FRICMODE_READY,
  };
  enum class TRIGMODE : uint8_t { RELAX, SAFE, SINGLE, CONTINUE, CALI };

  typedef struct {
    float heat_limit;
    float heat_cooling;
    float bullet_speed;
  } RefereeData;
  struct LauncherParam {
    /*摩擦轮转速*/
    float fric_setpoint_speed;
    /*拨弹盘电机减速比*/
    float trig_gear_ratio;
    /*拨齿数目*/
    uint8_t num_trig_tooth;
    /*弹频*/
    float expect_trig_freq_;
  };
  typedef struct {
    float single_heat;
    float launched_num;
    float current_heat;
    float heat_threshold;
    bool allow_fire;
  } HeatLimit;
  /**
   * @brief Launcher 构造函数
   *
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param task_stack_depth 任务堆栈深度
   * @param pid_param_trig 拨弹盘PID参数
   * @param pid_param_fric 摩擦轮PID参数
   * @param motor_can1 CAN1上的电机实例（trig）
   * @param motor_can2 CAN2上的电机实例 (fric)
   * @param  min_launch_delay_
   * @param  default_bullet_speed_ 默认弹丸初速度
   * @param  fric_radius_ 摩擦轮半径
   * @param trig_gear_ratio_ 拨弹电机减速比
   * @param  num_trig_tooth_ 拨弹盘中一圈能存储几颗弹丸
   * @param bullet_speed 弹丸速度
   */
  InfantryLauncher(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                   RMMotor* motor_fric_front_left,
                   RMMotor* motor_fric_front_right,
                   RMMotor* motor_fric_back_left,
                   RMMotor* motor_fric_back_right, RMMotor* motor_trig,
                   uint32_t task_stack_depth,
                   LibXR::PID<float>::Param pid_param_trig_angle,
                   LibXR::PID<float>::Param pid_param_trig_speed,
                   LibXR::PID<float>::Param pid_param_fric_0,
                   LibXR::PID<float>::Param pid_param_fric_1,
                   LauncherParam launch_param, CMD* cmd)
      : motor_fric_0_(motor_fric_front_left),
        motor_fric_1_(motor_fric_front_right),
        motor_trig_(motor_trig),
        pid_trig_angle_(pid_param_trig_angle),
        pid_trig_sp_(pid_param_trig_speed),
        pid_fric_0_(pid_param_fric_0),
        pid_fric_1_(pid_param_fric_1),
        param_(launch_param),
        default_expect_trig_freq_(launch_param.expect_trig_freq_),

        cmd_(cmd) {
    UNUSED(hw);
    UNUSED(app);
    UNUSED(motor_fric_back_left);
    UNUSED(motor_fric_back_right);
    UNUSED(cmd);
    thread_.Create(this, ThreadFunction, "LauncherThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, InfantryLauncher* launcher, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          launcher->LostCtrl();
        },
        this);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, InfantryLauncher* launcher, uint32_t event_id) {
          UNUSED(in_isr);
          launcher->SetMode(event_id);
        },
        this);

    auto fire_mode_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, InfantryLauncher* launcher, uint32_t event_id) {
          UNUSED(in_isr);
          launcher->SetFireModeByEvent(event_id);
        },
        this);

    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
    launcher_event_handler_.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX), callback);
    launcher_event_handler_.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_SAFE), callback);
    launcher_event_handler_.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_READY), callback);
    launcher_event_handler_.Register(
        static_cast<uint32_t>(FireModeEvent::SET_FIREMODE_SINGLE),
        fire_mode_callback);
    launcher_event_handler_.Register(
        static_cast<uint32_t>(FireModeEvent::SET_FIREMODE_CONTINUE),
        fire_mode_callback);
    launcher_event_handler_.Register(
        static_cast<uint32_t>(FireModeEvent::SET_FIREMODE_BOOST),
        fire_mode_callback);

    auto launcher_cmd_callback = LibXR::Callback<LibXR::RawData&>::Create(
        [](bool in_isr, InfantryLauncher* Launcher, LibXR::RawData& raw_data) {
          UNUSED(in_isr);
          CMD::LauncherCMD cmd_lau =
              *reinterpret_cast<CMD::LauncherCMD*>(raw_data.addr_);
          Launcher->launcher_cmd_.isfire = cmd_lau.isfire;
        },
        this);

    auto tp_cmd_launcher =
        LibXR::Topic(LibXR::Topic::Find("launcher_cmd", nullptr));

    tp_cmd_launcher.RegisterCallback(launcher_cmd_callback);
  }

  static void ThreadFunction(InfantryLauncher* launcher) {
    LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> launcher_cmd_tp(
        "launcher_cmd");
    LibXR::Topic::ASyncSubscriber<Referee::LauncherPack> launcher_referee_tp(
        "launcher_ref");
    launcher_cmd_tp.StartWaiting();
    launcher_referee_tp.StartWaiting();

    while (1) {
      if (launcher_cmd_tp.Available()) {
        launcher->launcher_cmd_ = launcher_cmd_tp.GetData();
        launcher_cmd_tp.StartWaiting();
      }
      if (launcher_referee_tp.Available()) {
        launcher->referee_data_.heat_cooling =
            launcher_referee_tp.GetData().rs.shooter_cooling_value;
        launcher->referee_data_.heat_limit =
            launcher_referee_tp.GetData().rs.shooter_heat_limit;
        launcher_referee_tp.StartWaiting();
      }

      launcher->Update();
      launcher->Heat();
      launcher->SetTrig();
      launcher->SetFric();
      launcher->Control();
      launcher->shoot_number_.Publish(launcher->number_);
      LibXR::Thread::Sleep(2);
    }
  }
  /**
   * @brief 更新函数
   *
   */
  void Update() {
    this->now_ = LibXR::Timebase::GetMicroseconds();
    dt_ = (now_ - last_online_time_).ToSecondf();
    last_online_time_ = now_;

    if (referee_data_.heat_cooling == 0 or referee_data_.heat_limit == 0) {
      referee_data_.heat_cooling = 20;
      referee_data_.heat_limit = 200;
    }
    if (referee_data_.bullet_speed < 15.0f or
        referee_data_.bullet_speed > 30.0f) {
      referee_data_.bullet_speed = 21.0f;
    }

    heat_limit_.single_heat = 10.0f;
    heat_limit_.heat_threshold = 8.0f;

    motor_fric_0_->Update();
    motor_fric_1_->Update();
    motor_trig_->Update();

    param_fric_0_ = motor_fric_0_->GetFeedback();
    param_fric_1_ = motor_fric_1_->GetFeedback();
    param_trig_ = motor_trig_->GetFeedback();

    static float last_motor_angle = 0.0f;
    static bool initialized = false;

    float current_motor_angle = param_trig_.position;

    if (!initialized) {
      last_motor_angle = current_motor_angle;
      initialized = true;
    }

    float delta_trig_angle = LibXR::CycleValue<float>(current_motor_angle) -
                             LibXR::CycleValue<float>(last_motor_angle);

    trig_angle_ += delta_trig_angle / param_.trig_gear_ratio;
    last_motor_angle = current_motor_angle;

    last_launcherstate_ = launcherstate_;

    bool single_fire_requested = fire_mode_ == FireMode::SINGLE &&
                                 launcher_cmd_.isfire && !last_fire_notify_;
    bool continuous_fire_requested =
        fire_mode_ != FireMode::SINGLE && launcher_cmd_.isfire;

    if (launcher_event_ != LauncherEvent::SET_FRICMODE_READY) {
      launcherstate_ = LauncherState::RELAX;
    } else if (!heat_limit_.allow_fire) {
      launcherstate_ = LauncherState::STOP;
    } else if (single_fire_requested || continuous_fire_requested) {
      launcherstate_ = LauncherState::NORMAL;
    } else {
      launcherstate_ = LauncherState::STOP;
    }

    if ((last_launcherstate_ == LauncherState::RELAX and
         launcherstate_ != LauncherState::RELAX) or
        (launcher_event_ == LauncherEvent::SET_FRICMODE_READY and
         (param_fric_0_.velocity <= 1000 and param_fric_1_.velocity <= 1000))) {
      need_cali_ = true;
    }
  }

  void Heat() {
    /*每周期都计算此周期的剩余热量*/
    heat_limit_.current_heat +=
        heat_limit_.single_heat * heat_limit_.launched_num;

    heat_limit_.launched_num = 0;

    if (heat_limit_.current_heat <
        (static_cast<float>(referee_data_.heat_cooling * dt_))) {
      heat_limit_.current_heat = 0;
    } else {
      heat_limit_.current_heat -=
          static_cast<float>(referee_data_.heat_cooling * dt_);
    }

    float residuary_heat = referee_data_.heat_limit - heat_limit_.current_heat;
    /*不同剩余热量启用不同实际弹频*/
    if (residuary_heat > heat_limit_.single_heat) {
      if (residuary_heat <= heat_limit_.single_heat * 2) {
        float max_freq = referee_data_.heat_cooling / heat_limit_.single_heat;
        float ratio = (residuary_heat - heat_limit_.single_heat) /
                      heat_limit_.single_heat;
        trig_freq_ = max_freq * (1.0f - ratio);
      } else if (residuary_heat <=
                 heat_limit_.single_heat * heat_limit_.heat_threshold) {
        float safe_freq =
            referee_data_.heat_cooling / heat_limit_.single_heat / 1.2f;
        float ratio =
            (residuary_heat - heat_limit_.single_heat * 2) /
            (heat_limit_.single_heat * (heat_limit_.heat_threshold - 2));
        trig_freq_ =
            (ratio * (param_.expect_trig_freq_ - safe_freq) + safe_freq) / 1.8f;

      } else {
        // trig_freq_ =
        // std::min(static_cast<uint16_t>(param_.expect_trig_freq_),
        //   static_cast<uint16_t>(referee_data_.heat_limit/10));
        trig_freq_ = GetModeTargetFreq();
      }
    } else {
      trig_freq_ = 0.00001;
      last_trig_time_ = now_; /*重置时间戳*/
    }
  }

  LibXR::Event& GetEvent() { return launcher_event_handler_; }

  void SetTrig() {
    /*根据状态选择拨弹盘模式*/
    switch (launcherstate_) {
      case LauncherState::RELAX:
        trig_mod_ = TRIGMODE::RELAX;
        need_cali_ = false;
        break;
      case LauncherState::STOP:
        if (trig_mod_ == TRIGMODE::RELAX) {
          target_trig_angle_ = trig_angle_;

          cal_trig_angle_ = trig_angle_;
        }
        trig_mod_ = TRIGMODE::SAFE;
        break;
      case LauncherState::NORMAL:
        if (need_cali_) {
          trig_mod_ = TRIGMODE::CALI;
        } else if (trig_mod_ == TRIGMODE::RELAX) {
          target_trig_angle_ = trig_angle_;
        }

        if (!need_cali_) {
          trig_mod_ = fire_mode_ == FireMode::SINGLE ? TRIGMODE::SINGLE
                                                     : TRIGMODE::CONTINUE;
        }
        break;
      default:
        break;
    }

    /*不同拨弹模式*/
    switch (trig_mod_) {
      case TRIGMODE::RELAX:
        motor_trig_->Relax();
        target_trig_angle_ = trig_angle_;

        break;
      case TRIGMODE::SAFE: {
        // target_trig_angle_ = trig_angle_;

      } break;
      case TRIGMODE::CALI: {
        // 校准模式：等待校准完成
        target_trig_angle_ += 3.0 * dt_;
        if (((fabs(param_fric_0_.velocity) <
              (param_.fric_setpoint_speed - launcher::param::DELTA_RPM)) and
             (fabs(param_fric_1_.velocity) <
              (param_.fric_setpoint_speed - launcher::param::DELTA_RPM)))) {
          target_trig_angle_ += 0.04;
          need_cali_ = false;
        }

      } break;
      case TRIGMODE::SINGLE: {
        if (last_trig_mod_ == TRIGMODE::SAFE ||
            last_trig_mod_ == TRIGMODE::RELAX) {
          if ((target_trig_angle_ - trig_angle_) >
              launcher::param::TRIGSTEP * 0.3f) {
            target_trig_angle_ -= launcher::param::TRIGSTEP;
          } else {
            target_trig_angle_ += launcher::param::TRIGSTEP;
          }
        }

      } break;

      case TRIGMODE::CONTINUE: {
        float since_last = (now_ - last_trig_time_).ToSecondf();

        float trig_speed = 1.0f / trig_freq_;
        if (since_last > trig_speed) {
          if ((target_trig_angle_ - trig_angle_) >
              launcher::param::TRIGSTEP * 0.3f) {
            target_trig_angle_ -= launcher::param::TRIGSTEP;
          } else {
            target_trig_angle_ += launcher::param::TRIGSTEP;
          }
          last_trig_time_ = now_;
        }

      } break;
      default:
        break;
    }
    last_trig_mod_ = trig_mod_;
    /*判断是否成功发射*/
    if ((trig_angle_ - cal_trig_angle_) > launcher::param::TRIGSTEP - 0.1) {
      heat_limit_.launched_num++;
      cal_trig_angle_ = trig_angle_;
      number_++;
    }

    last_fire_notify_ = launcher_cmd_.isfire;
  }

  void SetFric() {
    switch (launcher_event_) {
      case LauncherEvent::SET_FRICMODE_RELAX: {
        target_rpm_ = 0;
        motor_fric_0_->Relax();
        motor_fric_1_->Relax();
      } break;
      case LauncherEvent::SET_FRICMODE_SAFE: {
        target_rpm_ = 0;
      } break;
      case LauncherEvent::SET_FRICMODE_READY: {
        if (last_bulllet_speed_ != referee_data_.bullet_speed) {
          if (referee_data_.bullet_speed > 22.5f) {
            target_rpm_ -= 80;
          }
          last_bulllet_speed_ = referee_data_.bullet_speed;
        }

      } break;
      default:
        break;
    }
  }

  void Control() {
    float out_trig = 0.0f;
    float out_fric_0 = 0.0f;
    float out_fric_1 = 0.0f;

    if (launcher_event_ == LauncherEvent::SET_FRICMODE_RELAX) {
      return;
    }
    if (trig_mod_ == TRIGMODE::RELAX) {
      out_trig = 0.0f;
    } else {
      TrigControl(out_trig, target_trig_angle_, dt_);
    }
    FricControl(out_fric_0, out_fric_1, target_rpm_, dt_);
    auto cmd_trig = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                    .reduction_ratio = 36.0f,
                                    .velocity = out_trig};
    auto cmd_fric_0 = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                      .reduction_ratio = 1.0f,
                                      .velocity = out_fric_0};
    auto cmd_fric_1 = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                      .reduction_ratio = 1.0f,
                                      .velocity = out_fric_1};
    auto motor_control = [&](Motor* motor, const Motor::Feedback& fb,
                             const Motor::MotorCmd& cmd) {
      if (fb.state == 0) {
        motor->Enable();
      } else if (fb.state != 0 and fb.state != 1) {
        motor->ClearError();
      } else {
        motor->Control(cmd);
      }
    };

    motor_control(motor_trig_, param_trig_, cmd_trig);
    motor_control(motor_fric_0_, param_fric_0_, cmd_fric_0);
    motor_control(motor_fric_1_, param_fric_1_, cmd_fric_1);
  }

  void SetMode(uint32_t mode) {
    launcher_event_ = static_cast<LauncherEvent>(mode);
    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();
    if (launcher_event_ == LauncherEvent::SET_FRICMODE_READY) {
      target_rpm_ = param_.fric_setpoint_speed;
    }
  }

  void SetFireMode(FireMode mode) { fire_mode_ = mode; }

  void SetFireModeByEvent(uint32_t event_id) {
    switch (static_cast<FireModeEvent>(event_id)) {
      case FireModeEvent::SET_FIREMODE_SINGLE:
        fire_mode_ = FireMode::SINGLE;
        break;
      case FireModeEvent::SET_FIREMODE_CONTINUE:
        fire_mode_ = FireMode::CONTINUE;
        break;
      case FireModeEvent::SET_FIREMODE_BOOST:
        fire_mode_ = FireMode::BOOST;
        break;
      default:
        break;
    }
    trig_mod_ = TRIGMODE::SAFE;
    last_trig_mod_ = TRIGMODE::SAFE;
    target_trig_angle_ = trig_angle_;
    last_trig_time_ = now_;
  }

  void LostCtrl() {
    launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
    fire_mode_ = FireMode::CONTINUE;
    motor_trig_->Relax();
    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();
    target_trig_angle_ = trig_angle_;
    last_trig_angle_ = trig_angle_;
    cal_trig_angle_ = trig_angle_;
  }

 private:
  LauncherEvent launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
  LauncherState launcherstate_;
  LauncherState last_launcherstate_ = LauncherState::RELAX;
  float last_bulllet_speed_;
  TRIGMODE last_trig_mod_ = TRIGMODE::RELAX;
  TRIGMODE trig_mod_ = TRIGMODE::RELAX;
  FireMode fire_mode_ = FireMode::CONTINUE;

  bool need_cali_ = false;
  RefereeData referee_data_;
  HeatLimit heat_limit_{
      .single_heat = 0.0f,
      .launched_num = 0.0f,
      .current_heat = 0.0f,
      .heat_threshold = 0.0f,
      .allow_fire = true,
  };
  RMMotor* motor_fric_0_;
  RMMotor* motor_fric_1_;
  RMMotor* motor_trig_;
  // Referee::LauncherPack lp_;

  float trig_angle_ = 0.0f;
  float target_trig_angle_ = 0.0f;
  uint16_t number_ = 0;
  float last_trig_angle_ = 0.0f;
  float trig_freq_ = 0.0f;

  Motor::Feedback param_fric_0_;
  Motor::Feedback param_fric_1_;
  Motor::Feedback param_trig_;
  LibXR::PID<float> pid_trig_angle_;
  LibXR::PID<float> pid_trig_sp_;
  LibXR::PID<float> pid_fric_0_;
  LibXR::PID<float> pid_fric_1_;
  LauncherParam param_;
  float default_expect_trig_freq_;
  CMD::LauncherCMD launcher_cmd_;
  float dt_ = 0;
  LibXR::MicrosecondTimestamp now_;
  LibXR::MicrosecondTimestamp last_trig_time_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;
  LibXR::Topic shoot_number_ = LibXR::Topic::CreateTopic<float>("shoot_number");
  // bool press_continue_ = false;
  bool last_fire_notify_ = false;

  LibXR::Thread thread_;
  LibXR::Mutex mutex_;
  LibXR::Event launcher_event_handler_;
  CMD* cmd_;
  float cal_trig_angle_ = 0.0f;
  float target_rpm_ = 0;

  float GetOptimalBurstFreq() const {
    for (const auto& level : launcher::param::BURST_5S_TABLE) {
      if (static_cast<uint16_t>(referee_data_.heat_limit) == level.heat_limit &&
          static_cast<uint16_t>(referee_data_.heat_cooling) ==
              level.heat_cooling) {
        return level.burst_freq;
      }
    }
    return param_.expect_trig_freq_;
  }

  float GetModeTargetFreq() const {
    if (fire_mode_ == FireMode::CONTINUE) {
      return GetOptimalBurstFreq();
    }
    if (fire_mode_ == FireMode::BOOST) {
      return launcher::param::BOOST_FIRE_FREQ;
    }
    return default_expect_trig_freq_;
  }

  void TrigControl(float& out_trig, float target_trig_angle_, float dt_) {
    float plate_omega_ref = pid_trig_angle_.Calculate(
        target_trig_angle_, trig_angle_,
        param_trig_.omega / param_.trig_gear_ratio, dt_);
    out_trig = pid_trig_sp_.Calculate(
        plate_omega_ref, param_trig_.omega / param_.trig_gear_ratio, dt_);
  }

  void FricControl(float& out_fric_0, float& out_fric_1, float target_rpm,
                   float dt_) {
    /*缓停*/
    if (launcher_event_ == LauncherEvent::SET_FRICMODE_SAFE) {
      out_fric_0 =
          pid_fric_0_.Calculate(target_rpm, param_fric_0_.velocity, dt_) /
          40.0f;
      out_fric_1 =
          pid_fric_1_.Calculate(target_rpm, param_fric_1_.velocity, dt_) /
          40.0f;
    } else {
      out_fric_0 =
          pid_fric_0_.Calculate(target_rpm, param_fric_0_.velocity, dt_);
      out_fric_1 =
          pid_fric_1_.Calculate(target_rpm, param_fric_1_.velocity, dt_);
    }
  }
};
