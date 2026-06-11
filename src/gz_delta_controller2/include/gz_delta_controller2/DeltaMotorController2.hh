#ifndef GZ_DELTA_CONTROLLER2_DELTA_MOTOR_CONTROLLER2_HH_
#define GZ_DELTA_CONTROLLER2_DELTA_MOTOR_CONTROLLER2_HH_

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <sdf/Element.hh>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/EntityComponentManager.hh>

#include <gz/math/Vector3.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/double_v.pb.h>

namespace gz_delta_controller2
{

class DeltaMotorController2:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  DeltaMotorController2() = default;

  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

private:
  // =====================================================
  // Full trajectory point stored inside controller2
  // Each point:
  //   t   = time from trajectory start
  //   q   = desired joint position
  //   qd  = desired joint velocity
  //   qdd = desired joint acceleration
  // =====================================================
  struct JointTrajectoryPoint2
  {
    double t{0.0};

    std::array<double, 3> q{0.0, 0.0, 0.0};
    std::array<double, 3> qd{0.0, 0.0, 0.0};
    std::array<double, 3> qdd{0.0, 0.0, 0.0};
  };

  void OnTarget1(const gz::msgs::Double &_msg);
  void OnTarget2(const gz::msgs::Double &_msg);
  void OnTarget3(const gz::msgs::Double &_msg);

  // Old style: receive one target at a time
  void OnJointReference(const gz::msgs::Double_V &_msg);

  // New style: receive full trajectory once
  void OnJointTrajectory(const gz::msgs::Double_V &_msg);

  // Sample stored trajectory at time t
  bool SampleTrajectory(
    double t,
    std::array<double, 3> &q_ref,
    std::array<double, 3> &qd_ref,
    std::array<double, 3> &qdd_ref);

  double Clamp(double value, double min_value, double max_value) const;

  void ApplyTwistStabilizers(
    gz::sim::EntityComponentManager &_ecm);

private:
  gz::sim::Model model_{gz::sim::kNullEntity};

  std::array<std::string, 3> joint_names_{
    "joint_1",
    "joint_2",
    "joint_3"
  };

  std::array<gz::sim::Entity, 3> joint_entities_{
    gz::sim::kNullEntity,
    gz::sim::kNullEntity,
    gz::sim::kNullEntity
  };

  gz::sim::Entity base_entity_{gz::sim::kNullEntity};
  gz::sim::Entity endlink_entity_{gz::sim::kNullEntity};

  std::array<std::string, 6> twist_parent_names_{
    "1", "1", "2", "2", "3", "3"
  };
  std::array<std::string, 6> twist_child_names_{
    "11", "12", "21", "22", "31", "32"
  };
  std::array<gz::sim::Entity, 6> twist_parent_entities_{
    gz::sim::kNullEntity, gz::sim::kNullEntity,
    gz::sim::kNullEntity, gz::sim::kNullEntity,
    gz::sim::kNullEntity, gz::sim::kNullEntity
  };
  std::array<gz::sim::Entity, 6> twist_child_entities_{
    gz::sim::kNullEntity, gz::sim::kNullEntity,
    gz::sim::kNullEntity, gz::sim::kNullEntity,
    gz::sim::kNullEntity, gz::sim::kNullEntity
  };
  std::array<gz::math::Vector3d, 6> twist_reference_y_;
  std::array<gz::math::Vector3d, 6> twist_reference_z_;
  std::array<bool, 6> twist_reference_initialized_{
    false, false, false, false, false, false
  };
  std::array<gz::transport::Node::Publisher, 6> twist_angle_debug_pubs_;
  std::array<gz::transport::Node::Publisher, 6> twist_rate_debug_pubs_;

  std::array<gz::transport::Node::Publisher, 3> theta_feedback_pubs_;
  std::array<gz::transport::Node::Publisher, 3> xyz_feedback_pubs_;
  gz::transport::Node::Publisher state_feedback_pub_;

  std::array<gz::transport::Node::Publisher, 3> error_debug_pubs_;
  std::array<gz::transport::Node::Publisher, 3> omega_debug_pubs_;
  std::array<gz::transport::Node::Publisher, 3> torque_raw_debug_pubs_;
  std::array<gz::transport::Node::Publisher, 3> torque_cmd_debug_pubs_;
  std::array<gz::transport::Node::Publisher, 3> saturated_debug_pubs_;

  int feedback_counter_{0};
  int feedback_decimation_{20};

  // =====================================================
  // Current reference used by torque controller
  // These are updated either by:
  //   1. OnJointReference()
  //   2. SampleTrajectory() in PreUpdate()
  // =====================================================
  std::array<double, 3> targets_{0.0, 0.0, 0.0};
  std::array<double, 3> velocity_targets_{0.0, 0.0, 0.0};
  std::array<double, 3> acceleration_targets_{0.0, 0.0, 0.0};

  std::array<double, 3> integrals_{0.0, 0.0, 0.0};
  std::array<double, 3> last_torque_commands_{0.0, 0.0, 0.0};
  std::array<double, 3> last_saturated_{0.0, 0.0, 0.0};

  // =====================================================
  // Full trajectory storage
  // =====================================================
  std::vector<JointTrajectoryPoint2> trajectory_;

  // Used to ignore repeated copies of the same trajectory.
  int last_trajectory_id_{-1};

  bool trajectory_received_{false};
  bool trajectory_active_{false};

  double trajectory_sim_start_{0.0};
  double trajectory_duration_{0.0};
  double current_trajectory_time_{0.0};

  std::size_t last_segment_index_{0};

  // =====================================================
  // Saturation statistics
  // Count how many update steps each motor torque was clamped.
  // Useful to know whether tracking error is caused by torque limit.
  // =====================================================
  std::array<int, 3> saturation_count_{0, 0, 0};

  // =====================================================
  // PID + velocity / acceleration feedforward gains
  // =====================================================
  double kp_{80.0};
  double ki_{0.0};
  double kd_{5.0};

  // Compensates viscous damping and other velocity-proportional loads.
  double kv_{0.0};

  // Acceleration feedforward gain
  double ka_{0.0};

  double torque_limit_{20.0};
  double integral_limit_{0.25};
  double anti_windup_gain_{1.0};
  double twist_kp_{0.0};
  double twist_kd_{0.0};
  double twist_torque_limit_{0.0};

  bool configured_{false};
  bool targets_initialized_{false};

  // Khi đã nhận joint_ref hoặc full trajectory thì bỏ qua motor target cũ
  // để tránh topic q-only ghi đè q_dot_ref / q_ddot_ref.
  bool joint_ref_mode_{false};

  gz::transport::Node gz_node_;
  std::mutex mutex_;
};

}  // namespace gz_delta_controller2

#endif
