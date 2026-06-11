#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/double_v.pb.h>

class FeedbackBridgeNode : public rclcpp::Node
{
public:
  FeedbackBridgeNode()
  : Node("feedback_bridge_node")
  {
    this->feedback_xyz_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/delta_robot/feedback_xyz",
        10);

    this->feedback_theta_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/delta_robot/feedback_theta",
        10);

    this->debug_omega_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/delta_robot/debug_omega",
        10);

    this->state_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/delta_robot/state",
        10);

    // ==========================
    // Gazebo -> local callbacks
    // ==========================
    const bool sub_theta1 = this->gz_node_.Subscribe(
      "/delta_robot/feedback/theta1_gz",
      &FeedbackBridgeNode::OnTheta1,
      this);

    const bool sub_theta2 = this->gz_node_.Subscribe(
      "/delta_robot/feedback/theta2_gz",
      &FeedbackBridgeNode::OnTheta2,
      this);

    const bool sub_theta3 = this->gz_node_.Subscribe(
      "/delta_robot/feedback/theta3_gz",
      &FeedbackBridgeNode::OnTheta3,
      this);

    const bool sub_x = this->gz_node_.Subscribe(
      "/delta_robot/feedback/x_gz",
      &FeedbackBridgeNode::OnX,
      this);

    const bool sub_y = this->gz_node_.Subscribe(
      "/delta_robot/feedback/y_gz",
      &FeedbackBridgeNode::OnY,
      this);

    const bool sub_z = this->gz_node_.Subscribe(
      "/delta_robot/feedback/z_gz",
      &FeedbackBridgeNode::OnZ,
      this);

    const bool sub_omega1 = this->gz_node_.Subscribe(
      "/delta_robot/debug/omega1_gz",
      &FeedbackBridgeNode::OnOmega1,
      this);

    const bool sub_omega2 = this->gz_node_.Subscribe(
      "/delta_robot/debug/omega2_gz",
      &FeedbackBridgeNode::OnOmega2,
      this);

    const bool sub_omega3 = this->gz_node_.Subscribe(
      "/delta_robot/debug/omega3_gz",
      &FeedbackBridgeNode::OnOmega3,
      this);

    const bool sub_state = this->gz_node_.Subscribe(
      "/delta_robot/state_gz",
      &FeedbackBridgeNode::OnState,
      this);

    RCLCPP_INFO(
      this->get_logger(),
      "feedback_bridge_node subscribe result:\n"
      "  theta1=%s theta2=%s theta3=%s\n"
      "  x=%s y=%s z=%s\n"
      "  omega1=%s omega2=%s omega3=%s state=%s",
      sub_theta1 ? "true" : "false",
      sub_theta2 ? "true" : "false",
      sub_theta3 ? "true" : "false",
      sub_x ? "true" : "false",
      sub_y ? "true" : "false",
      sub_z ? "true" : "false",
      sub_omega1 ? "true" : "false",
      sub_omega2 ? "true" : "false",
      sub_omega3 ? "true" : "false",
      sub_state ? "true" : "false");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&FeedbackBridgeNode::PublishRosFeedback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "feedback_bridge_node started:\n"
      "  Gazebo feedback/*_gz -> ROS /delta_robot/feedback_xyz\n"
      "  Gazebo feedback/theta*_gz -> ROS /delta_robot/feedback_theta\n"
      "  Gazebo debug/omega*_gz -> ROS /delta_robot/debug_omega\n"
      "  Gazebo /delta_robot/state_gz -> ROS /delta_robot/state");
  }

private:
  bool IsFiniteArray(const std::array<double, 3> &values) const
  {
    return std::isfinite(values[0]) &&
           std::isfinite(values[1]) &&
           std::isfinite(values[2]);
  }

  void SetValue(
    std::array<double, 3> &values,
    std::array<bool, 3> &ready,
    std::size_t index,
    double value)
  {
    if (index >= 3)
    {
      return;
    }

    if (!std::isfinite(value))
    {
      return;
    }

    std::lock_guard<std::mutex> lock(this->mutex_);

    values[index] = value;
    ready[index] = true;
  }

  bool AllReady(const std::array<bool, 3> &ready) const
  {
    return ready[0] && ready[1] && ready[2];
  }

  void PublishArray(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr &pub,
    const std::array<double, 3> &values)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {values[0], values[1], values[2]};
    pub->publish(msg);
  }

  void PublishRosFeedback()
  {
    std::array<double, 3> xyz;
    std::array<double, 3> theta;
    std::array<double, 3> omega;

    bool xyz_ready = false;
    bool theta_ready = false;
    bool omega_ready = false;

    {
      std::lock_guard<std::mutex> lock(this->mutex_);

      xyz = this->xyz_;
      theta = this->theta_;
      omega = this->omega_;

      xyz_ready = this->AllReady(this->xyz_ready_) &&
                  this->IsFiniteArray(this->xyz_);

      theta_ready = this->AllReady(this->theta_ready_) &&
                    this->IsFiniteArray(this->theta_);

      omega_ready = this->AllReady(this->omega_ready_) &&
                    this->IsFiniteArray(this->omega_);
    }

    if (xyz_ready)
    {
      this->PublishArray(this->feedback_xyz_pub_, xyz);
    }

    if (theta_ready)
    {
      this->PublishArray(this->feedback_theta_pub_, theta);
    }

    if (omega_ready)
    {
      this->PublishArray(this->debug_omega_pub_, omega);
    }

    if ((xyz_ready || theta_ready || omega_ready) && !this->printed_first_feedback_)
    {
      this->printed_first_feedback_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "First feedback bridged: "
        "xyz_ready=%s, theta_ready=%s, omega_ready=%s",
        xyz_ready ? "true" : "false",
        theta_ready ? "true" : "false",
        omega_ready ? "true" : "false");
    }
  }

  // ==========================
  // Gazebo callbacks: theta
  // ==========================
  void OnTheta1(const gz::msgs::Double &_msg)
  {
    this->SetValue(this->theta_, this->theta_ready_, 0, _msg.data());
  }

  void OnTheta2(const gz::msgs::Double &_msg)
  {
    this->SetValue(this->theta_, this->theta_ready_, 1, _msg.data());
  }

  void OnTheta3(const gz::msgs::Double &_msg)
  {
    this->SetValue(this->theta_, this->theta_ready_, 2, _msg.data());
  }

  // ==========================
  // Gazebo callbacks: XYZ
  // ==========================
  void OnX(const gz::msgs::Double &_msg)
  {
    this->SetValue(this->xyz_, this->xyz_ready_, 0, _msg.data());
  }

  void OnY(const gz::msgs::Double &_msg)
  {
    this->SetValue(this->xyz_, this->xyz_ready_, 1, _msg.data());
  }

  void OnZ(const gz::msgs::Double &_msg)
  {
    this->SetValue(this->xyz_, this->xyz_ready_, 2, _msg.data());
  }

  // ==========================
  // Gazebo callbacks: omega
  // ==========================
  void OnOmega1(const gz::msgs::Double &_msg)
  {
    this->SetValue(this->omega_, this->omega_ready_, 0, _msg.data());
  }

  void OnOmega2(const gz::msgs::Double &_msg)
  {
    this->SetValue(this->omega_, this->omega_ready_, 1, _msg.data());
  }

  void OnOmega3(const gz::msgs::Double &_msg)
  {
    this->SetValue(this->omega_, this->omega_ready_, 2, _msg.data());
  }

  void OnState(const gz::msgs::Double_V &_msg)
  {
    if (_msg.data_size() < 22)
      return;

    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(_msg.data_size());
    for (int i = 0; i < _msg.data_size(); ++i)
      msg.data.push_back(_msg.data(i));
    this->state_pub_->publish(msg);
  }

private:
  gz::transport::Node gz_node_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    feedback_xyz_pub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    feedback_theta_pub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    debug_omega_pub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    state_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;

  std::array<double, 3> xyz_{0.0, 0.0, 0.0};
  std::array<double, 3> theta_{0.0, 0.0, 0.0};
  std::array<double, 3> omega_{0.0, 0.0, 0.0};

  std::array<bool, 3> xyz_ready_{false, false, false};
  std::array<bool, 3> theta_ready_{false, false, false};
  std::array<bool, 3> omega_ready_{false, false, false};

  bool printed_first_feedback_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<FeedbackBridgeNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
