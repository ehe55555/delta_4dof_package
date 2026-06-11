#include <memory>
#include <string>
#include <functional>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/double_v.pb.h>

class DeltaBridge2Node : public rclcpp::Node
{
public:
  DeltaBridge2Node()
  : Node("delta_bridge2_node")
  {
    this->joint_ref_pub_ =
      this->gz_node_.Advertise<gz::msgs::Double_V>(
        "/delta_robot/joint_ref_gz");

    this->joint_ref_sub_ =
      this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/kinematic2/joint_ref_ros",
        10,
        std::bind(
          &DeltaBridge2Node::OnJointRefRos,
          this,
          std::placeholders::_1));

    this->joint_traj_pub_ =
      this->gz_node_.Advertise<gz::msgs::Double_V>(
        "/delta_robot/joint_trajectory_gz");

    this->joint_traj_sub_ =
      this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/kinematic2/joint_trajectory_ros",
        10,
        std::bind(
          &DeltaBridge2Node::OnJointTrajectoryRos,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "delta_bridge2 started:\n"
      "  /kinematic2/joint_ref_ros -> /delta_robot/joint_ref_gz\n"
      "  /kinematic2/joint_trajectory_ros -> /delta_robot/joint_trajectory_gz");
  }

private:
  void OnJointRefRos(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 6)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Joint ref rejected: need at least [q1,q2,q3,qd1,qd2,qd3]");
      return;
    }

    gz::msgs::Double_V gz_msg;

    for (const auto &value : msg->data)
    {
      gz_msg.add_data(value);
    }

    const bool ok = this->joint_ref_pub_.Publish(gz_msg);

    if (!ok)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to publish /delta_robot/joint_ref_gz");
    }
  }

  void OnJointTrajectoryRos(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 12)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Joint trajectory rejected: data too short. size=%zu",
        msg->data.size());
      return;
    }

    const double trajectory_id = msg->data[0];
    const double n_waypoints = msg->data[1];

    const std::size_t expected_size =
      2 + static_cast<std::size_t>(n_waypoints) * 10;

    RCLCPP_INFO(
      this->get_logger(),
      "Bridge2 received full trajectory from ROS: size=%zu, id=%.0f, N=%.0f, expected=%zu",
      msg->data.size(),
      trajectory_id,
      n_waypoints,
      expected_size);

    if (n_waypoints < 2)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Joint trajectory rejected: N must be >= 2");
      return;
    }

    if (msg->data.size() < expected_size)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Joint trajectory rejected: expected size=%zu, got size=%zu",
        expected_size,
        msg->data.size());
      return;
    }

    gz::msgs::Double_V gz_msg;

    for (const auto &value : msg->data)
    {
      gz_msg.add_data(value);
    }

    // Publish repeated copies to reduce the chance of a missed Gazebo transport message.
    // Controller2 uses trajectory_id to ignore duplicates, so this will not restart
    // the same trajectory multiple times.
    bool any_ok = false;

    for (int i = 0; i < 5; ++i)
    {
      const bool ok = this->joint_traj_pub_.Publish(gz_msg);
      any_ok = any_ok || ok;

      RCLCPP_INFO(
        this->get_logger(),
        "Bridge2 published full trajectory to Gazebo [%d/5]: ok=%s, id=%.0f, N=%.0f, gz_size=%d",
        i + 1,
        ok ? "true" : "false",
        trajectory_id,
        n_waypoints,
        gz_msg.data_size());

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!any_ok)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to publish /delta_robot/joint_trajectory_gz");
    }
  }

private:
  gz::transport::Node gz_node_;

  gz::transport::Node::Publisher joint_ref_pub_;
  gz::transport::Node::Publisher joint_traj_pub_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    joint_ref_sub_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    joint_traj_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DeltaBridge2Node>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}