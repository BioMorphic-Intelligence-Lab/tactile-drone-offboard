#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


class BaseTFPublisher : public rclcpp::Node
{
public:
  BaseTFPublisher()
  : Node("base_tf_broadcaster")
  {
    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "base_pose", rclcpp::SensorDataQoS(),
      std::bind(&BaseTFPublisher::handle_base_pose, this, std::placeholders::_1));

    wall_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "wall_pose", rclcpp::SensorDataQoS(),
      std::bind(&BaseTFPublisher::handle_wall_pose, this, std::placeholders::_1));
  }

private:
  void handle_base_pose(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = msg->header.frame_id;
    t.child_frame_id = "base";

    t.transform.translation.x = msg->pose.position.x;
    t.transform.translation.y = msg->pose.position.y;
    t.transform.translation.z = msg->pose.position.z;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    t.transform.rotation.x = msg->pose.orientation.x;
    t.transform.rotation.y = msg->pose.orientation.y;
    t.transform.rotation.z = msg->pose.orientation.z;
    t.transform.rotation.w = msg->pose.orientation.w;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  void handle_wall_pose(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = msg->header.frame_id;
    t.child_frame_id = "wall";

    t.transform.translation.x = msg->pose.position.x;
    t.transform.translation.y = msg->pose.position.y;
    t.transform.translation.z = msg->pose.position.z;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    t.transform.rotation.x = msg->pose.orientation.x;
    t.transform.rotation.y = msg->pose.orientation.y;
    t.transform.rotation.z = msg->pose.orientation.z;
    t.transform.rotation.w = msg->pose.orientation.w;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr wall_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseTFPublisher>());
  rclcpp::shutdown();
  return 0;
}