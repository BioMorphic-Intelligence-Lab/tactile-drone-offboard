#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;


class BaseFakePosePublisher : public rclcpp::Node
{
  public:
    BaseFakePosePublisher()
    : Node("base_fake_pose_publisher")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("base_pose", 10);
      timer_ = this->create_wall_timer(
      10ms, std::bind(&BaseFakePosePublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::PoseStamped();
      message.header.stamp = this->now();
      message.header.frame_id = "world";

      message.pose.position.x = 0.0;
      message.pose.position.y = 0.0;
      message.pose.position.z = 1.0;

      message.pose.orientation.x = 0.0;
      message.pose.orientation.y = 0.0;
      message.pose.orientation.z = 0.0;
      message.pose.orientation.w = 1.0;

      this->publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseFakePosePublisher>());
  rclcpp::shutdown();
  return 0;
}