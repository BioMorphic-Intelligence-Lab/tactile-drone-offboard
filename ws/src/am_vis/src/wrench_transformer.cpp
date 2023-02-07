
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class WrenchTransformer : public rclcpp::Node
{
public:
  WrenchTransformer()
  : Node("wrench_transformer")
  {
    std::string wrench_topic_ = this->declare_parameter<std::string>("wrench_topic", "force");
    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "ee");

    std::chrono::duration<int> buffer_timeout(1);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    wrench_sub_.subscribe(this, wrench_topic_);
    tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::WrenchStamped>>(
      wrench_sub_, *tf2_buffer_, target_frame_, 100, this->get_node_logging_interface(),
      this->get_node_clock_interface(), buffer_timeout);
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    tf2_filter_->registerCallback(&WrenchTransformer::msgCallback, this);


    publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(wrench_topic_ + "/" + target_frame_, 10);
  }

private:
  void msgCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench_ptr)
  {
    geometry_msgs::msg::WrenchStamped wrench_out;
    try {
      tf2_buffer_->transform(*wrench_ptr, wrench_out, target_frame_);
      this->publisher_->publish(wrench_out);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        // Print exception which was caught
        this->get_logger(), "Failure %s\n", ex.what());
    }
  }

  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<geometry_msgs::msg::WrenchStamped> wrench_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::WrenchStamped>> tf2_filter_;

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchTransformer>());
  rclcpp::shutdown();
  return 0;
}