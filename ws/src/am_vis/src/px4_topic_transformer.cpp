#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_ros_com/frame_transforms.h"

using namespace std::chrono_literals;


class PX4TopicTransformer : public rclcpp::Node
{
  public:
    PX4TopicTransformer()
    : Node("PX4TopicTransformer")
    {
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("base_pose", 10);
    
        pose_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", 
		    rclcpp::SensorDataQoS(), std::bind(&PX4TopicTransformer::pose_callback, this, std::placeholders::_1));
         
    }

  private:
    void pose_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
      auto message = geometry_msgs::msg::PoseStamped();
      message.header.stamp = this->now(); 
      message.header.frame_id = "world";


      Eigen::Quaterniond orientation = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);
      Eigen::Vector3d position = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);

      orientation = px4_ros_com::frame_transforms::ned_to_enu_orientation(   
                               px4_ros_com::frame_transforms::baselink_to_aircraft_orientation(orientation));

      position = px4_ros_com::frame_transforms::ned_to_enu_local_frame(position);                                                                  
                                                                    
      message.pose.position.x = position.x();
      message.pose.position.y = position.y();
      message.pose.position.z = position.z();

      message.pose.orientation.x = orientation.x();
      message.pose.orientation.y = orientation.y();
      message.pose.orientation.z = orientation.z();
      message.pose.orientation.w = orientation.w();

      this->pose_publisher_->publish(message);

    }

    Eigen::Matrix3d _rot_x(double theta)
    {
      double sT = sin(theta), cT = cos(theta);
      Eigen::Matrix3d rot;
      rot << 1, 0, 0,
            0, cT, -sT,
            0, sT, cT;
      return rot;
    }

    Eigen::Matrix3d _rot_y(double theta)
    {

      double sT = sin(theta), cT = cos(theta);
      Eigen::Matrix3d rot;
      rot << cT, 0, sT,
            0, 1, 0,
            -sT, 0, cT;
      return rot;
    }
    Eigen::Matrix3d _rot_z(double theta)
    {
      double sT = sin(theta), cT = cos(theta);
      Eigen::Matrix3d rot;
      rot << cT, -sT, 0,
            sT, cT, 0,
            0, 0, 1;
      return rot;
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr pose_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4TopicTransformer>());
  rclcpp::shutdown();
  return 0;
}