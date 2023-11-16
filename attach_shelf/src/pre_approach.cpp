/*ROS2 galactic
-use of arguments: obstacle, degrees
-starts publishing a linear velocity to the /robot/cmd_vel 
-when the laser detects an obstacle, it stops 'x' m, stop 
*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <chrono>

using std::placeholders::_1;

class RobotRB1 : public rclcpp::Node
{
public:
  RobotRB1(std::string topic_pub, std::string topic_laser, std::string topic_odometry)
      : Node("robot_rb1"), MAX_LINEAR_SPEED_(0.5), MAX_ANGULAR_SPEED_(0.2), TIMER_PERIOD_MS_(100)
  {
    //publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_pub, 10);
    //subscriber laser
    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        topic_laser, 10, std::bind(&RobotRB1::laser_callback, this, _1));
    //subscriber odometry
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        topic_odometry, 10, std::bind(&RobotRB1::odometry_callback, this, _1));
    //parameters
    param_obstacle_desc.description = "Distance to obstacle";
    this->declare_parameter<float>("obstacle", 0.0, param_obstacle_desc);
    param_degrees_desc.description = "Degrees to turn";
    this->declare_parameter<float>("degrees", 0.0, param_degrees_desc);
    //timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)TIMER_PERIOD_MS_),
        std::bind(&RobotRB1::timer_callback, this));
  }
private:
    //publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist pub_msg_;
    const float MAX_LINEAR_SPEED_;
    const float MAX_ANGULAR_SPEED_;
    //subscriber laser
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    float front_distance_;
    //subscriber odometry
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    float orientation_;
    //arguments
    float obstacle_;
    auto param_obstacle_desc= rclcpp::ParameterDescriptor();
    float degrees_;
    auto param_degrees_desc= rclcpp::ParameterDescriptor();
    //timer
    rclcpp::TimerBase::SharedPtr timer_;
    const float TIMER_PERIOD_MS_;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    front_distance_ = msg->ranges[540];
    //RCLCPP_INFO(this->get_logger(), "Front distance: '%.2f'", front_distance_);
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    orientation_ = yaw*180/M_PI;
    //RCLCPP_INFO(this->get_logger(), "Angle degree: '%.2f'", orientation_);
  }
  
  void timer_callback()
  {
    this->get_parameter("obstacle", obstacle_);
    this->get_parameter("degrees", degrees_);
    RCLCPP_INFO(this->get_logger(), "Obstacle: '%.2f' Degrees: '%.2f'", obstacle_, degrees_);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<RobotRB1>("robot/cmd_vel","scan","odom"));
  rclcpp::shutdown();
  return 0;
}