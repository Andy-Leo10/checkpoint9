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

using std::placeholders::_1;

class RobotRB1 : public rclcpp::Node
{
public:
  RobotRB1(int argc, char *argv[], std::string topic_pub, std::string topic_laser, std::string topic_odometry)
      : Node("robot_rb1"), MAX_LINEAR_SPEED_(0.5), MAX_ANGULAR_SPEED_(0.2)
  {
    //publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_pub, 10);
    //subscriber laser
    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        topic_laser, 10, std::bind(&RobotRB1::laser_callback, this, _1));
    //subscriber odometry
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        topic_odometry, 10, std::bind(&RobotRB1::odometry_callback, this, _1));
    //timer
    timer_ = this->create_wall_timer(
        500ms, std::bind(&RobotRB1::timer_callback, this));
    //arguments
    obstacle_ = std::stof(argv[2]);
    degrees_ = std::stof(argv[4]);
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
    //timer
    rclcpp::TimerBase::SharedPtr timer_;
    //arguments
    float obstacle_;
    float degrees_;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    front_distance_ = msg->ranges[360];
    RCLCPP_INFO(this->get_logger(), "Front distance: '%.2f'", front_distance_);
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
    orientation_ = yaw;
    RCLCPP_INFO(this->get_logger(), "Angle degree: '%.2f'", front_distance_*180/M_PI);
  }
  
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<RobotRB1>(argc,argv,"robot/cmd_vel","scan","odom"));
  rclcpp::shutdown();
  return 0;
}