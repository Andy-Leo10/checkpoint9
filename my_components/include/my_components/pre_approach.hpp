#ifndef COMPOSITION__PRE_APPROACH_HPP_
#define COMPOSITION__PRE_APPROACH_HPP_
#include "my_components/visibility_control.h"

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

namespace my_components
{
class PreApproach : public rclcpp::Node
{
public:
    COMPOSITION_PUBLIC
    explicit PreApproach(const rclcpp::NodeOptions & options);
private:
    //publisher 
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist pub_msg_;
    const float MAX_LINEAR_SPEED_;
    const float MAX_ANGULAR_SPEED_;
    const float ZERO_LINEAR_SPEED_ = 0.0;
    const float ZERO_ANGULAR_SPEED_ = 0.0;
    std::string topic_pub_= "/diffbot_base_controller/cmd_vel_unstamped";
    //subscriber laser
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    std::string topic_laser_= "/scan";
    float front_distance_;
    //subscriber odometry
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    float orientation_;
    std::string topic_odometry_= "/diffbot_base_controller/odom";
    //arguments
    float obstacle_=0.4;
    int degrees_=-90;
    //timer
    rclcpp::TimerBase::SharedPtr timer_;
    const float TIMER_PERIOD_MS_;
    //steps
    bool step1_completed_ = false;
    bool step2_completed_ = false;
    bool step3_completed_ = false;
protected:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timer_callback();
    void robot_move(float linear_speed, float angular_speed);
    template <typename T>
    T saturate(T var, T min, T max);
    bool controller_kp(float control_var, float desired_var, float kp, float tolerance);
};
}  // namespace my_components

#endif  // COMPOSITION__PRE_APPROACH_HPP_