#ifndef COMPOSITION__ATTACH_SERVER_HPP_
#define COMPOSITION__ATTACH_SERVER_HPP_
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
#include "attach_shelf/srv/go_to_loading.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using GoToLanding = attach_shelf::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;

namespace my_components
{
class AttachServer : public rclcpp::Node
{
public:
    COMPOSITION_PUBLIC
    explicit AttachServer(const rclcpp::NodeOptions & options);
private:
    //subscriber laser
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    float front_distance_;
    std::string topic_laser_= "/scan";
    //service
    rclcpp::Service<GoToLanding>::SharedPtr service_;
    bool execute_service_;
    bool move_extra_distance_;
    //variables
    float cart_magnitude_=0.0;
    float cart_x_=0.0;
    float cart_y_=0.0;
    float cart_yaw_=0.0;
    tf2::Quaternion cart_quat_;
    int number_of_legs_=0;
    float extra_distance_=0.99;
    int times_to_move_extra_distance_;
    // tf2 listener variables
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string origin_frame_ = "odom";
    std::string destiny_frame_ = "robot_front_laser_base_link";
    //broadcast transform
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    geometry_msgs::msg::TransformStamped transformStamped_;
    std::string object_frame_ = "cart_frame";
    //publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist pub_msg_;
    const float MAX_LINEAR_SPEED_=0.5;
    const float MAX_ANGULAR_SPEED_=0.4;
    const float ZERO_LINEAR_SPEED_ = 0.0;
    const float ZERO_ANGULAR_SPEED_ = 0.0;
    std::string topic_pub_= "/diffbot_base_controller/cmd_vel_unstamped";
    //timer
    rclcpp::TimerBase::SharedPtr timer_;
    const float TIMER_PERIOD_MS_;
    //elevator up
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_publisher_;
    std::string topic_elevator_up_ = "/elevator_up";
protected:
    void timer_callback();
    void laser_intensities_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void service_callback(
        const std::shared_ptr<GoToLanding::Request> request,
        std::shared_ptr<GoToLanding::Response> response);
    void robot_move(float linear_speed, float angular_speed);
};
}  // namespace my_components

#endif  // COMPOSITION__ATTACH_SERVER_HPP_