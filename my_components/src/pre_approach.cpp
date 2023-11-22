/*ROS2 humble
-use of arguments: obstacle, degrees
-starts publishing a linear velocity to the /robot/cmd_vel 
-when the laser detects an obstacle, it stops 'x' m, stop 
*/

#include "my_components/pre_approach.hpp"

namespace my_components
{
PreApproach::PreApproach(const rclcpp::NodeOptions & options)
      : Node("robot_rb1", options), 
      MAX_LINEAR_SPEED_(0.5), MAX_ANGULAR_SPEED_(0.4), TIMER_PERIOD_MS_(100)
{
    //publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_pub_, 10);
    //subscriber laser
    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        topic_laser_, 10, std::bind(&PreApproach::laser_callback, this, _1));
    //subscriber odometry
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        topic_odometry_, 10, std::bind(&PreApproach::odometry_callback, this, _1));
    //parameters
    this->declare_parameter<float>("obstacle", 0.0);
    this->declare_parameter<int>("degrees", 0);
    //timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)TIMER_PERIOD_MS_),
        std::bind(&PreApproach::timer_callback, this));
}

void PreApproach::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    front_distance_ = msg->ranges[540];
    //RCLCPP_INFO(this->get_logger(), "Front distance: '%.2f'", front_distance_);
}

void PreApproach::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

void PreApproach::timer_callback()
{
    this->get_parameter("obstacle", obstacle_);
    this->get_parameter("degrees", degrees_);
    //step 1: go until obstacle
    if (!step1_completed_)
    {
        if (front_distance_ > obstacle_)
        {
        robot_move(MAX_LINEAR_SPEED_, ZERO_ANGULAR_SPEED_);
        }
        else
        {
        robot_move(ZERO_LINEAR_SPEED_,ZERO_ANGULAR_SPEED_);
        step1_completed_ = true;
        }
    }
    //step 2: turn 'degrees' degrees
    if (!step2_completed_ && step1_completed_)
    {
        /*orientation control
        -control variable : orientation_
        -desired variable : values close to degrees_
        */
        float control_var=orientation_*M_PI/180;
        float desired_var=degrees_*M_PI/180;
        float kp = 1.0;
        float tolerance = 1.0*M_PI/180;
        step2_completed_=controller_kp(control_var, desired_var, kp, tolerance);
    }
}

void PreApproach::robot_move(float linear_speed, float angular_speed)
{
    pub_msg_.linear.x = linear_speed;
    pub_msg_.angular.z = angular_speed;
    publisher_->publish(pub_msg_);
}

template <typename T>
T PreApproach::saturate(T var, T min, T max)
{
    if (var > max)
    {
        return max;
    }
    else if (var < min)
    {
        return min;
    }
    else
    {
        return var;
    }
}

bool PreApproach::controller_kp(float control_var, float desired_var, float kp, float tolerance)
{
    float error = desired_var - control_var;
    float angular_speed = kp * error;
    angular_speed = saturate(angular_speed, -MAX_ANGULAR_SPEED_, MAX_ANGULAR_SPEED_);
    robot_move(ZERO_LINEAR_SPEED_, angular_speed);
    if (fabs(error) < tolerance)
    {
        RCLCPP_INFO(this->get_logger(), "desired value achieved!");
        robot_move(ZERO_LINEAR_SPEED_, ZERO_ANGULAR_SPEED_);
        return true;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Error: '%.2f'", error*180/M_PI);
        return false;
    }
}
} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)