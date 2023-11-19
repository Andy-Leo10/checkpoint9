/*ROS2 galactic
-use of arguments: obstacle, degrees, final_approach
1 starts publishing a linear velocity to the /robot/cmd_vel 
when the laser detects an obstacle, it stops 'x' m, stop 
2 turns 'y' degrees, stop
3 call a service(/approach_shelf) for final approach
*/

//general libraries
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <chrono>
//custom service
#include "attach_shelf/srv/go_to_loading.hpp"

using std::placeholders::_1;

class RobotRB1 : public rclcpp::Node
{
public:
  RobotRB1(std::string topic_pub, std::string topic_laser, std::string topic_odometry)
      : Node("robot_rb1"), MAX_LINEAR_SPEED_(0.5), MAX_ANGULAR_SPEED_(0.4), TIMER_PERIOD_MS_(100)
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
    this->declare_parameter<float>("obstacle", 0.0);
    this->declare_parameter<int>("degrees", 0);
    this->declare_parameter<bool>("final_approach", false);
    //timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)TIMER_PERIOD_MS_),
        std::bind(&RobotRB1::timer_callback, this));
    //service client
    srv_client_ = this->create_client<attach_shelf::srv::GoToLoading>("approach_shelf");
  }
private:
    //publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist pub_msg_;
    const float MAX_LINEAR_SPEED_;
    const float MAX_ANGULAR_SPEED_;
    const float ZERO_LINEAR_SPEED_ = 0.0;
    const float ZERO_ANGULAR_SPEED_ = 0.0;
    //subscriber laser
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    float front_distance_;
    //subscriber odometry
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    float orientation_;
    //arguments
    float obstacle_;
    int degrees_;
    bool final_approach_;
    //timer
    rclcpp::TimerBase::SharedPtr timer_;
    const float TIMER_PERIOD_MS_;
    //service client
    rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr srv_client_;
    //steps
    bool step1_completed_ = false;
    bool step2_completed_ = false;
    bool step3_completed_ = false;

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
    this->get_parameter("final_approach", final_approach_);
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
    //step 3: go until obstacle by calling a service (if final_approach_ is true)
    if (!step3_completed_ && step2_completed_)
    {
      if (final_approach_) //call service
      {
        auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
        request->final_approach = final_approach_;
        auto result = srv_client_->async_send_request(request);
        step3_completed_ = result.get()->complete;
      }
    }
  }

  void robot_move(float linear_speed, float angular_speed)
  {
    pub_msg_.linear.x = linear_speed;
    pub_msg_.angular.z = angular_speed;
    publisher_->publish(pub_msg_);
  }

  template <typename T>
  T saturate(T var, T min, T max)
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

  bool controller_kp(float control_var, float desired_var, float kp, float tolerance)
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

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<RobotRB1>("robot/cmd_vel","scan","odom"));
  rclcpp::shutdown();
  return 0;
}