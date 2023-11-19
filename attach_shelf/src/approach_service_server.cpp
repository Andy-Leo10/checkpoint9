/*ROS2 galactic
-service name: approach_shelf
-detect the legs of the shelf using the laser intensity values
    1 shelf leg or none, it will return a False message
    both legs, the service will publish a transform named cart_frame to the center point between both legs
-the robot will use the TF to move towards the shelf
-then it will move forward 30 cm more (to end up right underneath the shelf)
-publish to the topics /elevator_up and /elevator_down
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
//to pusblish the transform
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
//to listen to the transform time stamp
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using GoToLanding = attach_shelf::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;

class MyServiceServer : public rclcpp::Node
{
public:
    MyServiceServer(std::string topic_pub, std::string topic_laser, std::string topic_odometry)
         : Node("my_service_server"), 
    {
        //subscriber laser
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            topic_laser, 10, std::bind(&MyServiceServer::laser_intensities_callback, this, _1));
        //service
        service_ = this->create_service<GoToLanding>("approach_shelf", 
            std::bind(&MyServiceServer::service_callback, this, std::placeholders::_1, std::placeholders::_2));
        execute_service_ = false;
        // initialize the tf2 listener for time stamp
        auto clock = this->get_clock();
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        origin_frame_ = "robot_odom";
        destiny_frame_ = "robot_front_laser_base_link";
        //broadcast transform
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        object_frame_ = "cart_frame";
        //publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_pub, 10);
        //timer
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)TIMER_PERIOD_MS_),
        std::bind(&RobotRB1::timer_callback, this));
    }

private:
    //subscriber laser
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    float front_distance_;
    //service
    rclcpp::Service<GoToLanding>::SharedPtr service_;
    bool execute_service_;
    //variables
    float cart_magnitude_=0.0;
    float cart_x_=0.0;
    float cart_y_=0.0;
    float cart_yaw_=0.0;
    tf2::Quaternion cart_quat_;
    int number_of_legs_=0;
    // tf2 listener variables
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string origin_frame_, destiny_frame_;
    //broadcast transform
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    geometry_msgs::msg::TransformStamped transformStamped_;
    std::string object_frame_;
    //publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist pub_msg_;
    const float MAX_LINEAR_SPEED_=0.5;
    const float MAX_ANGULAR_SPEED_=0.4;
    const float ZERO_LINEAR_SPEED_ = 0.0;
    const float ZERO_ANGULAR_SPEED_ = 0.0;
    //timer
    rclcpp::TimerBase::SharedPtr timer_;
    const float TIMER_PERIOD_MS_=100;

    void laser_intensities_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        //detect the intensities greater than 7000 and store the indexes in a vector
        std::vector<int> intensities_indexes;
        float intensity_threshold = 7000;
        for (std::vector<float>::size_type i = 0; i < msg->ranges.size(); i++)
        {
            if (msg->intensities[i] > intensity_threshold)
            {
                intensities_indexes.push_back(i);
            }
        }
        //group the indexes if they are at maximun 5 indexes away from each other
        std::vector<std::vector<int>> legs_indexes;
        std::vector<int> leg_indexes;
        int separation_threshold = 5;
        for (std::vector<float>::size_type i = 0; i < intensities_indexes.size(); i++)
        {
            if (i == 0)
            {
                leg_indexes.push_back(intensities_indexes[i]);
            }
            else
            {
                if (intensities_indexes[i] - intensities_indexes[i - 1] <= separation_threshold)
                {
                    leg_indexes.push_back(intensities_indexes[i]);
                }
                else
                {
                    legs_indexes.push_back(leg_indexes);
                    leg_indexes.clear();
                    leg_indexes.push_back(intensities_indexes[i]);
                }
            }
        }
        if (!leg_indexes.empty())
        {
            legs_indexes.push_back(leg_indexes);
        }
        //print how many legs were detected: std::vector::size_type
        //RCLCPP_INFO(this->get_logger(), "Legs detected: %zu", legs_indexes.size());
        //for (std::vector<float>::size_type i = 0; i < legs_indexes.size(); i++)
        //{
        //    RCLCPP_INFO(this->get_logger(), "Leg %zu: ", i);
        //    for (std::vector<float>::size_type j = 0; j < legs_indexes[i].size(); j++)
        //    {
        //        RCLCPP_INFO(this->get_logger(), "%d ", legs_indexes[i][j]);
        //    }
        //}
        number_of_legs_ = legs_indexes.size();
        if (number_of_legs_ == 2)
        {
            // Choose 1 point from each leg
            int leg_index_1 = legs_indexes[0][0];
            int leg_index_2 = legs_indexes[1][0];
            // The rays are from 0 to 1080 clockwise [225 to -45 degrees]
            cart_magnitude_ = (msg->ranges[leg_index_1] + msg->ranges[leg_index_2]) / 2.0;
            // Calculate the average angle between the two legs
            cart_yaw_ = -(msg->angle_min + (leg_index_1 + leg_index_2) / 2.0 * msg->angle_increment);
            // Calculate the cartesian coordinates 
            cart_x_ = cart_magnitude_ * cos(cart_yaw_);
            cart_y_ = cart_magnitude_ * sin(cart_yaw_);
            cart_quat_.setRPY(0.0, 0.0, cart_yaw_);
            //------------for take the value of time stamp--------------
            geometry_msgs::msg::TransformStamped transformStamped1;
            //geometry_msgs::msg::Vector3 translation;
            //geometry_msgs::msg::Quaternion rotation;
            try
            {
            transformStamped1 = tf_buffer_->lookupTransform(
                origin_frame_, destiny_frame_, tf2::TimePointZero);
            //translation = transformStamped1.transform.translation;
            //rotation = transformStamped1.transform.rotation;
            }
            catch (tf2::TransformException &ex)
            {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                        origin_frame_.c_str(), destiny_frame_.c_str(), ex.what());
            }
            //------------for take the value of time stamp--------------
            transformStamped_.header.stamp = transformStamped1.header.stamp;
            transformStamped_.header.frame_id = destiny_frame_;
            transformStamped_.child_frame_id = object_frame_;
            transformStamped_.transform.translation.x = cart_x_;
            transformStamped_.transform.translation.y = cart_y_;
            transformStamped_.transform.translation.z = 0.0;
            transformStamped_.transform.rotation.x = cart_quat_.x();
            transformStamped_.transform.rotation.y = cart_quat_.y();
            transformStamped_.transform.rotation.z = cart_quat_.z();
            transformStamped_.transform.rotation.w = cart_quat_.w();
            broadcaster_->sendTransform(transformStamped_);

            //depending on the service request, move the robot to the cart_frame
            if(execute_service_)
            {
                //calculate the distance to the cart_frame
                float distance_to_cart = sqrt(pow(cart_x_,2)+pow(cart_y_,2));
                //move the robot to the cart_frame
                if(distance_to_cart>0.05)
                {
                    robot_move(MAX_LINEAR_SPEED_, ZERO_ANGULAR_SPEED_);
                }
                else
                {
                    robot_move(ZERO_LINEAR_SPEED_,ZERO_ANGULAR_SPEED_);
                    execute_service_ = false;
                }
            }
        }
    }



    void service_callback(
        const std::shared_ptr<GoToLanding::Request> request,
        std::shared_ptr<GoToLanding::Response> response)
    {
        //if service request is true
        if(request->attach_to_shelf)
        {
            if(number_of_legs_==2)
            {
                //code to move the robot to the cart_frame inside laser callback
                execute_service_ = true;
                response->complete = true;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Number of legs detected is not 2: attach to shelf will not be executed");
                response->complete = false;
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Request is false: attach to shelf will not be executed");
            response->complete = false;
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyServiceServer>("robot/cmd_vel","scan","odom"));
    rclcpp::shutdown();
    return 0;
}