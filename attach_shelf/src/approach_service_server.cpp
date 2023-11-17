/*ROS2 galactic
-service name: approach_shelf
-detect the legs of the shelf using the laser intensity values
    1 shelf leg or none, it will return a False message
    both legs, the service will publish a transform named cart_frame to the center point between both legs
-the robot will use the TF to move towards the shelf
-then it will move forward 30 cm more (to end up right underneath the shelf)
-publish to the topics /elevator_up and /elevator_down
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
#include "attach_shelf/srv/go_to_loading.hpp"

using GoToLanding = attach_shelf::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;

class MyServiceServer : public rclcpp::Node
{
public:
    MyServiceServer(std::string topic_pub, std::string topic_laser, std::string topic_odometry) : Node("my_service_server")
    {
        //publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_pub, 10);
        //subscriber laser
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            topic_laser, 10, std::bind(&MyServiceServer::laser_intensities_callback, this, _1));
        //service
        service_ = this->create_service<GoToLanding>("approach_shelf", 
            std::bind(&MyServiceServer::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    //publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist pub_msg_;
    const float MAX_LINEAR_SPEED_=0.5;
    const float MAX_ANGULAR_SPEED_=0.4;
    const float ZERO_LINEAR_SPEED_ = 0.0;
    const float ZERO_ANGULAR_SPEED_ = 0.0;
    //subscriber laser
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    float front_distance_;
    //service
    rclcpp::Service<GoToLanding>::SharedPtr service_;
    //variables
    bool legs_0_=false;
    bool legs_1_=false;
    bool legs_2_=false;

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
        RCLCPP_INFO(this->get_logger(), "Legs detected: %zu", legs_indexes.size());
        for (std::vector<float>::size_type i = 0; i < legs_indexes.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), "Leg %zu: ", i);
            for (std::vector<float>::size_type j = 0; j < legs_indexes[i].size(); j++)
            {
                RCLCPP_INFO(this->get_logger(), "%d ", legs_indexes[i][j]);
            }
        }
    }

    void service_callback(
        const std::shared_ptr<GoToLanding::Request> request,
        std::shared_ptr<GoToLanding::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Recibida solicitud: %s", request->attach_to_shelf ? "true" : "false");
        response->complete = true;
    }

    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyServiceServer>("robot/cmd_vel","scan","odom"));
    rclcpp::shutdown();
    return 0;
}