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
#include "attach_shelf/srv/got_to_loading.hpp"

class MyServiceServer : public rclcpp::Node
{
public:
    MyServiceServer() : Node("my_service_server")
    {
        service_ = this->create_service<attach_shelf::srv::got_to_loading>("mi_servicio", 
            std::bind(&MyServiceServer::handle_service, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

private:
    void handle_service(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<attach_shelf::srv::got_to_loading::Request> request,
        std::shared_ptr<attach_shelf::srv::got_to_loading::Response> response)
    {
        (void)request_header;
        RCLCPP_INFO(this->get_logger(), "Recibida solicitud: %s", request->attach_to_shelf ? "true" : "false");
        response->complete = true;
    }

    rclcpp::Service<attach_shelf::srv::got_to_loading>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyServiceServer>());
    rclcpp::shutdown();
    return 0;
}