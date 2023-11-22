#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <math.h>
#include <chrono>
#include "attach_shelf/srv/go_to_loading.hpp"

using GoToLanding = attach_shelf::srv::GoToLoading;

class AttachClient2 : public rclcpp::Node
{
public:
AttachClient2(const rclcpp::NodeOptions & options)
: Node("attach_client2", options)
{
    //service client
    srv_client_ = this->create_client<GoToLanding>("approach_shelf");

    // Wait for the service to be available
    srv_client_->wait_for_service();

    //call the service
    std::shared_ptr<GoToLanding::Request> request =
        std::make_shared<GoToLanding::Request>();
    request->attach_to_shelf = true;
    auto result = srv_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Service call successful: %d", result.get()->complete);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Service call failed");
    }
}
private:
    //service client
    rclcpp::Client<GoToLanding>::SharedPtr srv_client_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AttachClient2>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}