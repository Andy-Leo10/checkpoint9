/*ROS2 humble
call a service(/approach_shelf) for executing the final approach
*/

#include "my_components/attach_client.hpp"

namespace my_components
{
AttachClient::AttachClient(const rclcpp::NodeOptions & options)
    : Node("attach_client", options)
{
    //service client
    srv_client_ = this->create_client<GoToLanding>("approach_shelf");
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
} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)
