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
    //timer
    timer_=this->create_wall_timer(100ms, std::bind(&AttachClient::timer_callback, this));
}
void AttachClient::timer_callback()
{
    if(!service_done_&&!srv_client_->wait_for_service(1s)){
        RCLCPP_ERROR(this->get_logger(), "Service not available after waiting");
        return;
    }
    //call the service
    auto request = std::make_shared<GoToLanding::Request>();
    request->attach_to_shelf = true;
    //response callback
    auto result = srv_client_->async_send_request(request, 
        std::bind(&AttachClient::response_callback, this, _1));
}
void AttachClient::response_callback(rclcpp::Client<GoToLanding>::SharedFuture future)
{
    RCLCPP_INFO(this->get_logger(), "Service call successful: %d", future.get()->complete);
    service_done_=true;
    timer_->cancel();
}
} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)
