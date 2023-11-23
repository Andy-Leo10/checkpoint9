#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <math.h>
#include <chrono>
#include "attach_shelf/srv/go_to_loading.hpp"

using GoToLanding = attach_shelf::srv::GoToLoading;
using namespace std::chrono_literals;
using std::placeholders::_1;

class AttachClient : public rclcpp::Node
{
public:
AttachClient(const rclcpp::NodeOptions & options)
: Node("attach_client2", options)
{
    //service client
    srv_client_ = this->create_client<GoToLanding>("approach_shelf");
    //timer
    timer_=this->create_wall_timer(100ms, std::bind(&AttachClient::timer_callback, this));
}
private:
    //service client
    rclcpp::Client<GoToLanding>::SharedPtr srv_client_;
    bool service_done_=false;
    //timer
    rclcpp::TimerBase::SharedPtr timer_;
protected:
    void timer_callback()
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
    void response_callback(rclcpp::Client<GoToLanding>::SharedFuture future)
    {
        RCLCPP_INFO(this->get_logger(), "Service call successful: %d", future.get()->complete);
        service_done_=true;
        timer_->cancel();
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AttachClient>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}