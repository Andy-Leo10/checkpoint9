#ifndef COMPOSITION__ATTACH_CLIENT_HPP_
#define COMPOSITION__ATTACH_CLIENT_HPP_
#include "my_components/visibility_control.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <math.h>
#include <chrono>
#include "attach_shelf/srv/go_to_loading.hpp"


using GoToLanding = attach_shelf::srv::GoToLoading;
using std::placeholders::_1;
using namespace std::chrono_literals;

namespace my_components
{
class AttachClient : public rclcpp::Node
{
public:
    COMPOSITION_PUBLIC
    explicit AttachClient(const rclcpp::NodeOptions & options);
private:
    //service client
    rclcpp::Client<GoToLanding>::SharedPtr srv_client_;
    bool service_done_=false;
    //timer
    rclcpp::TimerBase::SharedPtr timer_;
protected:
    void timer_callback();
    void response_callback(rclcpp::Client<GoToLanding>::SharedFuture future);
};
}  // namespace my_components

#endif  // COMPOSITION__ATTACH_CLIENT_HPP_