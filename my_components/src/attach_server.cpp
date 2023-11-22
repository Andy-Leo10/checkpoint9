/*ROS2 humble
-service name: approach_shelf
-detect the legs of the shelf using the laser intensity values
    1 shelf leg or none, it will return a False message
    both legs, the service will publish a transform named cart_frame to the center point between both legs
-the robot will use the TF to move towards the shelf
-then it will move forward 30 cm more (to end up right underneath the shelf)
-publish to the topics /elevator_up and /elevator_down
*/

#include "my_components/attach_server.hpp"

namespace my_components
{
AttachServer::AttachServer(const rclcpp::NodeOptions & options)
        : Node("attach_server"), TIMER_PERIOD_MS_(100)
{
    //subscriber laser
    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        topic_laser_, 10, std::bind(&AttachServer::laser_intensities_callback, this, _1));
    //service
    service_ = this->create_service<GoToLanding>("approach_shelf", 
        std::bind(&AttachServer::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    execute_service_ = false;
    // initialize the tf2 listener for time stamp
    auto clock = this->get_clock();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    //broadcast transform
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    //publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_pub_, 10);
    //timer
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds((int)TIMER_PERIOD_MS_),
    std::bind(&AttachServer::timer_callback, this));
    //elevator up: ros2 topic pub /elevator_up std_msgs/msg/Empty -1
    elevator_up_publisher_ = this->create_publisher<std_msgs::msg::Empty>(topic_elevator_up_, 10);
}
void AttachServer::timer_callback()
{
    if(move_extra_distance_)
    {
        //stop the robot after travel an "extra_distance_"
        //based on the time period
        if(times_to_move_extra_distance_>0)
        {
            RCLCPP_INFO(this->get_logger(), "Moving extra distance: %d", times_to_move_extra_distance_);
            robot_move(MAX_LINEAR_SPEED_, ZERO_ANGULAR_SPEED_);
            times_to_move_extra_distance_--;
        }
        else
        {
            robot_move(ZERO_LINEAR_SPEED_, ZERO_ANGULAR_SPEED_);
            move_extra_distance_ = false;
            execute_service_ = false;
            //publish to the topics /elevator_up
            RCLCPP_INFO(this->get_logger(), "Publishing to elevator_up");
            std_msgs::msg::Empty msg;
            elevator_up_publisher_->publish(msg);
        }

    }
}
void AttachServer::laser_intensities_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
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
        cart_yaw_ = (msg->angle_min + (leg_index_1 + leg_index_2) / 2.0 * msg->angle_increment);
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
        if(execute_service_ && !move_extra_distance_)
        {
            //calculate the distance to the cart_frame
            float distance_to_cart = sqrt(pow(cart_x_,2)+pow(cart_y_,2));
            //move the robot to the cart_frame
            if(distance_to_cart>0.4)
            {
                RCLCPP_INFO(this->get_logger(), "Moving to cart_frame");
                robot_move(MAX_LINEAR_SPEED_, ZERO_ANGULAR_SPEED_);
            }
            else
            {
                //stop the robot after travel an "extra_distance_"
                //calculate the number of times to move the robot
                times_to_move_extra_distance_ = (int)((extra_distance_/MAX_LINEAR_SPEED_)/(TIMER_PERIOD_MS_/1000.0));
                RCLCPP_INFO(this->get_logger(), "TIMES TO MOVE EXTRA DISTANCE: %d", times_to_move_extra_distance_);
                //by enabling a timer
                move_extra_distance_ = true;
            }
        }
        
    }
}
void AttachServer::service_callback(
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
void AttachServer::robot_move(float linear_speed, float angular_speed)
{
    pub_msg_.linear.x = linear_speed;
    pub_msg_.angular.z = angular_speed;
    publisher_->publish(pub_msg_);
}
} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)