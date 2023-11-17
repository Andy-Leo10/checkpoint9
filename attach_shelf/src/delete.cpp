void laser_intensities_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            //count legs
            n_legs = legs_indexes.size();

            //check if there are two legs
            if (n_legs == 2)
            {
                int leg_index_1 = legs_indexes[0];
                int leg_index_2 = legs_indexes[1];
                float leg_angle_1 = angles[leg_index_1];
                float leg_angle_2 = angles[leg_index_2];
                float legs_angle_diff = fabs(leg_angle_1 - leg_angle_2);
                float legs_distance = fabs(distances[leg_index_1] - distances[leg_index_2]);
                if (legs_angle_diff < LEGS_ANGLE_TOLERANCE && legs_distance > LEGS_DISTANCE - LEGS_DISTANCE_TOLERANCE && legs_distance < LEGS_DISTANCE + LEGS_DISTANCE_TOLERANCE)
                {
                    //publish transform
                    float cart_x = (distances[leg_index_1] + distances[leg_index_2]) / 2.0;
                    float cart_y = 0.0;
                    float cart_z = 0.0;
                    float cart_roll = 0.0;
                    float cart_pitch = 0.0;
                    float cart_yaw = (leg_angle_1 + leg_angle_2) / 2.0;
                    tf2::Quaternion cart_quat;
                    cart_quat.setRPY(cart_roll, cart_pitch, cart_yaw);
                    geometry_msgs::msg::TransformStamped transformStamped;
                    transformStamped.header.stamp = rclcpp::Time::now();
                    transformStamped.header.frame_id = "odom";
                    transformStamped.child_frame_id = "cart_frame";
                    transformStamped.transform.translation.x = cart_x;
                    transformStamped.transform.translation.y = cart_y;
                    transformStamped.transform.translation.z = cart_z;
                    transformStamped.transform.rotation.x = cart_quat.x();
                    transformStamped.transform.rotation.y = cart_quat.y();
                    transformStamped.transform.rotation.z = cart_quat.z();
                    transformStamped.transform.rotation.w = cart_quat.w();
                    tf_broadcaster_.sendTransform(transformStamped);

                    //move robot
                    pub_msg_.linear.x = MAX_LINEAR_SPEED_;
                    pub_msg_.angular.z = ZERO_ANGULAR_SPEED_;
                    publisher_->publish(pub_msg_);
                    rclcpp::sleep_for(std::chrono::milliseconds(1000));
                    pub_msg_.linear.x = ZERO_LINEAR_SPEED_;
                    pub_msg_.angular.z = ZERO_ANGULAR_SPEED_;
                    publisher_->publish(pub_msg_);

                    //publish elevator commands
                    std_msgs::msg::String msg_up;
                    std_msgs::msg::String msg_down;
                    msg_up.data = "up";
                    msg_down.data = "down";
                    publisher_elevator_up_->publish(msg_up);
                    publisher_elevator_down_->publish(msg_down);
                }
            }
        }