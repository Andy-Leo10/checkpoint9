void laser_intensities_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            //detect the intensities greater than 7000 and consider them as legs
            //if there are close enough, consider them as one leg
            //then count the number of legs
            const float LEG_THRESHOLD = 7000.0;
            const float LEG_DISTANCE_THRESHOLD = 0.1;
            const float LEGS_DISTANCE_THRESHOLD = 0.2;
            const float MAX_DISTANCE = 3.5;
            const float MIN_DISTANCE = 0.12;
            const float ANGLE_INCREMENT = msg->angle_increment;
            const float MIN_ANGLE = msg->angle_min;
            const float MAX_ANGLE = msg->angle_max;
            const float MAX_ANGLE_DIFF = 0.1;
            const float MAX_LEG_ANGLE_DIFF = 0.2;
            const float LEG_ANGLE = 0.05;
            const float LEGS_DISTANCE = 0.3;
            const float LEGS_DISTANCE_TOLERANCE = 0.05;
            const float LEGS_ANGLE_TOLERANCE = 0.1;

            std::vector<float> intensities = msg->intensities;
            std::vector<float> distances;
            std::vector<float> angles;
            std::vector<int> leg_indexes;
            std::vector<int> legs_indexes;
            int n_legs = 0;

            //calculate distances and angles
            for (int i = 0; i < intensities.size(); i++)
            {
                float intensity = intensities[i];
                if (intensity > LEG_THRESHOLD)
                {
                    float angle = MIN_ANGLE + i * ANGLE_INCREMENT;
                    float distance = intensity / 10000.0;
                    if (distance > MAX_DISTANCE || distance < MIN_DISTANCE)
                    {
                        continue;
                    }
                    distances.push_back(distance);
                    angles.push_back(angle);
                }
            }

            //detect legs
            for (int i = 0; i < distances.size(); i++)
            {
                float distance = distances[i];
                float angle = angles[i];
                bool is_leg = true;
                for (int j = 0; j < distances.size(); j++)
                {
                    if (i == j)
                    {
                        continue;
                    }
                    float other_distance = distances[j];
                    float other_angle = angles[j];
                    float distance_diff = fabs(distance - other_distance);
                    float angle_diff = fabs(angle - other_angle);
                    if (distance_diff < LEG_DISTANCE_THRESHOLD && angle_diff < MAX_LEG_ANGLE_DIFF)
                    {
                        is_leg = false;
                        break;
                    }
                }
                if (is_leg)
                {
                    leg_indexes.push_back(i);
                }
            }

            //group legs
            for (int i = 0; i < leg_indexes.size(); i++)
            {
                int leg_index = leg_indexes[i];
                float leg_angle = angles[leg_index];
                bool is_new_leg_group = true;
                for (int j = 0; j < legs_indexes.size(); j++)
                {
                    int other_leg_index = legs_indexes[j];
                    float other_leg_angle = angles[other_leg_index];
                    float angle_diff = fabs(leg_angle - other_leg_angle);
                    if (angle_diff < MAX_ANGLE_DIFF)
                    {
                        legs_indexes[j] = leg_index;
                        is_new_leg_group = false;
                        break;
                    }
                }
                if (is_new_leg_group)
                {
                    legs_indexes.push_back(leg_index);
                }
            }

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