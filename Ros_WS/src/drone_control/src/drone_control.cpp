#include "rclcpp/rclcpp.hpp"
#include "drone_control.h"


    PID::PID(double kp, double ki, double kd, double output_limit) 
        : kp_(kp), ki_(ki), kd_(kd), output_limit_(output_limit), 
          prev_error_(0), integral_(0) {}

    double PID::compute(double error, double dt) {
        double derivative = (error - prev_error_) / dt;
        integral_ += error * dt;
        output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        prev_error_ = error;

        // Implement anti-windup
        if (output > output_limit_) {
            output = output_limit_;
            integral_ -= error * dt; // Subtract the last addition to prevent windup
        } else if (output < -output_limit_) {
            output = -output_limit_;
            integral_ += error * dt; // Subtract the last addition to prevent windup
        }

        return output;
    }

    void PID::reset() {
        prev_error_ = 0;
        integral_ = 0;
    }



    DroneControlNode::DroneControlNode() : Node("drone_control_node"),
                         pid_x_(1.60, 0.15, 0.10, 5.5),  
                         pid_y_(1.60, 0.15, 0.10, 5.5),
                         pid_z_(0.0035, 0.0025, 0.0009, 3.7),
                         pidFind_x_(1.60, 0.15, 0.10, 2.5),  
                         pidFind_y_(1.60, 0.15, 0.10, 2.5),
                         pidFind_z_(0.0020, 0.0015, 0.0009, 3),
                         pid_twist_(0.0078, 0.0053, 0.0015, 1.6) //3.48 = (200 * (M_PI / 180)) dat 0 na D

    {
        auto qos_profile = rclcpp::SystemDefaultsQoS();
       publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/uas_2/setpoint_velocity/cmd_vel", 10);  
       data_publisher_ = this->create_publisher<dronecontrol_msg::msg::DroneControlData>("/drone_data_topic", 10);

       
        subscriber_leader_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/uas_1/local_position/pose", qos_profile, std::bind(&DroneControlNode::pose_callback_1, this, std::placeholders::_1));
        subscriber_follower_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/uas_2/local_position/pose", qos_profile, std::bind(&DroneControlNode::pose_callback_2, this, std::placeholders::_1)); 
        subscriber_followerVel_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/uas_2/local_position/velocity_body", qos_profile, std::bind(&DroneControlNode::velocity_callback_2, this, std::placeholders::_1));


        subscriber_control_ = this->create_subscription<std_msgs::msg::Bool>(
            "/control_topic", qos_profile, std::bind(&DroneControlNode::controlCallback, this, std::placeholders::_1));

        subscriber_setDistance_ = this->create_subscription<std_msgs::msg::Float64>(
            "/set_distance", qos_profile, std::bind(&DroneControlNode::setDistanceCallback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/semantic/colored_map", qos_profile,std::bind(&DroneControlNode::imgCallback, this, std::placeholders::_1)); ///depth_camera/points

        image_sub_depth = this->create_subscription<sensor_msgs::msg::Image>(
                "/depth_cameramain", qos_profile,std::bind(&DroneControlNode::imgCallbackDepth, this, std::placeholders::_1)); //


        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&DroneControlNode::timer_callback, this));

        lastUpdateTime_ = this->now();
        lastDepthUpdateTimeDepth_  = this->now();
        lastDepthUpdateTimeLateral_  = this->now();
    }


    void DroneControlNode::pose_callback_1(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        pose_1_ = msg->pose;
    }

    void DroneControlNode::pose_callback_2(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        pose_2_ = msg->pose;
    }

    void DroneControlNode::velocity_callback_2(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        velocity_2_ = msg->twist;
    }

    void DroneControlNode::controlCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        control = msg->data;
    }

    void DroneControlNode::setDistanceCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        target_distance = msg->data;
    }


    void DroneControlNode::imgCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image = cv_ptr->image;
        cv::Mat grayImage, thresholdedImage;

        // Convert the image to grayscale
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

        // Threshold the image to isolate non-black pixels (potential drone pixels)
        cv::threshold(grayImage, thresholdedImage, 1, 255, cv::THRESH_BINARY);

        // Find contours in the thresholded image
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(thresholdedImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        maxArea = 0;
        std::vector<cv::Point> largestContour;

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
                droneRect = cv::boundingRect(contour);
            }
        }

        // Calculate the center of the image (camera view of the follower drone)
        cv::Point imageCenter(image.cols / 2, image.rows / 2);
        // Draw a green dot at the center of the image
        cv::circle(image, imageCenter, 5, cv::Scalar(0, 255, 0), -1);

        // If a leader drone is detected in the image
        if (maxArea > 0) {
            
            // Calculate the center of the bounding rectangle
            cv::Point droneCenter(droneRect.x + droneRect.width / 2, droneRect.y + droneRect.height / 2);

            // Draw a rectangle around the drone
            cv::rectangle(image, droneRect, cv::Scalar(0, 255, 0), 2);
            cv::circle(image, droneCenter, 5, cv::Scalar(0, 0, 255), -1);

            // Draw a blue line between the center of the follower's camera view and the leader drone's center
            cv::line(image, imageCenter, droneCenter, cv::Scalar(255, 0, 0), 2);


            // Calculate the error between the leader's center x position and the center x of the follower's view
            image_center_x = image.cols / 2;
            image_center_y = image.rows / 2;

            // You've already calculated the droneCenter for the leader drone, so:
            leader_drone_image_x_ = droneCenter.x;
            leader_drone_image_y_ = droneCenter.y;
        }

        // Show the image with the rectangle, dots, and line
        cv::imshow("Drone Tracking", image);
        cv::waitKey(1);
    }


    void DroneControlNode::imgCallbackDepth(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat depth_image = cv_ptr->image;

        if (maxArea > 0) {
            cv::Rect boundedRect = droneRect & cv::Rect(0, 0, depth_image.cols, depth_image.rows);
            cv::Mat depthRoi = depth_image(boundedRect);

            // Temporary variables to store the computed min and max depths
            double tempMinDepth, tempMaxDepth;
            cv::minMaxIdx(depthRoi, &tempMinDepth, &tempMaxDepth, NULL, NULL, depthRoi != 0);

            // Check if the computed minimum depth is valid
            if (tempMinDepth > 0 && !std::isinf(tempMinDepth)) {
                minDepth = tempMinDepth; // Update the minDepth only if the new value is valid
            }

        }

        if (maxArea > 0) {
            cv::rectangle(depth_image, droneRect, cv::Scalar(255, 0, 0));
        }
        cv::imshow("Depth Image with ROI", depth_image);
        cv::waitKey(1);
    }



    void DroneControlNode::distance_callback() {
        dx = pose_1_.position.x - pose_2_.position.x;
        dy = pose_1_.position.y - pose_2_.position.y;
        dz = pose_1_.position.z - pose_2_.position.z;

        distance_xy = std::sqrt(dx*dx + dy*dy);
    }
    
    void DroneControlNode::timer_callback() {
    if(control){
        distance_callback();
        
        // Altitude error (Z axis)
        error_altitude = image_center_y - leader_drone_image_y_; 
        error_lateral = image_center_x - leader_drone_image_x_;
        error_distance = minDepth - target_distance;

        RCLCPP_INFO(this->get_logger(), "Height: [%i]", error_altitude);
        RCLCPP_INFO(this->get_logger(), "Depth: [%f]", minDepth);
        RCLCPP_INFO(this->get_logger(), "lateral: [%i]", error_lateral);

        geometry_msgs::msg::Quaternion followerOrientation = pose_2_.orientation; 
        double yaw = quaternionToYaw(followerOrientation);

        control_signal_lateral = pid_twist_.compute(error_lateral, dt);
        
        if(maxArea > 0){
            if(!finding){
                //reset regulators for finding
                if(pidFind_x_.prev_error_ || pidFind_y_.prev_error_ || pidFind_z_.prev_error_){
                    pidFind_x_.reset();
                    pidFind_y_.reset();
                    pidFind_z_.reset();
                }

                if(phase != -2) phase = -2;
                // Check if the drone is not moving or if it's pitched down, and the altitude error is within bounds, or the leader drone moves fast.
                if ((!isDroneMoving(1) && std::abs(error_altitude) <= 100) || isPitchBeyondThreshold(followerOrientation, 3) || isDroneMoving(2) || (minDepth > target_distance && isDroneMoving(0.2))) {
                    // Then, control X and Y based on error distance if not very close or if drone is moving
                    if ((std::abs(error_distance) >= 0.2) || isDroneMoving(1)) {
                        control_signal_x = pid_x_.compute(error_distance * std::cos(yaw), dt);
                        control_signal_y = pid_y_.compute(error_distance * std::sin(yaw), dt); 
                    } else {
                        // Optimization when very close and nearly stationary
                        pid_x_.reset();
                        pid_y_.reset();
                        control_signal_x = control_signal_y = 0;
                    }
                } else {
                    // Reset controls if none of the above conditions are met
                    pid_x_.reset();
                    pid_y_.reset();
                    control_signal_x = control_signal_y = 0;
                } 
                            
                if(std::abs(error_altitude) >= 20 || !isPitchBeyondThreshold(followerOrientation, 7)){
                    control_signal_z = pid_z_.compute(error_altitude, dt);
                } else {
                    pid_z_.reset();
                    control_signal_z = 0;
                }
            } else{
                //if drone was lost for too long so optimal finding and regulation is set
                if(pid_x_.prev_error_ || pid_y_.prev_error_ || pid_z_.prev_error_){
                    pid_x_.reset();
                    pid_y_.reset();
                    pid_z_.reset();
                    control_signal_x = 0;
                    control_signal_y = 0;
                }


                auto now = this->now();
                auto elapsedTime = (now - lastDepthUpdateTimeLateral_).seconds();

                if(error_lateral >= 80) lastDepthUpdateTimeLateral_ = now;

                if(elapsedTime >= 1) {
                    control_signal_z = pidFind_z_.compute(error_altitude, dt);
                    if(std::abs(error_altitude) <= 80 || isDroneMoving(0.2)){
                        control_signal_x = pidFind_x_.compute(error_distance * std::cos(yaw), dt);
                        control_signal_y = pidFind_y_.compute(error_distance * std::sin(yaw), dt); 
                    } else{
                        pidFind_x_.reset();
                        pidFind_y_.reset();
                        control_signal_x = control_signal_y = 0;
                    }
                }
                else control_signal_z = 0;

                if(error_distance == 0.5){
                    finding = false;
                }
            } 
                
        }
        else{
            //if drone is lost, search pattern is executed
            if(phase == -2){
                lastUpdateTime_ = this->now();
                phase = -1;
                recalcutaleFindingSpeed = false;

            }
            
            executeSearchPattern();
        }


        RCLCPP_INFO(this->get_logger(), "error: [%f, %f, %f, %f]",
        control_signal_x, control_signal_y, control_signal_z, control_signal_lateral);
        } else {
            // Control is disabled, reset all PIDs and control signals
            if(pid_x_.prev_error_ || pid_y_.prev_error_ || pid_z_.prev_error_){
                pid_x_.reset();
                pid_y_.reset();
                pid_z_.reset();
                pid_twist_.reset();
            }
        
            control_signal_x = control_signal_y = control_signal_z = control_signal_lateral = 0;
        }

        // Prepare and publish control message
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        msg.twist.angular.z = control_signal_lateral;
        msg.twist.linear.x = control_signal_x;  
        msg.twist.linear.y = control_signal_y;  
        msg.twist.linear.z = control_signal_z;

        publisher_->publish(msg);
        publishDroneData();
    }



    bool DroneControlNode::isPitchBeyondThreshold(const geometry_msgs::msg::Quaternion& orientation, double threshold_deg) {
        // Calculate pitch in radians from the quaternion
        double pitch = std::atan2(2.0 * (orientation.w * orientation.y + orientation.z * orientation.x), 1.0 - 2.0 * (orientation.y * orientation.y + orientation.x * orientation.x));

        // Convert pitch to degrees
        double pitch_deg = pitch * (180.0 / M_PI);

        RCLCPP_INFO(this->get_logger(), "pitch: [%f]", std::abs(pitch_deg));
        return std::abs(pitch_deg) > threshold_deg;
    }

    bool DroneControlNode::isDroneMoving(double velocityThreshold) {
        // Calculate the magnitude of the velocity vector
        double velocityMagnitude = std::sqrt(
            velocity_2_.linear.x * velocity_2_.linear.x +
            velocity_2_.linear.y * velocity_2_.linear.y 
        );

        return velocityMagnitude > velocityThreshold;
    }



    double DroneControlNode::calculateRelativePosition(double distance) {
        geometry_msgs::msg::Quaternion followerOrientation = pose_2_.orientation; 


        double yaw = quaternionToYaw(followerOrientation);

        // Calculate deltaX and deltaY based on distance and yaw angle
        double deltaX = distance * cos(yaw);
        double deltaY = distance * sin(yaw);
      
        return std::atan2(deltaY, deltaX);
    }

    double DroneControlNode::quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);
        
        return yaw;
    }

    void DroneControlNode::executeSearchPattern() {
        auto now = this->now();
        auto elapsedTime = (now - lastUpdateTime_).seconds();

        if(finding) pid_twist_.reset();
        

        // Define constant speeds
        const double speed = 0.5; 
        double yawSpeed;
        if(control_signal_lateral > 0) yawSpeed = 1;
        else  yawSpeed = -1;


        if(phase == -1){
            if(!recalcutaleFindingSpeed){
                control_signal_x = 0.5*control_signal_x;
                control_signal_y = 0.5*control_signal_y;
                if(std::sqrt(error_altitude) >= 230) control_signal_z = 1.3 *control_signal_z;
                else control_signal_z = 0;
                recalcutaleFindingSpeed = true;
            }

            if (elapsedTime >= 1.5) {
                phase = 0;
                lastUpdateTime_ = now; // Reset timing
            } 
        } else if(phase == 0){
            if (elapsedTime <= 6) {
                control_signal_x = 0;
                control_signal_y = 0;
                control_signal_z = 0;
                control_signal_lateral = yawSpeed;
            } else {
                // Move to next phase
                phase = 1;
                lastUpdateTime_ = now; // Reset timing
            }

        } else if (phase == 1) { 
            if (elapsedTime <= 6*2) {
                finding = true;
                control_signal_x = 0;
                control_signal_z = speed;
                control_signal_lateral = yawSpeed;
            } else {
                // Move to next phase
                phase = 2;
                lastUpdateTime_ = now; // Reset timing
            }
        } else if (phase == 2) { 
            if (elapsedTime <= 6*2) {
                control_signal_x = -speed;
                if(pose_2_.position.z <= 2) control_signal_z = 0;
                else control_signal_z = -speed; 
                control_signal_lateral = yawSpeed;
            } else {
                phase = 3;
                lastUpdateTime_ = now; 
            }
        } else if (phase == 3) { 
            if (elapsedTime <= 6*2) {
                control_signal_x = speed;
                if(pose_2_.position.z <= 2) control_signal_z = 0;
                else control_signal_z = -speed; 
                control_signal_lateral = yawSpeed;
            } else {
                phase = 4; 
                lastUpdateTime_ = now; 
            }
        } else if (phase == 4) { 
            if (elapsedTime <= 6*2) {
                control_signal_x = 0;
                control_signal_z = speed;
                control_signal_lateral = yawSpeed;
            } else {
                phase = 1; 
                lastUpdateTime_ = now; 
            }
        }
    }


    void DroneControlNode::publishDroneData() {
        auto msg = dronecontrol_msg::msg::DroneControlData();

        msg.x1 = pose_1_.position.x;
        msg.y1 = pose_1_.position.y;
        msg.z1 = pose_1_.position.z;
        msg.x2 = pose_2_.position.x;
        msg.y2 = pose_2_.position.y;
        msg.z2 = pose_2_.position.z;

        msg.distance_xy = minDepth;
        msg.target_distance = target_distance;

        msg.control_signal_x = control_signal_x;
        msg.control_signal_y = control_signal_y;
        msg.control_signal_z = control_signal_z;
        msg.control_angular_z = control_signal_lateral;

        data_publisher_->publish(msg);
    }



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneControlNode>());
    rclcpp::shutdown();
    return 0;
}


