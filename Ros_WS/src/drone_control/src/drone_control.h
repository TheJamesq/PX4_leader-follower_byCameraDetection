// DroneControlNode.h
#ifndef DRONECONTROLNODE_H
#define DRONECONTROLNODE_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "dronecontrol_msg/msg/drone_control_data.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cmath>

class PID {
public:
    PID(double kp, double ki, double kd, double output_limit);
    double compute(double error, double dt);
    void reset();

    double kp_, ki_, kd_;
    double prev_error_, integral_;
    double output_limit_, output;
};

class DroneControlNode : public rclcpp::Node {
public:
    DroneControlNode();

    void pose_callback_1(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void pose_callback_2(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void velocity_callback_2(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void controlCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void setDistanceCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void imgCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void imgCallbackDepth(const sensor_msgs::msg::Image::SharedPtr msg);
    void distance_callback();
    void timer_callback();
    void publishDroneData();
    double calculateRelativePosition(double distance);
    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q);
    bool isPitchBeyondThreshold(const geometry_msgs::msg::Quaternion& orientation, double threshold_deg);
    bool isDroneMoving(double velocityThreshold);
    void executeSearchPattern();

    PID pid_x_, pid_y_, pid_z_, pid_twist_, pidFind_x_, pidFind_y_, pidFind_z_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Publisher<dronecontrol_msg::msg::DroneControlData>:: SharedPtr data_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_leader_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_follower_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_followerVel_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_control_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_setDistance_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_depth;
    geometry_msgs::msg::Pose pose_1_;
    geometry_msgs::msg::Pose pose_2_;
    geometry_msgs::msg::Twist velocity_2_;
    geometry_msgs::msg::Point previousPosition_;
    rclcpp::Time lastUpdateTime_, lastDepthUpdateTimeDepth_, lastDepthUpdateTimeLateral_;
    cv::Rect droneRect;
    double dx, dy, dz, control_signal_lateral, control_signal_x, control_signal_y, control_signal_z ,error_angle, error_distance, maxArea, previousMinDepth_;
    double distance_xy, minDepth, maxDepth;
    double target_distance = 1.5;
    bool control = false;
    bool finding =false,  recalcutaleFindingSpeed = false;
    double dt = 0.01;
    cv::Mat currentImage;
    int leader_drone_image_x_, image_center_x, error_altitude, error_lateral, image_center_y, leader_drone_image_y_;  
    int phase = -2;
};

#endif // DRONECONTROLNODE_H

