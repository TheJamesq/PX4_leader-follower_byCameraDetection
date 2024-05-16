#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QTimer>
#include <QMessageBox>
#include <QProcess>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "std_msgs/msg/bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "std_msgs/msg/float64.hpp"
#include <QtDataVisualization/QScatterDataItem>
#include <QVector3D>
#include <QStatusBar>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <QtDataVisualization/Q3DScatter>
#include "UIInitializer.h"


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

   
public slots:
    void cameraButtonClicked();
    void updateCameraWindow();
    QImage MatToQImage(const cv::Mat& mat) ;

    void armButton_follower();
    void offboardButton_follower();
    void takeoffButton_follower();
    void controlButton_follower();
    
    void armButton_leader();
    void takeoffButton_leader();

    void publisControlDistance();

private:
    void poseCallback1(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> msg);
    void poseCallback2(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> msg);
    
    void velCallback1(const std::shared_ptr<const geometry_msgs::msg::TwistStamped> msg);
    void velCallback2(const std::shared_ptr<const geometry_msgs::msg::TwistStamped> msg);
    void imgCallback(const std::shared_ptr<const sensor_msgs::msg::Image> msg);
    
    
    
    void updateUI();
    
    void updateBarGraphData(const QVector<double> &drone1Values, const QVector<double> &drone2Values);
    void Arm(auto drone_client);
    void TakeOff(auto drone_client, double i);




    QTimer *timer;

    UIInitializer *ui;
    QWidget *opencvImageWindow = nullptr;
    bool control;
    bool received_follower_pose, received_leader_pose, received_follower_vel, received_leader_vel;
    double setDistance = 1.5;
    int numPoints = 100;
    double angleStep = 360.0 / numPoints;


    QVector<double> Vel_bar_leader; 
    QVector<double> Vel_bar_follower; 

    std::mutex imageMutex;
    cv::Mat currentImage;
    bool isCameraWindowOpen = false;



    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_leader_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_follower_pose_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_leader_vel_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_follower_vel_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr control_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_publisher_;


    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_follower_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_leader_;

    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_follower;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_leader;

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
	
    std::shared_ptr<const geometry_msgs::msg::PoseStamped> Pose_1;
    std::shared_ptr<const geometry_msgs::msg::PoseStamped> Pose_2;
    std::shared_ptr<const geometry_msgs::msg::TwistStamped> Vel_1;
    std::shared_ptr<const geometry_msgs::msg::TwistStamped> Vel_2;
    


    
};

#endif // MAINWINDOW_H

