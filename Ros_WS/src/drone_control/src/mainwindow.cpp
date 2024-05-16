#include "mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    statusBar()->setStyleSheet("color: green;");

    
    QWidget *centralWidget = new QWidget(this);
    centralWidget->setStyleSheet("background-color: white;");
    QMainWindow::setCentralWidget(centralWidget);


    // Set up window dimensions25
    QWidget::resize(1300, 800);
    //centralWidget->resize(1300, 800);
    
    ui = new UIInitializer(centralWidget);
    //ui->offboardButton_follower->setEnabled(false);
    this->setMenuBar(ui->menuBar);
    
    // Set the title for the main window
    setWindowTitle("Drone Control Panel");

    connect(ui->cameraButton, &QPushButton::clicked, this, &MainWindow::cameraButtonClicked);

    connect(ui->armButton_follower, &QPushButton::clicked, this, &MainWindow::armButton_follower);
    connect(ui->offboardButton_follower, &QPushButton::clicked, this, &MainWindow::offboardButton_follower);
    connect(ui->takeoffButton_follower, &QPushButton::clicked, this, &MainWindow::takeoffButton_follower);
    connect(ui->controlButton_follower, &QPushButton::clicked, this, &MainWindow::controlButton_follower);
    
    connect(ui->armButton_leader, &QPushButton::clicked, this, &MainWindow::armButton_leader);
    connect(ui->takeoffButton_leader, &QPushButton::clicked, this, &MainWindow::takeoffButton_leader); 
    connect(ui->publishButton, &QPushButton::clicked, this, &MainWindow::publisControlDistance);  

    // Initialize ROS 2 node
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("gui");

    // Subscription to the pose topic //mavros/local_position/velocity_body

    auto qos_profile = rclcpp::SystemDefaultsQoS();
    subscriber_leader_pose_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/mavros/uas_1/local_position/pose", qos_profile,
                    std::bind(&MainWindow::poseCallback1, this, std::placeholders::_1)
                    );

    subscriber_follower_pose_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/mavros/uas_2/local_position/pose", qos_profile,
                    std::bind(&MainWindow::poseCallback2, this, std::placeholders::_1)
                    );
                    
    subscriber_leader_vel_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
                    "/mavros/uas_1/local_position/velocity_local", qos_profile,
                    std::bind(&MainWindow::velCallback1, this, std::placeholders::_1)
                    );

    subscriber_follower_vel_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
                    "/mavros/uas_2/local_position/velocity_local", qos_profile,
                    std::bind(&MainWindow::velCallback2, this, std::placeholders::_1)
                    );

    image_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
            "/semantic/colored_map", qos_profile,
            std::bind(&MainWindow::imgCallback, this, std::placeholders::_1));
                        
    set_mode_client_ = node->create_client<mavros_msgs::srv::SetMode>("/mavros/uas_2/set_mode");

    arming_client_follower_ = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/uas_2/cmd/arming");
    arming_client_leader_ = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/uas_1/cmd/arming");

    takeoff_client_follower = node->create_client<mavros_msgs::srv::CommandTOL>("/mavros/uas_2/cmd/takeoff");
    takeoff_client_leader = node->create_client<mavros_msgs::srv::CommandTOL>("/mavros/uas_1/cmd/takeoff");

    control_publisher_ = node->create_publisher<std_msgs::msg::Bool>("control_topic", 10);
    distance_publisher_ = node->create_publisher<std_msgs::msg::Float64>("set_distance", 10);
                    
                    
    statusBar()->showMessage("Ready", 4000);
    // Start a Qt timer to run the ROS 2 event loop

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, [this]() {
        rclcpp::spin_some(node);
        updateUI();
    });
    timer->start(100); 
    
}

MainWindow::~MainWindow()
{
    rclcpp::shutdown();
    delete timer;  // Clean up the timer

}



void MainWindow::velCallback1(std::shared_ptr<const geometry_msgs::msg::TwistStamped> msg)
{
	Vel_1 = msg;
    received_leader_vel = true;
}

void MainWindow::velCallback2(std::shared_ptr<const geometry_msgs::msg::TwistStamped> msg)
{
	Vel_2 = msg;
    received_follower_vel = true;
}

void MainWindow::poseCallback1(std::shared_ptr<const geometry_msgs::msg::PoseStamped> msg)
{
	Pose_1 = msg;
    received_leader_pose = true;
}

void MainWindow::poseCallback2(std::shared_ptr<const geometry_msgs::msg::PoseStamped> msg)
{
	Pose_2 = msg;
    received_follower_pose = true;
}

void MainWindow::imgCallback(std::shared_ptr<const sensor_msgs::msg::Image> msg) {
    /*cv_bridge::CvImagePtr cv_ptr;
    std::lock_guard<std::mutex> lock(imageMutex);
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e) {
        return;
    }

    currentImage = cv_ptr->image.clone();*/
}


void MainWindow::updateUI()
{

    QtDataVisualization::QScatterDataArray data1;
    QtDataVisualization::QScatterDataArray data2;
    QtDataVisualization::QScatterDataArray circlePoints;
    double angle, x, y;

    if (received_leader_pose || received_leader_vel) {
        ui->leaderStatusLabel->setText("Leader Mavlink: Connected");
        ui->leaderStatusLabel->setStyleSheet("background-color: green; padding: 5px;");

       if(Pose_1){
        // Update QLabel for drone 2
    	    ui->label_leader_pos->setText("X: " + QString::number(Pose_1->pose.position.x, 'f', 4) + "\n" +
    	 		       "Y: " + QString::number(Pose_1->pose.position.y, 'f', 4) + "\n" +
    	 		       "Z: " + QString::number(Pose_1->pose.position.z, 'f', 4));
            
            for (int i = 0; i < numPoints; ++i) {
                angle = qDegreesToRadians(angleStep * i);
                x = Pose_1->pose.position.x + setDistance * cos(angle);
                y = Pose_1->pose.position.y + setDistance * sin(angle);
                circlePoints << QVector3D(x, Pose_1->pose.position.z, y); 
            }

            data1 << QVector3D(Pose_1->pose.position.x, Pose_1->pose.position.z, Pose_1->pose.position.y);
            ui->scatter->seriesList().at(0)->dataProxy()->resetArray(new QtDataVisualization::QScatterDataArray(data1));
            ui->scatter->seriesList().at(2)->dataProxy()->resetArray(new QtDataVisualization::QScatterDataArray(circlePoints));


        }
    	if(Vel_1){ 		       
   	        ui->label_leader_vel->setText("X: " + QString::number(Vel_1->twist.linear.x, 'f', 4) + "\n" +
 		       	       "Y: " + QString::number(Vel_1->twist.linear.y, 'f', 4) + "\n" +
 		       	       "Z: " + QString::number(Vel_1->twist.linear.z, 'f', 4));

            Vel_bar_leader = {Vel_1->twist.linear.x, Vel_1->twist.linear.y, Vel_1->twist.linear.z};
         }
               
    } else{
        ui->leaderStatusLabel->setText("Leader Mavlink: Disconnected");
        ui->leaderStatusLabel->setStyleSheet("background-color: red; padding: 5px;");
    }

    if (received_follower_pose || received_follower_vel) {
        ui->followerStatusLabel->setText("Follower Mavlink: Connected");
        ui->followerStatusLabel->setStyleSheet("background-color: green; padding: 5px;");

        if(Pose_2){
    	    ui->label_follower_pos->setText("Position  X: " + QString::number(Pose_2->pose.position.x, 'f', 4) + "\n" +
    	 		       "	Y: " + QString::number(Pose_2->pose.position.y, 'f', 4) + "\n" +
    	 		       "	Z: " + QString::number(Pose_2->pose.position.z, 'f', 4)); 
            data2 << QVector3D(Pose_2->pose.position.x, Pose_2->pose.position.z, Pose_2->pose.position.y);
            ui->scatter->seriesList().at(1)->dataProxy()->resetArray(new QtDataVisualization::QScatterDataArray(data2));
        }
        if(Vel_2){ 		       
   	        ui->label_follower_vel->setText("Velocity  X: " + QString::number(Vel_2->twist.linear.x,'f', 4) + "\n" +
    	 		       "	Y: " + QString::number(Vel_2->twist.linear.y, 'f', 4) + "\n" +
    	 		       "	Z: " + QString::number(Vel_2->twist.linear.z, 'f', 4));
            Vel_bar_follower = {Vel_2->twist.linear.x, Vel_2->twist.linear.y, Vel_2->twist.linear.z};
        }
    

    }else {
        ui->followerStatusLabel->setText("Follower Mavlink: Disconnected");
        ui->followerStatusLabel->setStyleSheet("background-color: red; padding: 5px;");
    }

    updateBarGraphData(Vel_bar_leader, Vel_bar_follower);
    received_follower_pose = received_leader_pose = received_follower_vel = received_leader_vel = false;
}

void MainWindow::updateBarGraphData(const QVector<double> &drone1Data, const QVector<double> &drone2Data) {
   
    QBarSeries* series = dynamic_cast<QBarSeries*>(ui->chart->series().at(0));  
    if (series) {
        QBarSet* set0 = series->barSets().at(0);
        QBarSet* set1 = series->barSets().at(1);
        
        if (set0 && set1) {
            // Replace old data with new data
            for(int i = 0; i < drone1Data.size(); ++i) {
                set0->replace(i, drone1Data.at(i));
            }

            // Replace data in set1
            for(int i = 0; i < drone2Data.size(); ++i) {
                set1->replace(i, drone2Data.at(i));
            }
                
            ui->chart->update();
            
        }
    }
}
void MainWindow::Arm(auto drone_client)
{
    QMessageBox msgBox;
    msgBox.setWindowTitle("Drone arm issue");

    while (!drone_client->wait_for_service(std::chrono::seconds(1))) {
    
        msgBox.setText("Interrupted while waiting for the service.");
        msgBox.exec();
        return;
    }

    // Creating a request
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;  // true to arm the drone

    // Sending the request
    auto result_future = drone_client->async_send_request(request);
    // Wait for the result
    if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = result_future.get();
      if (result->success) {
        //RCLCPP_INFO(node->get_logger(), "Drone armed successfully!");
         statusBar()->showMessage("Drone armed successfully!", 4000);
      } else {
         msgBox.setText("Failed to arm drone");
         msgBox.exec();
        //RCLCPP_ERROR(node->get_logger(), "Failed to arm drone %s" , result->result_str.c_str());
      }
    } else {
        msgBox.setText("Failed to call service to arm drone");
        msgBox.exec();
    }
}

void MainWindow::TakeOff(auto drone_client, double i)
{ 
    QMessageBox msgBox(this);
    msgBox.setWindowTitle("Drone takeOff issue");
     while (!drone_client->wait_for_service(std::chrono::seconds(1))) {
     
        msgBox.setText("Interrupted while waiting for the service.");
        msgBox.exec();
        return;
      
    }

    // Creating a request with altitude
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();

    request->latitude = 47.3977517;
    request->longitude = 8.545594;
    request->altitude = 536.3339201551239 - 40 - i;

    // Sending the request
    auto result_future = drone_client->async_send_request(request);
    // Wait for the result
    if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = result_future.get();
      if (result->success) {
       statusBar()->showMessage("Takeoff command sent successfully!", 4000);
      } else {
        msgBox.setText("Failed to takeoff command");
        msgBox.exec();
      }
    } else {
       msgBox.setText("Failed to call service for take off");
       msgBox.exec();
    }
}



void MainWindow::offboardButton_follower() {
    //QMessageBox::warning(this, "Input Error", "Offboard mode issue");

   if (!set_mode_client_->wait_for_service(std::chrono::seconds(1))) { 
        QMessageBox::warning(this, "Input Error", "Offboard node in not available");
        return;
    }

    auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    set_mode_request->custom_mode = "OFFBOARD";
    auto set_mode_future = set_mode_client_->async_send_request(set_mode_request);

    if (rclcpp::spin_until_future_complete(node, set_mode_future) == 
        rclcpp::FutureReturnCode::SUCCESS)
            
    {
    statusBar()->showMessage("Offboard enabled", 4000);

    } else {
        QMessageBox::warning(this, "Input Error", "Service is not available");

        return;
    }
}

void MainWindow::controlButton_follower() {
    auto message = std_msgs::msg::Bool();
      
      if(!control){
      //ui->offboardButton_follower->setEnabled(true);
      ui->controlButton_follower->setStyleSheet(
		"QPushButton { background-color: green; }"
		"QPushButton:pressed { background-color: darkgreen; }"
		"QPushButton:hover { background-color: lightgreen; }");
      control = true;
      }
      else {
      //ui->offboardButton_follower->setEnabled(false);
      ui->controlButton_follower->setStyleSheet("");
      control=false;
      }
      
      
    message.data = control;
    control_publisher_->publish(message);
}

void MainWindow::takeoffButton_follower() {
    TakeOff(takeoff_client_follower, 3);
}
void MainWindow::armButton_leader() {
    Arm(arming_client_leader_);
    
}

void MainWindow::takeoffButton_leader() {
    TakeOff(takeoff_client_leader, -1);
}

void MainWindow::armButton_follower() {
    Arm(arming_client_follower_);

}

void MainWindow::publisControlDistance(){

    bool isDouble;
    double number = ui->numberInput->text().toDouble(&isDouble);

    if (!isDouble) {
        // If the conversion fails, show a message box and don't publish
        QMessageBox::warning(this, "Input Error", "Please enter a valid number.");
    } else {
        setDistance = number;
        ui->numberInput->setPlaceholderText(QString::number(setDistance));
        // If the conversion is successful, publish the number
        std_msgs::msg::Float64 msg;
        msg.data = setDistance;
        distance_publisher_->publish(msg);
        statusBar()->showMessage("Distance updated to " + QString::number(setDistance), 4000);
    }

}    


void MainWindow::cameraButtonClicked() {
    /*if(currentImage.empty()){
        QMessageBox::warning(this, "Input Error", "There is no camera stream");
    }
    else if (!opencvImageWindow) { // Only create a new window if it doesn't exist
        opencvImageWindow = new QWidget();
        opencvImageWindow->setAttribute(Qt::WA_DeleteOnClose); // Important
        opencvImageWindow->setWindowTitle("Camera Image");

        QVBoxLayout *layout = new QVBoxLayout(opencvImageWindow);
        QLabel *imageLabel = new QLabel();
        imageLabel->setObjectName("ImageLabel"); // Set object name for later retrieval
        imageLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        layout->addWidget(imageLabel);

        connect(opencvImageWindow, &QWidget::destroyed, this, [this]() {
            opencvImageWindow = nullptr; // Reset pointer when window is closed
        });

        opencvImageWindow->show();

        QTimer *timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, &MainWindow::updateCameraWindow);
        timer->start(100); 
    }*/
}

void MainWindow::updateCameraWindow() {
    /*if (!opencvImageWindow || currentImage.empty()) return;
    std::lock_guard<std::mutex> lock(imageMutex);

    QImage qimg = MatToQImage(currentImage); // Convert cv::Mat to QImage
    QLabel *imageLabel = opencvImageWindow->findChild<QLabel *>("ImageLabel");
    if (imageLabel) {
        imageLabel->setPixmap(QPixmap::fromImage(qimg));
    }*/
}

QImage MainWindow::MatToQImage(const cv::Mat& mat) {
    /*if (mat.channels() == 3) {
        return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888).rgbSwapped();
    } else if (mat.channels() == 1) {
        return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
    } else {
        return QImage();
    }*/
}


