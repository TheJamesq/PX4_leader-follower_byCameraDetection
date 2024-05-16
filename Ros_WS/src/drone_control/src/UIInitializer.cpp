#include "UIInitializer.h"

UIInitializer::UIInitializer(QWidget *centralWidget) {
    QHBoxLayout *mainLayout = new QHBoxLayout(centralWidget); // Pass the centralWidget as parent to the layout

    initScatterSection(centralWidget);
    scatterWidget->setMinimumHeight(800);
    initMenuBar();

    // Create a new widget and vertical layout for the right side
    rightSideWidget = new QWidget(centralWidget);
    QVBoxLayout *rightSideLayout = new QVBoxLayout(rightSideWidget); // No need to pass rightSideWidget here

    // Initialize the label and control sections, which will be added to the right side layout
    initControlSection(rightSideWidget);
    initBarGraphSection(rightSideWidget);

    rightSideWidget->setMinimumHeight(800);
    rightSideWidget->setMaximumWidth(400);


    // Set the layout for the right side widget
    rightSideWidget->setLayout(rightSideLayout);

    // Add the scatter plot and right side widget to the main layout
    mainLayout->addWidget(scatterWidget);
    mainLayout->addWidget(rightSideWidget);

    // Set the stretch factors to control the proportions
    mainLayout->setStretch(0, 2); 
    mainLayout->setStretch(1, 1);
}

void UIInitializer::initScatterSection(QWidget *centralWidget) {
    scatterWidget = new QWidget(centralWidget);
    QVBoxLayout *scatterLayout = new QVBoxLayout(scatterWidget);

    scatter = new QtDataVisualization::Q3DScatter();
    scatter->setAspectRatio(1.0);
    scatter->setHorizontalAspectRatio(1.0);

    QWidget *scatterContainer = QWidget::createWindowContainer(scatter);
    scatterLayout->addWidget(scatterContainer);

    // Initialize a scatter series for leader drone
    auto series1 = new QtDataVisualization::QScatter3DSeries();
    series1->setItemSize(0.04f);
    series1->setBaseColor(QColor(Qt::red));
    scatter->addSeries(series1);

    // Initialize a scatter series for followerdrone
    auto series2 = new QtDataVisualization::QScatter3DSeries();
    series2->setItemSize(0.04f);
    series2->setBaseColor(QColor(Qt::blue));
    scatter->addSeries(series2);

    // Initialize a scatter series for control ring
    auto series3 = new QtDataVisualization::QScatter3DSeries();
    series3->setItemSize(0.006f);
    series3->setBaseColor(QColor(Qt::red));
    scatter->addSeries(series3);


    scatter->axisX()->setTitle("X Axis");
    scatter->axisY()->setTitle("Y Axis");
    scatter->axisZ()->setTitle("Z Axis");

    scatter->axisX()->setRange(-15, 15);
    scatter->axisY()->setRange(0, 30);
    scatter->axisZ()->setRange(-15,15);

    centralWidget->layout()->addWidget(scatterWidget);
}

void UIInitializer::initMenuBar(){

    menuBar = new QMenuBar();

    QWidget *rightWidget = new QWidget();
    QHBoxLayout *rightLayout = new QHBoxLayout(rightWidget);

    // Create a QLabel for the Mavlink status
    followerStatusLabel = new QLabel("Follower Mavlink: Disconnected");
    followerStatusLabel->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    followerStatusLabel->setStyleSheet("ba/ckground-color: red; padding: 5px;");
    rightLayout->addWidget(followerStatusLabel);

    // Create another QLabel for some other status
    leaderStatusLabel = new QLabel("Leader Mavlink: Disconnected");
    leaderStatusLabel->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    leaderStatusLabel->setStyleSheet("background-color: red; padding: 5px;");
    rightLayout->addWidget(leaderStatusLabel);

    //rightLayout->setContentsMargins(0, 10, 10, 0);

    menuBar->setCornerWidget(rightWidget, Qt::TopRightCorner);

    // Create a QPushButton
    cameraButton = new QPushButton("Camera");

    // Create a widget to hold the button and the label
    QWidget *leftWidget = new QWidget();
    QHBoxLayout *leftLayout = new QHBoxLayout(leftWidget);
    leftLayout->addWidget(cameraButton);
    //leftLayout->addWidget(leaderStatusLabel);
    leftWidget->setLayout(leftLayout);
    leftWidget->setContentsMargins(0, 0, 0, 0);

    // Set the custom widget on the left side of the menu bar
    menuBar->setCornerWidget(leftWidget, Qt::TopLeftCorner);
    
}

void UIInitializer::initLabelSection(QWidget *parentWidget) {
    labelWidget = new QWidget(parentWidget);
    QHBoxLayout *labelLayout = new QHBoxLayout(labelWidget);

    /*label_drone1 = new QLabel("Drone 1 X: uninitialized\nDrone 1 Y: uninitialized\nDrone 1 Z: uninitialized\n");
    label_drone2 = new QLabel("Drone 2 X: uninitialized\nDrone 2 Y: uninitialized\nDrone 2 Z: uninitialized\n");

    labelLayout->addWidget(label_drone1);
    labelLayout->addWidget(label_drone2);

    parentWidget->layout()->addWidget(labelWidget);*/
}

void UIInitializer::initBarGraphSection(QWidget *parentWidget) {
    chart = new QChart();
    QBarSet *set0 = new QBarSet("Follower Drone");
    QBarSet *set1 = new QBarSet("Leader Drone");

    *set0 << 0 << 0 << 0; 
    *set1 << 0 << 0 << 0; 

    set1->setBrush(QBrush(Qt::blue)); 
    set0->setBrush(QBrush(Qt::red)); 


    QBarSeries *series = new QBarSeries();
    series->append(set1);
    series->append(set0);

   
    chart->addSeries(series);
    chart->setTitle("Velocity");
    chart->setAnimationOptions(QChart::SeriesAnimations);

    QStringList categories;
    categories << "X" << "Y" << "Z";
    QBarCategoryAxis *axisX = new QBarCategoryAxis();
    axisX->append(categories);
    
    // Do not call createDefaultAxes when manually adding axes
    // chart->createDefaultAxes();

    // Set axis to the chart
    chart->addAxis(axisX, Qt::AlignBottom);
    series->attachAxis(axisX);

    QValueAxis *axisY = new QValueAxis();
    axisY->setRange(-7,7); // Adjust this to fit your data's range
    chart->addAxis(axisY, Qt::AlignLeft);
    series->attachAxis(axisY);
    series->setBarWidth(0.7);

    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    
    // Set a fixed height or maximum height for the chart
    //chartView->setMinimumHeight(180); // Or another value that suits your needs
    //chartView->setMaximumHeight(700); // Ensures the chart does not get taller than this

    QHBoxLayout *barLayout = new QHBoxLayout();
    barLayout->addWidget(chartView);

    barGraphWidget = new QWidget(parentWidget);
    barGraphWidget->setLayout(barLayout);

    parentWidget->layout()->addWidget(barGraphWidget);
}



void UIInitializer::initControlSection(QWidget *parentWidget) {
    QWidget *mainControlWidget = new QWidget(parentWidget);
    QHBoxLayout *mainControlLayout = new QHBoxLayout(mainControlWidget);

    controlWidget = new QWidget(parentWidget);
    QGridLayout *controlLayout = new QGridLayout(controlWidget);
    
    QLabel *section_follower = new QLabel("Follower Drone");
    section_follower->setStyleSheet("QLabel { font-size: 16pt; }");
    controlLayout->addWidget(section_follower, 0,0, Qt::AlignCenter);


    
    label_follower_pos = new QLabel("Position  X: uninitialized\n	Y: uninitialized\n	Z: uninitialized\n");
    controlLayout->addWidget(label_follower_pos, 1,0, Qt::AlignCenter);

    label_follower_vel = new QLabel("Velocity  X: uninitialized\n	Y: uninitialized\n	Z: uninitialized\n");
    controlLayout->addWidget(label_follower_vel, 2,0, Qt::AlignCenter);
    

    armButton_follower = new QPushButton("Arm drone");
    controlLayout->addWidget(armButton_follower, 3,0);
    
    takeoffButton_follower = new QPushButton("Takeoff");
    controlLayout->addWidget(takeoffButton_follower, 4,0);
    
     
    QWidget *SetControlWidget = new QWidget(controlWidget);
    QHBoxLayout *SetControlLayout = new QHBoxLayout(SetControlWidget);
    
    controlButton_follower = new QPushButton("Control");
    SetControlLayout->addWidget(controlButton_follower);
	
    offboardButton_follower = new QPushButton("Offboard");
    SetControlLayout->addWidget(offboardButton_follower);
    
    controlLayout->addWidget(SetControlWidget);

    
    section_follower = new QLabel("Leader Drone");
    section_follower->setStyleSheet("QLabel { font-size: 16pt; }");
    controlLayout->addWidget(section_follower, 0,1, Qt::AlignCenter);
    
    label_leader_pos = new QLabel("X: uninitialized\nY: uninitialized\nZ: uninitialized\n");
    controlLayout->addWidget(label_leader_pos, 1,1, Qt::AlignCenter);
    
    label_leader_vel = new QLabel("X: uninitialized\nY: uninitialized\nZ: uninitialized\n");
    controlLayout->addWidget(label_leader_vel, 2,1, Qt::AlignCenter);
    
    
   // secondControlWidget = new QWidget(mainControlWidget);
    //QVBoxLayout *secondControlLayout = new QVBoxLayout(secondControlWidget);
    
    armButton_leader = new QPushButton("Arm drone");
    controlLayout->addWidget(armButton_leader, 3,1);
    
     // Takeoff button
    takeoffButton_leader = new QPushButton("Takeoff");
    controlLayout->addWidget(takeoffButton_leader, 4,1);

    QWidget *SetdistanceWidget = new QWidget(controlWidget);
    QHBoxLayout *SetDistanceLayout = new QHBoxLayout(SetdistanceWidget);

    numberInput = new QLineEdit();
    numberInput->setPlaceholderText("1.5");
    numberInput->setFixedSize(75,23);
    SetDistanceLayout->addWidget(numberInput); 

    publishButton = new QPushButton("Publish");
    SetDistanceLayout->addWidget(publishButton); 

    controlLayout->addWidget(SetdistanceWidget, 5,1);

	
    mainControlLayout->addWidget(controlWidget, 1);

    //mainControlLayout->addWidget(secondControlWidget, 1);
    parentWidget->layout()->addWidget(mainControlWidget);
}



