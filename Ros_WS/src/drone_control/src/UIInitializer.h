#ifndef UIINITIALIZER_H
#define UIINITIALIZER_H


#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMenuBar>
#include <QLabel>
#include <QPushButton>
#include <QtDataVisualization/Q3DScatter>
#include <QtCharts>

using namespace QtCharts;

class UIInitializer {
public:
    UIInitializer(QWidget *centralWidget);
    QLabel *followerStatusLabel;
    QLabel *leaderStatusLabel;

    QPushButton *cameraButton;


    QLabel *label_follower_pos;
    QLabel *label_follower_vel;
    QLabel *label_leader_pos;
    QLabel *label_leader_vel;

    QtDataVisualization::Q3DScatter* scatter;
    QChart *chart;

    
    QPushButton *armButton_follower;
    QPushButton *offboardButton_follower;
    QPushButton *takeoffButton_follower;
    QPushButton *controlButton_follower;
    
    QPushButton *armButton_leader;
    QPushButton *takeoffButton_leader;
    QPushButton *publishButton;
    QLineEdit *numberInput;
    
    QWidget* scatterWidget;
    QWidget* labelWidget;
    QWidget* controlWidget;
    QWidget* barGraphWidget;
    QWidget *rightSideWidget;

    QMenuBar *menuBar;

private:
    void initScatterSection(QWidget *centralWidget);
    void initLabelSection(QWidget *parentWidget);
    void initBarGraphSection(QWidget *parentWidget);
    void initControlSection(QWidget *parentWidget);
    void initMenuBar();
};


#endif // UIINITIALIZER_H

