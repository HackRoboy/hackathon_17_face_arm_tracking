/**
 * @file /include/interface/main_window.hpp
 *
 * @brief Qt based gui for interface.
 *
 * @date November 2010
 **/
#ifndef interface_MAIN_WINDOW_H
#define interface_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <QtGui/QMainWindow>
#include <QFileSystemModel>
#include "ui_main_window.h"
#include "roboy_managing_node/myoMaster.hpp"
#include <tinyxml.h>
#include <fstream>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/JointState.h>

#define RUN_IN_THREAD
#define NUMBER_OF_FPGAS 5
#define NUMBER_OF_MOTORS_PER_FPGA 14

#endif

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace interface {

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    MyoMaster *myoMaster;
private:
    void MotorStatus(const communication::MotorStatus::ConstPtr &msg);
    void MotorRecordPack(const communication::MotorRecord::ConstPtr &msg);
    void calculateMovement(float, float, float, int&, int&);
    void positionCallback(const geometry_msgs::Point32::ConstPtr &msg);
    void jointStatesCallback(const sensor_msgs::JointState msg);
//    void moveArm(float, float, float);

public Q_SLOTS:
	void on_actionAbout_triggered();
    void updateSetPoints(int percent);
    void updateSetPointsAll(int percent);
	void updateControllerParams();
    void movementPathChanged();
    void recordMovement();
    void plotData(int id);
    bool playMovement();
    void stopMovement();
    void rewindMovement();
    void pauseMovement();
    void loopMovement();
    void stopButtonClicked();
    void moveArm(float, float, float);
Q_SIGNALS:
    void newData(int id);
private:
	Ui::MainWindowDesign ui;
    ros::NodeHandlePtr nh;
    ros::Publisher motorConfig, motorRecordConfig, motorTrajectory, motorTrajectoryControl;
    ros::Subscriber motorStatus, motorRecord, positionSubscriber, jointStatesSubscriber;
    
    QVector<double> time;
    QVector<double> motorData[NUMBER_OF_FPGAS][NUMBER_OF_MOTORS_PER_FPGA][4];
    long int counter = 0;
    int samples_per_plot = 300;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
	QColor color_pallette[14] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
							   Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray};
    QFileSystemModel *model;
    int numberOfRecordsToWaitFor = 0;
    map<int, vector<int32_t>[NUMBER_OF_MOTORS_PER_FPGA]> records;
    unsigned long long ullPlotCounter = 0;
};

}  // namespace interface

#endif // interface_MAIN_WINDOW_H
