/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/interface/interface/main_window.hpp"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/JointState.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace interface {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

    QObject::connect(this, SIGNAL(newData(int)), this, SLOT(plotData(int)));

    QObject::connect(ui.motor0, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor1, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor2, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor3, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor4, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor5, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor6, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor7, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor8, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor9, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor10, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor11, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor12, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor13, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.allMotors, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsAll(int)));
    QObject::connect(ui.updateController, SIGNAL(clicked()), this, SLOT(updateControllerParams()));
    QObject::connect(ui.record, SIGNAL(clicked()), this, SLOT(recordMovement()));
    QObject::connect(ui.play, SIGNAL(clicked()), this, SLOT(playMovement()));
    QObject::connect(ui.stop, SIGNAL(clicked()), this, SLOT(stopMovement()));
    QObject::connect(ui.rewind, SIGNAL(clicked()), this, SLOT(rewindMovement()));
    QObject::connect(ui.pause, SIGNAL(clicked()), this, SLOT(pauseMovement()));
    QObject::connect(ui.loop, SIGNAL(clicked()), this, SLOT(loopMovement()));
    QObject::connect(ui.stop_button, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
    ui.stop_button->setStyleSheet("background-color: red");

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "interface",
                  ros::init_options::NoSigintHandler |
                          ros::init_options::AnonymousName|
                          ros::init_options::NoRosout);
    }
    motorStatus = nh->subscribe("/roboy/MotorStatus", 1, &MainWindow::MotorStatus, this);
    motorConfig = nh->advertise<communication::MotorConfig>("/roboy/MotorConfig", 1);
    motorRecordConfig = nh->advertise<communication::MotorRecordConfig>("/roboy/MotorRecordConfig", 1);
    motorRecord = nh->subscribe("/roboy/MotorRecord", 100, &MainWindow::MotorRecordPack, this);
    motorTrajectory = nh->advertise<communication::MotorRecord>("/roboy/MotorTrajectory", 1);
    motorTrajectoryControl = nh->advertise<communication::MotorTrajectoryControl>("/roboy/MotorTrajectoryControl", 1);

    //subscribe to the baloon position
    positionSubscriber = nh->subscribe("/balloonPosition", 1 ,&MainWindow::positionCallback, this);
    jointStatesSubscriber = nh->subscribe("/jStates", 1, &MainWindow::jointStatesCallback, this);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    for(uint motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++) {
        ui.position_plot0->addGraph();
        ui.position_plot0->graph(motor)->setPen(QPen(color_pallette[motor]));
        ui.velocity_plot0->addGraph();
        ui.velocity_plot0->graph(motor)->setPen(QPen(color_pallette[motor]));
        ui.displacement_plot0->addGraph();
        ui.displacement_plot0->graph(motor)->setPen(QPen(color_pallette[motor]));
        ui.current_plot0->addGraph();
        ui.current_plot0->graph(motor)->setPen(QPen(color_pallette[motor]));
    }
    ui.position_plot0->xAxis->setLabel("x");
    ui.position_plot0->yAxis->setLabel("ticks");
    ui.position_plot0->replot();

    ui.velocity_plot0->xAxis->setLabel("x");
    ui.velocity_plot0->yAxis->setLabel("ticks/s");
    ui.velocity_plot0->replot();

    ui.displacement_plot0->xAxis->setLabel("x");
    ui.displacement_plot0->yAxis->setLabel("ticks");
    ui.displacement_plot0->replot();

    ui.current_plot0->xAxis->setLabel("x");
    ui.current_plot0->yAxis->setLabel("mA");
    ui.current_plot0->replot();

    updateControllerParams();

    model = new QFileSystemModel;
    movementPathChanged();

    ullPlotCounter = 0;
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

void MainWindow::MotorStatus(const communication::MotorStatus::ConstPtr &msg){
    ROS_INFO_THROTTLE(5, "receiving motor status");
    ROS_INFO_THROTTLE(5, "position %d velocity %d displacement %d current %d", msg->position[0], msg->velocity[0], msg->displacement[0], msg->current[0]);
    ROS_INFO_THROTTLE(5, "position %d velocity %d displacement %d current %d", msg->position[1], msg->velocity[1], msg->displacement[1], msg->current[1]);

    time.push_back(counter++);
    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        motorData[msg->id][motor][0].push_back(msg->position[motor]);
        motorData[msg->id][motor][1].push_back(msg->velocity[motor]);
        motorData[msg->id][motor][2].push_back(msg->displacement[motor]);
        motorData[msg->id][motor][3].push_back(msg->current[motor]);
        if(motorData[msg->id][motor][0].size()>samples_per_plot){
            motorData[msg->id][motor][0].pop_front();
            motorData[msg->id][motor][1].pop_front();
            motorData[msg->id][motor][2].pop_front();
            motorData[msg->id][motor][3].pop_front();
        }
    }
    if(time.size()>samples_per_plot)
        time.pop_front();
    Q_EMIT newData(msg->id);
}

void MainWindow::MotorRecordPack(const communication::MotorRecord::ConstPtr &msg){
    numberOfRecordsToWaitFor--;
    records[msg->id][0] = msg->motor0;
    records[msg->id][1] = msg->motor1;
    records[msg->id][2] = msg->motor2;
    records[msg->id][3] = msg->motor3;
    records[msg->id][4] = msg->motor4;
    records[msg->id][5] = msg->motor5;
    records[msg->id][6] = msg->motor6;
    records[msg->id][7] = msg->motor7;
    records[msg->id][8] = msg->motor8;
    records[msg->id][9] = msg->motor9;
    records[msg->id][10] = msg->motor10;
    records[msg->id][11] = msg->motor11;
    records[msg->id][12] = msg->motor12;
    records[msg->id][13] = msg->motor13;
    ROS_INFO("received record from %d of length %ld with average sampling time %f ms",
    msg->id, msg->motor0.size(), msg->recordTime/msg->motor0.size()*1000.0f);
    if(numberOfRecordsToWaitFor==0){
        ROS_INFO("all records received");
        for(auto const& rec:records) {
            std::ofstream outfile;
            outfile.open(ui.movementName->text().toStdString().c_str());
            if (outfile.is_open()) {
                outfile << "<?xml version=\"1.0\" ?>"
                        << std::endl;
                outfile << "<roboy_trajectory control_mode=\""
                        << ui.control_mode->text().toStdString() << "\" samplingTime=\""
                        << atoi(ui.samplingTime->text().toStdString().c_str()) << "\">" << endl;
                for(uint motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++) {
                    outfile << "<trajectory motorid=\"" << motor << "\">"
                            << std::endl;
                    for (uint i = 0; i < rec.second[motor].size(); i++)
                        outfile << rec.second[motor][i] << " ";
                    outfile << "</trajectory>" << std::endl;
                }
                outfile << "</roboy_trajectory>" << endl;
                outfile.close();
            }
        }
    }
}


void MainWindow::positionCallback(const geometry_msgs::Point32::ConstPtr& msg)
{
  ROS_INFO("Face detected at azimuth, elevation and distance of %f, %f, %f", msg->x, msg->y, msg->z);
}

void MainWindow::jointStatesCallback(const sensor_msgs::JointState msg)
{
  ROS_INFO("Received joints");
  ROS_INFO("Joint 0: name %s position %f velocity %f effort %f ", msg.name[0].c_str(), msg.position[0], msg.velocity[0], msg.effort[0]);
  ROS_INFO("Joint 1: name %s position %f velocity %f effort %f ", msg.name[1].c_str(), msg.position[1], msg.velocity[1], msg.effort[1]);

 std::lock_guard<std::mutex> lock(myoMaster->mux);


 int nMotor1 = 0, nMotor2 = 0;

//to update the position
 nMotor1 = (int)motorData[0][0][0][0] + (int)msg.position[0]*3000; //motorData[motorData.size()-1][0][2] + msg.position[0];
 nMotor2 = (int)motorData[0][2][0][0] + (int)msg.position[1]*3000; //motorData[motorData.size()-1][2][2] + msg.position[1];


 if(nMotor1 < 0) nMotor1 = 0;
 if(nMotor2 < 0) nMotor2 = 0;
 std::cout << "Set the motors to " << nMotor1 << " " << nMotor2 << std::endl;
 myoMaster->changeSetPoint(0, nMotor1);
 myoMaster->changeSetPoint(2, nMotor2);

 ui.setPoint_motor0->setText(QString::number(nMotor1));
 ui.setPoint_motor2->setText(QString::number(nMotor2));

}


void MainWindow::calculateMovement(float x, float y, float z, int &nMotor1, int &nMotor2)
{
   nMotor1 = 1;
   nMotor2 = -1;
}

void MainWindow::moveArm(float x, float y, float z)
  {
  int nMotor1 = 0, nMotor2 = 0;
  calculateMovement(x, y, z, nMotor1, nMotor2);
//set the motor values
 
  std::lock_guard<std::mutex> lock(myoMaster->mux);

  myoMaster->changeSetPoint(0, nMotor1);
  myoMaster->changeSetPoint(2, nMotor2);

  }


void MainWindow::plotData(int id) {

ullPlotCounter++;
//readout every 50ms, plot every 5 seconds
if(ullPlotCounter % 100 != 0)
    return; //DON'T PLOT!

//else normal plotting

    switch(id){
        case 0:
            for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
                ui.position_plot0->graph(motor)->setData(time, motorData[id][motor][0]);
                ui.velocity_plot0->graph(motor)->setData(time, motorData[id][motor][1]);
                ui.displacement_plot0->graph(motor)->setData(time, motorData[id][motor][2]);
                ui.current_plot0->graph(motor)->setData(time, motorData[id][motor][3]);

                if (motor == 0) {
                    ui.position_plot0->graph(motor)->rescaleAxes();
                    ui.velocity_plot0->graph(motor)->rescaleAxes();
                    ui.displacement_plot0->graph(motor)->rescaleAxes();
                    ui.current_plot0->graph(motor)->rescaleAxes();
                } else {
                    ui.position_plot0->graph(motor)->rescaleAxes(true);
                    ui.velocity_plot0->graph(motor)->rescaleAxes(true);
                    ui.displacement_plot0->graph(motor)->rescaleAxes(true);
                    ui.current_plot0->graph(motor)->rescaleAxes(true);
                }
            }
            ui.position_plot0->replot();
            ui.velocity_plot0->replot();
            ui.displacement_plot0->replot();
            ui.current_plot0->replot();
            break;
        case 1:
            for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
                ui.position_plot1->graph(motor)->setData(time, motorData[id][motor][0]);
                ui.velocity_plot1->graph(motor)->setData(time, motorData[id][motor][1]);
                ui.displacement_plot1->graph(motor)->setData(time, motorData[id][motor][2]);
                ui.current_plot1->graph(motor)->setData(time, motorData[id][motor][3]);

                if (motor == 0) {
                    ui.position_plot1->graph(motor)->rescaleAxes();
                    ui.velocity_plot1->graph(motor)->rescaleAxes();
                    ui.displacement_plot1->graph(motor)->rescaleAxes();
                    ui.current_plot1->graph(motor)->rescaleAxes();
                } else {
                    ui.position_plot1->graph(motor)->rescaleAxes(true);
                    ui.velocity_plot1->graph(motor)->rescaleAxes(true);
                    ui.displacement_plot1->graph(motor)->rescaleAxes(true);
                    ui.current_plot1->graph(motor)->rescaleAxes(true);
                }
            }
            ui.position_plot1->replot();
            ui.velocity_plot1->replot();
            ui.displacement_plot1->replot();
            ui.current_plot1->replot();
            break;
        case 2:
            for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
                ui.position_plot2->graph(motor)->setData(time, motorData[id][motor][0]);
                ui.velocity_plot2->graph(motor)->setData(time, motorData[id][motor][1]);
                ui.displacement_plot2->graph(motor)->setData(time, motorData[id][motor][2]);
                ui.current_plot2->graph(motor)->setData(time, motorData[id][motor][3]);

                if (motor == 0) {
                    ui.position_plot2->graph(motor)->rescaleAxes();
                    ui.velocity_plot2->graph(motor)->rescaleAxes();
                    ui.displacement_plot2->graph(motor)->rescaleAxes();
                    ui.current_plot2->graph(motor)->rescaleAxes();
                } else {
                    ui.position_plot2->graph(motor)->rescaleAxes(true);
                    ui.velocity_plot2->graph(motor)->rescaleAxes(true);
                    ui.displacement_plot2->graph(motor)->rescaleAxes(true);
                    ui.current_plot2->graph(motor)->rescaleAxes(true);
                }
            }
            ui.position_plot2->replot();
            ui.velocity_plot2->replot();
            ui.displacement_plot2->replot();
            ui.current_plot2->replot();
            break;
        case 3:
            for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
                ui.position_plot3->graph(motor)->setData(time, motorData[id][motor][0]);
                ui.velocity_plot3->graph(motor)->setData(time, motorData[id][motor][1]);
                ui.displacement_plot3->graph(motor)->setData(time, motorData[id][motor][2]);
                ui.current_plot3->graph(motor)->setData(time, motorData[id][motor][3]);

                if (motor == 0) {
                    ui.position_plot3->graph(motor)->rescaleAxes();
                    ui.velocity_plot3->graph(motor)->rescaleAxes();
                    ui.displacement_plot3->graph(motor)->rescaleAxes();
                    ui.current_plot3->graph(motor)->rescaleAxes();
                } else {
                    ui.position_plot3->graph(motor)->rescaleAxes(true);
                    ui.velocity_plot3->graph(motor)->rescaleAxes(true);
                    ui.displacement_plot3->graph(motor)->rescaleAxes(true);
                    ui.current_plot3->graph(motor)->rescaleAxes(true);
                }
            }
            ui.position_plot3->replot();
            ui.velocity_plot3->replot();
            ui.displacement_plot3->replot();
            ui.current_plot3->replot();
            break;
        case 4:
            for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
                ui.position_plot4->graph(motor)->setData(time, motorData[id][motor][0]);
                ui.velocity_plot4->graph(motor)->setData(time, motorData[id][motor][1]);
                ui.displacement_plot4->graph(motor)->setData(time, motorData[id][motor][2]);
                ui.current_plot4->graph(motor)->setData(time, motorData[id][motor][3]);

                if (motor == 0) {
                    ui.position_plot4->graph(motor)->rescaleAxes();
                    ui.velocity_plot4->graph(motor)->rescaleAxes();
                    ui.displacement_plot4->graph(motor)->rescaleAxes();
                    ui.current_plot4->graph(motor)->rescaleAxes();
                } else {
                    ui.position_plot4->graph(motor)->rescaleAxes(true);
                    ui.velocity_plot4->graph(motor)->rescaleAxes(true);
                    ui.displacement_plot4->graph(motor)->rescaleAxes(true);
                    ui.current_plot4->graph(motor)->rescaleAxes(true);
                }
            }
            ui.position_plot4->replot();
            ui.velocity_plot4->replot();
            ui.displacement_plot4->replot();
            ui.current_plot4->replot();
            break;
    }

}

bool MainWindow::playMovement(){
    // initialize TiXmlDocument doc with a string
    QModelIndexList indexList = ui.movementFolder->selectionModel()->selectedIndexes();
    ROS_INFO_STREAM("loading trajectory " << model->filePath(indexList[0]).toStdString().c_str());
    TiXmlDocument doc(model->filePath(indexList[0]).toStdString().c_str());
    if (!doc.LoadFile()) {
        return false;
    }

    TiXmlElement *root = doc.RootElement();

    int numberOfSamples, samplingTime, control_mode;

    root->QueryIntAttribute("control_mode", &control_mode);
    root->QueryIntAttribute("samplingTime",&samplingTime);
    ROS_INFO("recognized roboy_trajectory in control_mode %d with samplingTime %d", control_mode, samplingTime);

    communication::MotorRecord msg;

    // Constructs the myoMuscles by parsing custom xml.
    TiXmlElement *trajectory_it = NULL;
    for (trajectory_it = root->FirstChildElement("trajectory"); trajectory_it;
         trajectory_it = trajectory_it->NextSiblingElement("trajectory")) {
        int motor;
        if (trajectory_it->QueryIntAttribute("motorid", &motor)==TIXML_SUCCESS) {

            stringstream stream(trajectory_it->GetText());
            while (1) {
                int n;
                stream >> n;
                switch (motor) {
                    case 0:
                        msg.motor0.push_back(n);
                        break;
                    case 1:
                        msg.motor1.push_back(n);
                        break;
                    case 2:
                        msg.motor2.push_back(n);
                        break;
                    case 3:
                        msg.motor3.push_back(n);
                        break;
                    case 4:
                        msg.motor4.push_back(n);
                        break;
                    case 5:
                        msg.motor5.push_back(n);
                        break;
                    case 6:
                        msg.motor6.push_back(n);
                        break;
                    case 7:
                        msg.motor7.push_back(n);
                        break;
                    case 8:
                        msg.motor8.push_back(n);
                        break;
                    case 9:
                        msg.motor9.push_back(n);
                        break;
                    case 10:
                        msg.motor10.push_back(n);
                        break;
                    case 11:
                        msg.motor11.push_back(n);
                        break;
                    case 12:
                        msg.motor12.push_back(n);
                        break;
                    case 13:
                        msg.motor13.push_back(n);
                        break;
                    default:
                        ROS_ERROR("motorid %d is not available, aborting. check you recorded trajectory", motor);
                        return false;
                }

                if (!stream) {
                    numberOfSamples = msg.motor0.size();
                    break;
                }
            }
        }else{
            ROS_ERROR("trajectory without motorid, aborting. check you trajectory file");
            return false;
        }
    }

    msg.samplingTime = samplingTime;
    msg.control_mode = control_mode;

    motorTrajectory.publish(msg);
    return true;
}

void MainWindow::stopMovement(){
    ROS_INFO("stop movement");
    communication::MotorTrajectoryControl msg;
    msg.play = false;
    msg.pause = ui.pause->isChecked();
    msg.rewind = false;
    msg.loop = ui.loop->isChecked();
    motorTrajectoryControl.publish(msg);
}

void MainWindow::rewindMovement(){
    ROS_INFO("rewind movement");
    communication::MotorTrajectoryControl msg;
    msg.play = true;
    msg.pause = ui.pause->isChecked();
    msg.rewind = true;
    msg.loop = ui.loop->isChecked();
    motorTrajectoryControl.publish(msg);
}

void MainWindow::pauseMovement(){
    ROS_INFO("pause movement");
    communication::MotorTrajectoryControl msg;
    msg.play = true;
    msg.pause = ui.pause->isChecked();
    msg.rewind = false;
    msg.loop = ui.loop->isChecked();
    motorTrajectoryControl.publish(msg);
}

void MainWindow::loopMovement(){
    ROS_INFO("loop movement");
    communication::MotorTrajectoryControl msg;
    msg.play = true;
    msg.pause = ui.pause->isChecked();
    msg.rewind = false;
    msg.loop = ui.loop->isChecked();
    motorTrajectoryControl.publish(msg);
}

void MainWindow::stopButtonClicked(){
    ROS_INFO("stop button clicked");
    if(!ui.stop_button->isChecked()) {
        updateControllerParams();
    }else{ // set controller gains to zero
        communication::MotorConfig msg;
        for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
            msg.motors.push_back(motor);
            msg.control_mode.push_back(2);
            msg.outputPosMax.push_back(1000); // pwm max
            msg.outputNegMax.push_back(-1000); // pwm min
            msg.spPosMax.push_back(100000000);
            msg.spNegMax.push_back(-100000000);
            msg.IntegralPosMax.push_back(100);
            msg.IntegralNegMax.push_back(-100);
            msg.Kp.push_back(0);
            msg.Ki.push_back(0);
            msg.Kd.push_back(0);
            msg.forwardGain.push_back(0);
            msg.deadBand.push_back(0);
        }
        motorConfig.publish(msg);
    }
}

void MainWindow::updateSetPoints(int percent){
    std::lock_guard<std::mutex> lock(myoMaster->mux);
    int setpoints[NUMBER_OF_MOTORS_PER_FPGA];
    switch(ui.control_mode->value()){
        case POSITION:
            setpoints[0] = ui.motor0->value()*3000;
            setpoints[1] = ui.motor1->value()*3000;
            setpoints[2] = ui.motor2->value()*3000;
            setpoints[3] = ui.motor3->value()*3000;
            setpoints[4] = ui.motor4->value()*3000;
            setpoints[5] = ui.motor5->value()*3000;
            setpoints[6] = ui.motor6->value()*3000;
            setpoints[7] = ui.motor7->value()*3000;
            setpoints[8] = ui.motor8->value()*3000;
            setpoints[9] = ui.motor9->value()*3000;
            setpoints[10] = ui.motor10->value()*3000;
            setpoints[11] = ui.motor11->value()*3000;
            setpoints[12] = ui.motor12->value()*3000;
            setpoints[13] = ui.motor13->value()*3000;
            std::cout << "Set position" << std::endl;
            break;
        case VELOCITY:
            setpoints[0] = ui.motor0->value();
            setpoints[1] = ui.motor1->value();
            setpoints[2] = ui.motor2->value();
            setpoints[3] = ui.motor3->value();
            setpoints[4] = ui.motor4->value();
            setpoints[5] = ui.motor5->value();
            setpoints[6] = ui.motor6->value();
            setpoints[7] = ui.motor7->value();
            setpoints[8] = ui.motor8->value();
            setpoints[9] = ui.motor9->value();
            setpoints[10] = ui.motor10->value();
            setpoints[11] = ui.motor11->value();
            setpoints[12] = ui.motor12->value();
            setpoints[13] = ui.motor13->value();
            std::cout << "set velocity" << std::endl;
            break;
        case DISPLACEMENT:
            setpoints[0] = ui.motor0->value();
            setpoints[1] = ui.motor1->value();
            setpoints[2] = ui.motor2->value();
            setpoints[3] = ui.motor3->value();
            setpoints[4] = ui.motor4->value();
            setpoints[5] = ui.motor5->value();
            setpoints[6] = ui.motor6->value();
            setpoints[7] = ui.motor7->value();
            setpoints[8] = ui.motor8->value();
            setpoints[9] = ui.motor9->value();
            setpoints[10] = ui.motor10->value();
            setpoints[11] = ui.motor11->value();
            setpoints[12] = ui.motor12->value();
            setpoints[13] = ui.motor13->value();
            std::cout << "Set displacement" << std::endl;
            break;
    }
    for(int motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++){
        myoMaster->changeSetPoint(motor,setpoints[motor]);
    }
    ui.setPoint_motor0->setText(QString::number(setpoints[0]));
    ui.setPoint_motor1->setText(QString::number(setpoints[1]));
    ui.setPoint_motor2->setText(QString::number(setpoints[2]));
    ui.setPoint_motor3->setText(QString::number(setpoints[3]));
    ui.setPoint_motor4->setText(QString::number(setpoints[4]));
    ui.setPoint_motor5->setText(QString::number(setpoints[5]));
    ui.setPoint_motor6->setText(QString::number(setpoints[6]));
    ui.setPoint_motor7->setText(QString::number(setpoints[7]));
    ui.setPoint_motor8->setText(QString::number(setpoints[8]));
    ui.setPoint_motor9->setText(QString::number(setpoints[9]));
    ui.setPoint_motor10->setText(QString::number(setpoints[10]));
    ui.setPoint_motor11->setText(QString::number(setpoints[11]));
    ui.setPoint_motor12->setText(QString::number(setpoints[12]));
    ui.setPoint_motor13->setText(QString::number(setpoints[13]));
}

void MainWindow::updateSetPointsAll(int percent){
    std::lock_guard<std::mutex> lock(myoMaster->mux);
    int setpoints[NUMBER_OF_MOTORS_PER_FPGA];
    switch(ui.control_mode->value()){
        case POSITION:
            setpoints[0] = ui.allMotors->value()*3000;
            setpoints[1] = ui.allMotors->value()*3000;
            setpoints[2] = ui.allMotors->value()*3000;
            setpoints[3] = ui.allMotors->value()*3000;
            setpoints[4] = ui.allMotors->value()*3000;
            setpoints[5] = ui.allMotors->value()*3000;
            setpoints[6] = ui.allMotors->value()*3000;
            setpoints[7] = ui.allMotors->value()*3000;
            setpoints[8] = ui.allMotors->value()*3000;
            setpoints[9] = ui.allMotors->value()*3000;
            setpoints[10] = ui.allMotors->value()*3000;
            setpoints[11] = ui.allMotors->value()*3000;
            setpoints[12] = ui.allMotors->value()*3000;
            setpoints[13] = ui.allMotors->value()*3000;
            break;
        case VELOCITY:
            setpoints[0] = ui.allMotors->value();
            setpoints[1] = ui.allMotors->value();
            setpoints[2] = ui.allMotors->value();
            setpoints[3] = ui.allMotors->value();
            setpoints[4] = ui.allMotors->value();
            setpoints[5] = ui.allMotors->value();
            setpoints[6] = ui.allMotors->value();
            setpoints[7] = ui.allMotors->value();
            setpoints[8] = ui.allMotors->value();
            setpoints[9] = ui.allMotors->value();
            setpoints[10] = ui.allMotors->value();
            setpoints[11] = ui.allMotors->value();
            setpoints[12] = ui.allMotors->value();
            setpoints[13] = ui.allMotors->value();
            break;
        case DISPLACEMENT:
            setpoints[0] = ui.allMotors->value();
            setpoints[1] = ui.allMotors->value();
            setpoints[2] = ui.allMotors->value();
            setpoints[3] = ui.allMotors->value();
            setpoints[4] = ui.allMotors->value();
            setpoints[5] = ui.allMotors->value();
            setpoints[6] = ui.allMotors->value();
            setpoints[7] = ui.allMotors->value();
            setpoints[8] = ui.allMotors->value();
            setpoints[9] = ui.allMotors->value();
            setpoints[10] = ui.allMotors->value();
            setpoints[11] = ui.allMotors->value();
            setpoints[12] = ui.allMotors->value();
            setpoints[13] = ui.allMotors->value();
            break;
    }
    for(int motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++){
        myoMaster->changeSetPoint(motor,setpoints[motor]);
    }
    ui.setPoint_motor0->setText(QString::number(setpoints[0]));
    ui.setPoint_motor1->setText(QString::number(setpoints[1]));
    ui.setPoint_motor2->setText(QString::number(setpoints[2]));
    ui.setPoint_motor3->setText(QString::number(setpoints[3]));
    ui.setPoint_motor4->setText(QString::number(setpoints[4]));
    ui.setPoint_motor5->setText(QString::number(setpoints[5]));
    ui.setPoint_motor6->setText(QString::number(setpoints[6]));
    ui.setPoint_motor7->setText(QString::number(setpoints[7]));
    ui.setPoint_motor8->setText(QString::number(setpoints[8]));
    ui.setPoint_motor9->setText(QString::number(setpoints[9]));
    ui.setPoint_motor10->setText(QString::number(setpoints[10]));
    ui.setPoint_motor11->setText(QString::number(setpoints[11]));
    ui.setPoint_motor12->setText(QString::number(setpoints[12]));
    ui.setPoint_motor13->setText(QString::number(setpoints[13]));
}

void MainWindow::updateControllerParams(){
    ui.stop_button->setChecked(false);
    communication::MotorConfig msg;
    for(uint motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++){
        msg.motors.push_back(motor);
        msg.control_mode.push_back(ui.control_mode->value());
        msg.outputPosMax.push_back(1000); // pwm max
        msg.outputNegMax.push_back(-1000); // pwm min
        msg.spPosMax.push_back(100000000);
        msg.spNegMax.push_back(-100000000);
        msg.IntegralPosMax.push_back(100);
        msg.IntegralNegMax.push_back(-100);
        msg.Kp.push_back(atoi(ui.Kp->text().toStdString().c_str()));
        msg.Ki.push_back(atoi(ui.Ki->text().toStdString().c_str()));
        msg.Kd.push_back(atoi(ui.Kd->text().toStdString().c_str()));
        msg.forwardGain.push_back(atoi(ui.forwardGain->text().toStdString().c_str()));
        msg.deadBand.push_back(atoi(ui.deadBand->text().toStdString().c_str()));
    }
    motorConfig.publish(msg);
}

void MainWindow::movementPathChanged(){
    model->setRootPath(QDir::currentPath());
    ui.movementFolder->setModel(model);
}

void MainWindow::recordMovement(){
    ROS_INFO("start recording");
    communication::MotorRecordConfig msg;
    msg.ids = 0;
    numberOfRecordsToWaitFor = 0;
    if(ui.record_fpga0->isChecked()) {
        msg.ids |= (1 << 0);
        numberOfRecordsToWaitFor++;
    }
    if(ui.record_fpga1->isChecked()){
        msg.ids |= (1<<1);
        numberOfRecordsToWaitFor++;
    }
    if(ui.record_fpga2->isChecked()){
        msg.ids |= (1<<2);
        numberOfRecordsToWaitFor++;
    }
    if(ui.record_fpga3->isChecked()){
        msg.ids |= (1<<3);
        numberOfRecordsToWaitFor++;
    }
    if(ui.record_fpga4->isChecked()){
        msg.ids |= (1<<4);
        numberOfRecordsToWaitFor++;
    }

    msg.control_mode = ui.control_mode->value();
    msg.samplingTime = atoi(ui.samplingTime->text().toStdString().c_str());
    msg.recordTime = atoi(ui.recordTime->text().toStdString().c_str());
    motorRecordConfig.publish(msg);
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
    moveArm(0.0, 0.0, 0.0);
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
//    QSettings settings("Qt-Ros Package", "interface");
//    restoreGeometry(settings.value("geometry").toByteArray());
//    restoreState(settings.value("windowState").toByteArray());
//    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
//    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
//    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
//    ui.line_edit_master->setText(master_url);
//    ui.line_edit_host->setText(host_url);
//    //ui.line_edit_topic->setText(topic_name);
//    bool remember = settings.value("remember_settings", false).toBool();
//    ui.checkbox_remember_settings->setChecked(remember);
//    bool checked = settings.value("use_environment_variables", false).toBool();
//    ui.checkbox_use_environment->setChecked(checked);
//    if ( checked ) {
//    	ui.line_edit_master->setEnabled(false);
//    	ui.line_edit_host->setEnabled(false);
//    	//ui.line_edit_topic->setEnabled(false);
//    }
}

void MainWindow::WriteSettings() {
//    QSettings settings("Qt-Ros Package", "interface");
//    settings.setValue("master_url",ui.line_edit_master->text());
//    settings.setValue("host_url",ui.line_edit_host->text());
//    //settings.setValue("topic_name",ui.line_edit_topic->text());
//    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
//    settings.setValue("geometry", saveGeometry());
//    settings.setValue("windowState", saveState());
//    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace interface

