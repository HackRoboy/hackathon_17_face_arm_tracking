#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/JointState.h"

#include <iostream>
#include <fstream>


ros::Publisher joints_pub;


void positionCallback(const geometry_msgs::Point32::ConstPtr& msg)
{
  ROS_INFO("Face detected at azimuth, elevation and distance  %f, %f, %f", msg->x, msg->y, msg->z);

  //now publish the joint states
  sensor_msgs::JointState jointMsg;
  jointMsg.name.push_back("joint0");
  jointMsg.name.push_back("joint1");

  int motor1Inc = 0, motor2Inc = 0;
  int elevation = msg->y;
  int azimuth = msg->x;

  if (elevation < 0) {
   // use motor 1
   motor1Inc = 4;
   motor2Inc = -4;
   
  } 
  else {
   // use motor 2
   motor1Inc = -4;
   motor2Inc = 4;
   }

   jointMsg.position.push_back(motor1Inc);  //increment relative to the current motor position
   jointMsg.position.push_back(motor2Inc);

   jointMsg.velocity.push_back(0.0);
   jointMsg.velocity.push_back(0.0);

   jointMsg.effort.push_back(0.0);
   jointMsg.effort.push_back(0.0);
//to move the servo motor
   joints_pub.publish(jointMsg);

}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  ros::Subscriber sub = n.subscribe("balloonPosition", 1000, positionCallback);

  joints_pub = n.advertise<sensor_msgs::JointState>("jStates", 1000);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
