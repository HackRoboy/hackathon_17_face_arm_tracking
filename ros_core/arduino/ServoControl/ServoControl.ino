/* 
 * rosserial Subscriber that listens to the geometry_msgs on the balloonPosition topic
 * Controls the servo motor over an Genuino board
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point32.h>
#include <Servo.h>

ros::NodeHandle  nh;

Servo base;

void messageCb( const geometry_msgs::Point32& msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  float azimuth = msg.x;
  Serial.println(azimuth);
  if(azimuth < -1.0)
     {
      base.write(80); //clockwise
     }
  if(azimuth > 1.0)
     {
      base.write(100); //counterclockwise
     }

   delay(1000);
   base.write(92); //stop after 1 sec
 
}

ros::Subscriber<geometry_msgs::Point32> sub("balloonPosition", &messageCb );

void setup()
{ 
  base.attach(3);  
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  base.write(92); //stop the motor
  Serial.setTimeout(2000);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

