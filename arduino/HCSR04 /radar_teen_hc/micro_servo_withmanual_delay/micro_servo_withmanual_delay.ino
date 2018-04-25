/*
* 180 micro servo 
* Modified By: Amr Wanly and Jeovanny Reyes
* Modified to work on ROS
* Raytheon Radar Guided Rescue Robot
* Cal State LA
*
* Input: None
* Output: "Float32" messages that contains position of micro servo
*
* Publisher:servopos pubslishes to "HerculesUltrasound_Position" topic --> Position of servo attached to ultrasound
* 
* Subscriber: None
*
* How to Run from Terminal: Run "roscore" 
*                           On a sperate terminal run "rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
*                           Note: _port may be different for other and baud rate parameter can be changed
*/

//#if (ARDUINO >= 100)
// #include <Arduino.h>
//#else
// #include <WProgram.h>
//#endif

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h> // Used for range messages
//#include <std_msgs/Int8.h> // Used for radar display states

#include <Servo.h> 

ros::NodeHandle nh;

std_msgs::Float32 str_msg; // This creates the message type "Float 32" 
 

int servofeed = A14; // analog pin
int servo_pos = 0; // position placeholder for servo
int temp = 0; //variable that will store the analog feedback signal
int start_pos = 15; // Starting position for servo
int sweep_state = 0; // 0 for radar still sweeping. 1 for stop sweeping

Servo myServo; // Creates a servo object for controlling the servo motor

ros::Publisher servopos("/HerculesUltrasound_Position", &str_msg);

void setup() {
  delay(2000);
  nh.initNode();
  nh.advertise(servopos); 

  myServo.attach(5); // Defines on which pin is the servo motor attached
}

void loop() {
  
  for(int i=start_pos;i<=165;i++)
  {
    myServo.write(i);
    delay(30); // Takes 8 seconds to sweep
    temp = analogRead(servofeed);
    servo_pos = ((0.3956*temp)-75.165); // Converts feedback counts to angles
    str_msg.data = servo_pos; // Servo position publishing
    servopos.publish( &str_msg);
    //delay(15); // 
  }
  delay(100); // Stops the servo before turning the other way

  // Repeats the previous lines from 165 to 15 degrees
  for(int i=165;i>start_pos;i--)
  {
    myServo.write(i);
    delay(30);
    temp = analogRead(servofeed);
    servo_pos = ((0.3956*temp)-75.165);
    str_msg.data = servo_pos; // Servo position publishing
    servopos.publish( &str_msg);
    //delay(15);
  }
  delay(100);
  
  nh.spinOnce();
}
  
