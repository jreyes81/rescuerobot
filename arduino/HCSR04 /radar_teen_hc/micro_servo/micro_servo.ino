/*
* 180 micro servo 
* Modified By: Amr Wanly and Jeovanny Reyes
* Modified to work on ROS
* Raytheon Radar Guided Rescue Robot
* Cal State LA
*
* Input: None
* Output: None
*
* Publisher: pub_range publishes to "HerculesUltrasound_Range" topic --> Distance of Object
*            servopos pubslishes to "HerculesUltrasound_Position" topic --> Position of servo attached to ultrasound
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
#include <std_msgs/Float32.h>

#include <Servo.h> 

ros::NodeHandle nh;

std_msgs::Float32 str_msg; // This creates the message type "Float 32"
ros::Publisher servopos("/HerculesUltrasound_Position", &str_msg); 
 

int servofeed = A14; // analog pin
int servo_pos = 0; // position placeholder for servo
int temp = 0; //variable that will store the analog feedback signal

Servo myServo; // Creates a servo object for controlling the servo motor


void setup() {

  nh.initNode();
  nh.advertise(servopos);


  //Serial.begin(9600);
  myServo.attach(5); // Defines on which pin is the servo motor attached
}

void loop() {
  
  // rotates the servo motor from 15 to 165 degrees
  for(int i=15;i<=165;i++){   
  myServo.write(i);
  
  temp = analogRead(servofeed);

  servo_pos = ((0.3956*temp)-75.165); // Converts feedback counts to angles.

  
  str_msg.data = servo_pos; // Servo position publishing
  servopos.publish( &str_msg);
  
  delay(30);
  }
  delay(100); // Stops the servo before turning the other way
  
  // Repeats the previous lines from 165 to 15 degrees
  for(int i=165;i>15;i--){  
  myServo.write(i);
  
 temp = analogRead(servofeed);

  servo_pos = ((0.3956*temp)-75.165);

  
  str_msg.data = servo_pos; // Servo position publishing
  servopos.publish( &str_msg);
  
  delay(30);
  }
  delay(100); // Stops the servo before turning the other way
  
  nh.spinOnce();
  //delay(1);

}

