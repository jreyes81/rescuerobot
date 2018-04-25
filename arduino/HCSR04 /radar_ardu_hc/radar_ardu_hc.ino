/*
* HC-SR04 ultrasound sensor with 180 micro servo 
* Modified By: Jeovanny Reyes
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

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>

#include <Servo.h> 

ros::NodeHandle nh;

sensor_msgs::Range range_msg; // This creates the message type "Range"
std_msgs::Float32 str_msg; // This creates the message type "Float 32"
ros::Publisher pub_range( "/HerculesUltrasound_Range", &range_msg);
ros::Publisher servopos("/HerculesUltrasound_Position", &str_msg); 
 
// Defines Trig and Echo pins of the Ultrasonic Sensor
const int trigPin = 10; //digital pin
const int echoPin = 11; // digital pin
int servofeed = A0; // analog pin
int servo_pos = 0; // position placeholder for servo

// Variables for the duration and the distance
long duration;
int distance;
Servo myServo; // Creates a servo object for controlling the servo motor


void setup() {

  nh.initNode();
  nh.advertise(pub_range);
  nh.advertise(servopos);
  //Serial1.begin(115200);

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id = "/USH";
  range_msg.field_of_view = 0.523; // 30 degree field of vision
  range_msg.min_range = 0.020;  // 2 cm
  range_msg.max_range = 4.00;   // 4 m
  
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  myServo.attach(12); // Defines on which pin is the servo motor attached

}

void loop() {

  // Rotates from 165 to 15 degrees
  for(float i=180;i>0;i = i- 0.65){ // Used to be i--  
  myServo.write(i);
  
  servo_pos = analogRead(servofeed);

  range_msg.range = calculateDistance(); // Ultrasound publishing
  range_msg.header.stamp = nh.now();
  pub_range.publish( &range_msg);
  
  str_msg.data = servo_pos; // Servo position publishing
  servopos.publish( &str_msg);
  
  delay(30);
  }
  //delay(100); // Stops the servo before turning the other way
  
  // rotates the servo motor from 15 to 165 degrees
  for(float i=0;i<=180;i = i + 0.65){ // Used to be i++   
  myServo.write(i);
  
  servo_pos = analogRead(servofeed);

  range_msg.range = calculateDistance(); // Ultrasound publishing
  range_msg.header.stamp = nh.now();
  pub_range.publish( &range_msg);
  
  str_msg.data = servo_pos; // Servo position publishing
  servopos.publish( &str_msg);
  
  delay(30);
  }
  //delay(100); // Stops the servo before turning the other way
  
  nh.spinOnce();
  //delay(1);
}

// Function for calculating the distance measured by the Ultrasonic sensor
float calculateDistance(){ 
  
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
  distance= duration*0.034/2;
  return distance; // 100; // in meters. Try 1000 next time
}
