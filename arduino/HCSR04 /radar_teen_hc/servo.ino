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

/*
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

  servo_pos = ((0.3956*temp)-75.165);

  
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

 
Servo myservo;  
 
// Control and feedback pins
unsigned const int servoPin = 5;
unsigned const int feedbackPin = A14;
 
// Calibration values
int minDegrees;
int maxDegrees;
int minFeedback;
int maxFeedback;
int pos;
int temp01;
/*
  This function establishes the feedback values for 2 positions of the servo.
  With this information, we can interpolate feedback values for intermediate positions
*/
void calibrate(Servo servo, int analogPin, int minPos, int maxPos)
{
  // Move to the minimum position and record the feedback value
  servo.write(minPos);
  minDegrees = minPos;
  delay(2000); // make sure it has time to get there and settle
  minFeedback = analogRead(analogPin);
  
  // Move to the maximum position and record the feedback value
  servo.write(maxPos);
  maxDegrees = maxPos;
  delay(2000); // make sure it has time to get there and settle
  maxFeedback = analogRead(analogPin);
}
 
 
void setup() 
{ 

  nh.initNode();
  nh.advertise(servopos);
  
  myservo.attach(servoPin); 
  
  calibrate(myservo, feedbackPin, 0, 180);  // calibrate for the 20-160 degree range
} 
 
void loop()
{
      //Serial.println(minFeedback);
      //delay(500);
      //Serial.println(maxFeedback);
      //delay(500);
    
  
  for (pos = 0; pos <= 180; pos += 20) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    temp01 = analogRead(feedbackPin);
    temp01 = map(temp01, 132, 685, 0, 179);
      str_msg.data = temp01; // Servo position publishing
      servopos.publish( &str_msg);
          delay(500);                       // waits 15ms for the servo to reach the position

      
    }
    
   nh.spinOnce();
  //delay(1);
}

