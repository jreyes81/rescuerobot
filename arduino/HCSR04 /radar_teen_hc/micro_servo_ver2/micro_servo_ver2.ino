/*
* 180 micro servo 
* Modified By: Amr Wanly and Jeovanny Reyes
* Modified to work on ROS
* Raytheon Radar Guided Rescue Robot
* Cal State LA
*
* Input: "Float32" messages that contains state information on when to sweep and when to stop sweeping
* Output: "Float32" messages that contains position of micro servo
*
* Publisher:servopos pubslishes to "HerculesUltrasound_Position" topic --> Position of servo attached to ultrasound
* 
* Subscriber: subscribes to "scan_once_to_whls" topic --> Contains either 0 (sweep) or 1 (don't sweep)
*
* How to Run from Terminal: Run "roscore" 
*                           On a sperate terminal run "rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600"
*                           To start ultrasound sensor run "rostopic pub -r 10 /scan_once_to_whls std_msgs/Float32 0"
*                           Note: _port may be different for other and baud rate parameter can be changed
*/

//#if (ARDUINO >= 100)
// #include <Arduino.h>
//#else
// #include <WProgram.h>
//#endif

#include <ros.h>
//#include <ros/time.h>
#include <std_msgs/Float32.h> // Used for range messages

#include <Servo.h> 

ros::NodeHandle nh;

int servofeed = A14; // analog pin
int servo_pos = 0; // position placeholder for servo
int temp = 0; //variable that will store the analog feedback signal
int start_pos = 5; // Starting position for servo


Servo myServo; // Creates a servo object for controlling the servo motor

void radar_swp_cb( const std_msgs::Float32& state_msg){ //Used to be Float32 
  float sweep_state = state_msg.data; // Contains either 0 or 1. 0 for sweep and 1 for not sweep

  if (sweep_state == 0) // Radar sweeping
  {
    for(int i=175;i>start_pos;i--)
    {
      //if (sweep_state == 1)
      //{
        //myServo.write(175);
        //break;
        //}
      myServo.write(i);
      delay(1);
      temp = analogRead(servofeed);
      servo_pos = ((0.3956*temp)-75.165);
      //str_msg.data = servo_pos; // Servo position publishing
      //servopos.publish( &str_msg);
      delay(28); // Used to be 30 then 20 then 45
      }
      delay(100); // Stoping before rotating other way!
     for(int i=start_pos;i<=175;i++)
     {
     // if (sweep_state == 1)
      //{
       // myServo.write(175);
       // break;
      //  }
      myServo.write(i);
      delay(1);
      temp = analogRead(servofeed);
      servo_pos = ((0.3956*temp)-75.165);
      //str_msg.data = servo_pos; // Servo position publishing
      //servopos.publish( &str_msg);
      delay(28); // Used to be 30 then 20 then 45
      }
      delay(19500); // Used to be 100 then 8000
    }

  if (sweep_state == 1) // Radar not sweeping and robot moving
  {
    myServo.write(170);
    temp = analogRead(servofeed);
    servo_pos = ((0.3956*temp)-75.165); // Converts feedback counts to angles.
    //str_msg.data = servo_pos; // Servo position publishing. 0 degrees
    //servopos.publish( &str_msg);
    }
}

ros::Subscriber<std_msgs::Float32> sub_state_msg("scan_once_to_whls", radar_swp_cb);

std_msgs::Float32 int_msg; // Creatng message instance
//ros::Publisher servopos("/HerculesUltrasound_Position", &str_msg);

void setup() {
  nh.initNode();
  nh.subscribe(sub_state_msg);

  myServo.attach(5); // Defines on which pin is the servo motor attached
  myServo.write(170);
}

void loop() {
  nh.spinOnce();
  //delay(1);
}

