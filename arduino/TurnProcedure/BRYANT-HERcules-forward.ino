/* 
    Created by: Amr Wanly, Bryant Santos and Mario Medina

*/




#include <Hercules.h>



void setup() {
  // put your setup code here, to run once:
MOTOR.begin();

}

void loop() {

  //phase 1:

MOTOR.setSpeedDir(30,DIRF); // moves the robot forward 
delay(1500);

MOTOR.setStop1();           // stops the right motor wheels
MOTOR.setSpeedDir2(40,DIRF); // moves the robot to the right 
delay(500);                   
MOTOR.setStop1();     // stops right motors
MOTOR.setStop2();     // stops left motors
delay(5000);

//phase 2:
MOTOR.setSpeedDir(10,DIRF);// moves forward
delay(1000);
MOTOR.setSpeedDir1(40,DIRF); //moves the right wheels forward
MOTOR.setSpeedDir2(40,DIRR); // moves the left wheels reverse 
delay(500);
MOTOR.setStop1();
MOTOR.setStop2();
delay(5000);



}
