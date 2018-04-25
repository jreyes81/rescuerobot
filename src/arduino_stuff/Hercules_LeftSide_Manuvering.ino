/* 
 *  
 *  Amr - Code for the Heculues Motor Controller
 *  
 *  
 *  Credit: the Herculues Motor Driver Library was obtained from  https://github.com/Seeed-Studio/Hercules_Motor_Driver
 *     
 *     
 *     Work in Progress
 *     
*/


// Case 3 - Deviating from track to the left side then going back on track


/* ALL THE SPEEDS AND DELAY PERIODS ARE PLACEHOLDERS */



#include <Hercules.h>

void setup()
{
    MOTOR.begin();                      // initialize the motors
}

void loop()
{
  MOTOR.setSpeedDir(20, DIRF);         // move all the motors forward at speed of PWM = 20% for 2 seconds (DIRF = Direction Forward)
  delay(2000);

  MOTOR.setStop1();                     // Stop both left and right side motors fopr 2 seconds
  MOTOR.setStop2();
  delay(2000);

  // Assuming there is an object in the way, thr robot will be go around the object from the left side 
   
  // Stop the right side motors while keeping the left ones rotating forward at speed of PWM = 20%
  // The orientation of the vehicle will begin to change towards the left side
  MOTOR.setSpeedDir2(20, DIRF);
  MOTOR.setStop1();
  delay(2000);

  //the char now is ready to manuever around the object
  //We begin so by moving it a little bit forwad so that the right side wheels are facing the edge of the object
  MOTOR.setSpeedDir(10, DIRF);         
  delay(1000);


  //Adjust the orientation of the car, a little bit to the right
  MOTOR.setStop2();
  MOTOR.setSpeedDir1(20, DIRF);
  delay(2000);

  //move the car forward for 3 seconds
  MOTOR.setSpeedDir(20, DIRF);
  delay(3000);


  //Adjust the orientation of the car, a little bit to the right
  MOTOR.setStop2();
  MOTOR.setSpeedDir1(20, DIRF);
  delay(2000);


  //move the car forward for 3 seconds
  MOTOR.setSpeedDir(20, DIRF);
  delay(3000);

  //Bring the car back to its original track by adjusting its orientation a little bit to the left
  MOTOR.setSpeedDir2(10, DIRF);
  MOTOR.setStop1();
  delay(2000);


  //Now Keep moving forward to Point B, the destination 
  MOTOR.setSpeedDir(20, DIRF);
  delay(3000);
  
  MOTOR.setStop2();                    // Stop all the motors for 5 seconds
  MOTOR.setStop1();
  delay(5000);

}



