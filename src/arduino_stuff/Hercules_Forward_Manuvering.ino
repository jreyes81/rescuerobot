/* 
 *  Amr - Code for the Heculues Motor Controller
 *  
 *  
 *  Credit: the Herculues Motor Driver Library was obtained from  https://github.com/Seeed-Studio/Hercules_Motor_Driver
 *     
 *     
 *     Work in Progress
 *     
*/


// Case 2 - No need to deviate from track


/* ALL THE SPEEDS AND DELAY PERIODS ARE PLACEHOLDERS */

#include <Hercules.h>

void setup()
{
    MOTOR.begin();                      // initialize
}

void loop()
{


   MOTOR.setStop1();                    // keep the motors off for 2 seconds
   MOTOR.setStop2();
   delay(2000);

   MOTOR.setSpeedDir(15, DIRF);         // all motors rotates forward for 5 seconds
   delay(5000); 

      
   MOTOR.setStop1();                    // Stop the motors for 10 seconds to double check that there is not anything in the way of the robot
   MOTOR.setStop2();
   delay(10000); 

   MOTOR.setSpeedDir(10, DIRF);         // all motors rotates forward for 2 seconds at speed of PWM = 10%
   delay(2000); 
   MOTOR.setSpeedDIR(30, DIRF);         // all motors rotates forward for 5 seconds at speed of PWM = 30%
   delay(5000);


   MOTOR.setStop1();                    // Stop all the motors for 7 seconds
   MOTOR.setStop2();
   delay(7000);
   
         
}



