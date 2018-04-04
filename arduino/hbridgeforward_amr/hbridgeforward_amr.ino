 /* 
    Amr - 11/14/2017
    Basic Navigation 
    Car Moves Forward and makes a turn procedure
*/

// defining the pins connected between the L298N Dual H-Bridge and Arduino Microcontroller

//pins that will control speed of motors, enable PWM
int ENA = 30;
int ENB = 2;

//right motors
int IN1 = 10;
int IN2 = 9;

//left motors
int IN3 = 7;
int IN4 = 6;

void straight()
{
  digitalWrite(ENA, HIGH); // Going forward
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  }

void right ()
{
  digitalWrite(ENA, HIGH); // Going Right
  digitalWrite(ENB, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  }

void setup() {
  // put your setup code here, to run once:
  //define the mode of the pins, are they pins sending data to outside peripherals? or recieving data? .....
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}


/*
    To move the car forward, both left motors and right motors must be rotating forward.

    So that means:
    Right Motor - ENA = 1; IN1 = 0; IN2 = 1
    Left Motor - ENB = 1; IN3 = 0; IN4 = 1


    We write the code for that in void loop() below
*/

void loop() {
  // put your main code here, to run repeatedly:
  straight();
  delay(3000);
  right();
  delay(3000); 
  
}
