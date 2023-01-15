#include "ax12Serial.h"         //  for Teensy
#include "BioloidSerial.h"      //  for Teensy
#include "ax12_rb.h"            //  custom Library
#include "rb_movement.h"        //  movement library
#include <Servo.h>

#define SERVO_DIRECTION_PIN -1  //  leave as is 
#define AX_BUS_UART Serial1     //  use UART1 (pin 0 and 1 on Teensy)
#define SERVO1_SPECIAL  19      //  We wish to reserve servo 1 so we can see servo reset
#define POMPA1 12
#define Crane 23

Servo myservo;  // create servo object to control a servo 
                // twelve servo objects can be created on most boards
 
int pos = 0;    // variable to store the servo position

//=============================================================================
// Define Servos
//=============================================================================

BioloidControllerEx bioloid;    //  create bioloid class

Leg LF(7, 2, 3, 0);
Leg LM(4, 5, 6, 0);
Leg LR(19, 8, 9, 0);
Leg RR(10, 11, 15, 1);
Leg RM(13, 14, 12, 1);
Leg RF(16, 17, 18, 1);

Robot Reignblaze(&LF, &LM, &LR, &RR, &RM, &RF);

#define coxalength 29
#define femurlength 50
#define tibialength 75

#define reignblaze_speed 128

void InitRobot()
{
  //======DEFAULT PROCEDURE=====================================================================================================================================
  while (!Serial && (millis() < 3000)) ;                        // Give time for Teensy and other USB arduinos to create serial port
  Serial.begin(9600);                                          // start off the serial port.
  delay(250);
  bioloid.begin(1000000, &AX_BUS_UART, SERVO_DIRECTION_PIN);    // begin communication with 1 Mbaud in chosen Serial
  delay(100);
  word myVoltage;
  Serial.println(myVoltage = ax12GetRegister(7, AX_PRESENT_VOLTAGE, SERVO1_SPECIAL), DEC);  //  Debug voltage in LF COXA (should be the same as supply voltage)
  //======DEFAULT PROCEDURE====================================================================================================================================
  pinMode(A13, INPUT);
  pinMode(A17, INPUT);
  pinMode(A12, INPUT);
  pinMode(A16, INPUT);
  pinMode(A15, INPUT);
  pinMode(A20, INPUT);
  pinMode(17,INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  //======REIGNBLAZE===========================================================================================================================================
  LF.init(coxalength, femurlength, tibialength, 512, 515, 511); //  . . 505 coxlength, femlength, tiblength, normal_pos1, normal_pos2, normal_pos3
  LM.init(coxalength, femurlength, tibialength, 512, 516, 514); //  . . 510
  LR.init(coxalength, femurlength, tibialength, 542, 522, 516); //  . 522.  520
  RR.init(coxalength, femurlength, tibialength, 512, 520, 517); //  . 520.  520
  RM.init(coxalength, femurlength, tibialength, 512, 510, 512); //  . 500.  
  RF.init(coxalength, femurlength, tibialength, 512, 478, 509); //
  //======REIGNBLAZE===========================================================================================================================================
  Reignblaze.NormalInit(128);
}

bool parsing = false;
String data_cek;
String data[6];

int nilai_int[6];

void setup() 
{
  InitRobot();
  data_cek = "";
  //RobotRotateLeft(4);
  //RobotForward(1);
  digitalWrite();
  myservo.attach(23);  // attaches the servo on pin 23 to the servo object
}

void PompaON()
{
  digitalWrite(POMPA1, LOW);
}

void PompaOFF()
{
  digitalWrite(POMPA1, HIGH);
}

void pemadaman1() {
  if (data[1] == 'h') {
    PompaON();
    delay(1000);
    PompaOFF();
  } else if (data [1] == 'i') {
    // Robot Turun
    for(pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      myservo.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(15);                       // waits 15ms for the servo to reach the position 
    } 
    for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
    {                                
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
    }
    //Robot Naik
    RobotRotateRight(8);
    RobotForward(1);
    for(pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      myservo.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(15);                       // waits 15ms for the servo to reach the position 
    }
    RobotReverse(2);
    RobotRotateLeft(4);
    RobotForward(1);
  }
}

void pemadaman2() {
  if (data[1] == 'h') {
    PompaON();
    delay(1000);
    PompaOFF();
  } else if (data [1] == 'i') {
    // Robot Turun
    for(pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      myservo.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(15);                       // waits 15ms for the servo to reach the position 
    } 
    for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
    {                                
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
    }
    //Robot Naik
    RobotReverse(1);
    RobotRotateLeft(4);
    RobotForward(1);
    for(pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      myservo.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(15);                       // waits 15ms for the servo to reach the position 
    }
    RobotReverse(1);
    RobotRotateLeft(4);
    RobotForward(1);
  }
}

void loop()
{
  while (Serial.available() > 0)
  {
    char data_masuk = Serial.read();
    //Serial.print(data_masuk);
    data_cek += data_masuk;
    if(data_masuk == '$')
    {
      parsing = true;
    }
    if(parsing)
    {
      int q = 0;
      for(int i = 0; i < data_cek.length(); i++)
      {
        if(data_cek[i] == ';')
        {
          q++;
          data[q] = "";
        }
        else
        {
          data[q] += data_cek[i];
        }
      }
      
      // Start SWITCH CASE
      //int k1 = 0;
      //int k2 = 0;
      //int k3 = 0;
      //int k5 = 0;
      /*if (data[1] == 'z') {
          if (k1 == 0) {
            RobotRotateLeft(4);
            RobotForward(2);
            k1++;
          } else {
            RobotRotateRight(4);
          ` RobotForward(1);
            TurnOff();
          };
        }
        if (data[1] == 'y') {
          if (k2 < 2) {
            RobotRotateLeft(4);
            RobotForward(2);
            k2++;
          } else {
            RobotForward(2);
          }
        }
        if (data[1] == 'x') {
          if (k3 == 0) {
            RobotRotateLeft(4);
            RobotForward(1);
            k3++;
          } else {
            RobotRotateRight(4);
            RobotForward(2);
          }
        }
        if (data[1] == 'w') {
          RobotRotateRight(4);
          Pemadaman1();
        }
        if (data[1] == 'v') {
          if (k5 == 0) {
            RobotRotateLeft(4);
            Pemadaman2();
            k5++;
          } else {
            RobotRotateRight(4);
            RobotForward(2);
          }
        }*/
      
      Serial.print("input " + data[1] + " sedang diproses");
      
      parsing = false;
      data_cek = "";
    }
  }
}

void RobotForward(int robot_steps)
{
  ForwardTripod(Reignblaze, reignblaze_speed, robot_steps, 0, 0);
}

void RobotReverse(int robot_steps)
{
  ReverseTripod(Reignblaze, reignblaze_speed, robot_steps, 0, 0);
}

void RobotRotateRight(int robot_steps)
{
  RotationRightTripod(Reignblaze, reignblaze_speed, robot_steps, 0, 0);
}

void RobotRotateLeft(int robot_steps)
{
  RotationLeftTripod(Reignblaze, reignblaze_speed, robot_steps, 0, 0);
}

void RobotRotateRightSmooth(int robot_steps)
{
  RotationRightSmooth(Reignblaze, reignblaze_speed, robot_steps);
}

void RobotRotateLeftSmooth(int robot_steps)
{
  RotationLeftSmooth(Reignblaze, reignblaze_speed, robot_steps);
}
