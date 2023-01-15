#include <Servo.h>

#include "ax12Serial.h"         //  for Teensy
#include "BioloidSerial.h"      //  for Teensy
#include "ax12_rb.h"            //  custom Library
#include "rb_movement.h"        //  movement library

#define SERVO_DIRECTION_PIN -1  //  leave as is 
#define AX_BUS_UART Serial1     //  use UART1 (pin 0 and 1 on Teensy)
#define SERVO1_SPECIAL  19      //  We wish to reserve servo 1 so we can see servo reset
#define POMPA1 12
#define START 6
#define Crane 23

Servo myservo;
int pos = 0;

unsigned long previousMillis_goyang;
unsigned long interval_goyang = 500;
bool goyang;

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

//=============================================================================
// Backtrack Mechanism
//=============================================================================

enum moveDirection {forward, reverse, rotateleft, rotateright, rotateleftsmooth, rotaterightsmooth};
moveDirection myMoveDirection, lastMoveDirection;

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
  pinMode(POMPA1, OUTPUT);
  pinMode(START, INPUT);
  myservo.attach(Crane);

  pinMode(LED_BUILTIN, OUTPUT);

  //======REIGNBLAZE===========================================================================================================================================
  LF.init(coxalength, femurlength, tibialength, 512, 515, 511); // coxlength, femlength, tiblength, normal_pos1, normal_pos2, normal_pos3
  LM.init(coxalength, femurlength, tibialength, 512, 516, 514); // sudah terkalibrasi
  LR.init(coxalength, femurlength, tibialength, 542, 522, 516); //
  RR.init(coxalength, femurlength, tibialength, 512, 520, 517); //
  RM.init(coxalength, femurlength, tibialength, 512, 510, 512); //
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
  delay(5000);
  InitRobot();
  delay(3000);
  data_cek = "";
}

void loop()
{
  int baca = digitalRead(START);
  if (baca == 0){
    jalan = true;
  }
  while(jalan == true){
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
        
        //BARU

        RobotRotateLeft(16, 0, 0);
        RobotForward(8, 0, 0);

        RobotRotateLeft(16, 0, 0);
        RobotForward(4, 0, 0);
        RobotForward(16, 11, 11);
        RobotForward(4, 0, 0);

        RobotRotateLeft(16, 0, 0);
        RobotForward(16, 0, 0);
        
        RobotRotateLeft(16, 0, 0);
        RobotForward(8, 0, 0);

        RobotRotateRight(16, 0, 0);
        RobotForward(4, 0, 0);

        digitalWrite(POMPA1, LOW);
        delay(1000);
        digitalWrite(POMPA1, HIGH);

        for(pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
        {                                  // in steps of 1 degree 
          myservo.write(pos);              // tell servo to go to position in variable 'pos' 
          delay(15);                       // waits 15ms for the servo to reach the position 
        }        

        RobotForward(4, 0, 0);

        for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
        {                                
          myservo.write(pos);              // tell servo to go to position in variable 'pos'
          delay(15)
        }

        RobotReverse(8, 11, 11);
        
        RobotRotateRight(40, 11, 11);
        RobotForward(16, 11, 11);
        RobotForward(4, 0, 0);

        for(pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
        {                                  // in steps of 1 degree 
          myservo.write(pos);              // tell servo to go to position in variable 'pos' 
          delay(15);                       // waits 15ms for the servo to reach the position 
        }

        RobotReverse(8, 0, 0);

        for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
        {                                
          myservo.write(pos);              // tell servo to go to position in variable 'pos'
          delay(15);
        }

        RobotRotateLeft(16, 0, 0);
        RobotForward(8, 0, 0);

        RobotRotateRight(16, 0, 0);
        RobotForward(16, 0, 0);

        RobotRotateLeft(16, 0, 0);
        RobotForward(4, 0, 0);
        RobotForward(16, 11, 11);
        RobotForward(4, 0, 0);

        RobotRotateLeft(16, 0, 0);
        RobotForward(8, 0, 0);

        digitalWrite(POMPA1, LOW);
        delay(1000);
        digitalWrite(POMPA1, HIGH);

        RobotForward(4, 0, 0);

        for(pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
        {                                  // in steps of 1 degree 
          myservo.write(pos);              // tell servo to go to position in variable 'pos' 
          delay(15);                       // waits 15ms for the servo to reach the position 
        }        

        RobotForward(4, 0, 0);

        for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
        {                                
          myservo.write(pos);              // tell servo to go to position in variable 'pos'
          delay(15)
        }

        RobotReverse(8, 11, 11);
        
        RobotRotateLeft(16, 11, 11);
        RobotForward(8, 11, 11);
        RobotForward(4, 0, 0);

        for(pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
        {                                  // in steps of 1 degree 
          myservo.write(pos);              // tell servo to go to position in variable 'pos' 
          delay(15);                       // waits 15ms for the servo to reach the position 
        }

        RobotReverse(8, 0, 0);

        for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
        {                                
          myservo.write(pos);              // tell servo to go to position in variable 'pos'
          delay(15)
        }

        RobotRotateLeft(16, 0, 0);
        RobotForward(8, 0, 0);

        RobotRotateRight(16, 0, 0);
        RobotForward(4, 0, 0);
        RobotForward(16, 11, 11);
        RobotForward(4, 0, 0);

        RobotForward(8, 0, 0);

        RobotForward(4, 0, 0);
        RobotForward(16, 11, 11);
        RobotForward(4, 0, 0);

        RobotRotateRight(16, 0, 0);
        RobotForward(4, 0, 0);

        Serial.print("input " + data[1] + " sedang diproses");
        
        parsing = false;
        data_cek = "";
      }
    }
  }

}

//=============================================================================================================================================
//      MOVEMENTS
//=============================================================================================================================================
void MoveRobot(moveDirection newMoveDirection)
{
  int mySpeed = 150;
  if (newMoveDirection != lastMoveDirection)
  {
    //lcd.clear();
    //lcd.print("Change Dir");
    Reignblaze.NormalInit(mySpeed);
    ResetSiklus();
    delay(500);
  }
  switch (myMoveDirection)
  {
    case forward:
      ForwardTripodV2(Reignblaze, mySpeed, 0, 0);
      lastMoveDirection = myMoveDirection;
      break;

    case reverse:
      ReverseTripodV2(Reignblaze, mySpeed, 0, 0);
      lastMoveDirection = myMoveDirection;
      break;

    case rotateleft:
      RotationLeftTripodV2(Reignblaze, mySpeed, 0, 0);
      lastMoveDirection = myMoveDirection;
      break;

    case rotateright:
      RotationRightTripodV2(Reignblaze, mySpeed, 0, 0);
      lastMoveDirection = myMoveDirection;
      break;

    default:
      break;
  }
}

void wiggle() {
  if (millis() - previousMillis_goyang >= interval_goyang)
  {
    if (!goyang)
    {
      myMoveDirection = rotateleft;
      MoveRobot(myMoveDirection);
    }
    else
    {
      myMoveDirection = rotateright;
      MoveRobot(myMoveDirection);
    }

    previousMillis_goyang = millis();
    goyang = !goyang;
  }
}

void RobotForward(int robot_steps, float FEMANG, float TIBANG)
{
  ForwardTripod(Reignblaze, reignblaze_speed, robot_steps, FEMANG, TIBANG);
}

void RobotReverse(int robot_steps, float FEMANG, float TIBANG)
{
  ReverseTripod(Reignblaze, reignblaze_speed, robot_steps, FEMANG, TIBANG);
}

void RobotRotateRight(int robot_steps, float FEMANG, float TIBANG)
{
  RotationRightTripod(Reignblaze, reignblaze_speed, robot_steps, FEMANG, TIBANG);
}

void RobotRotateLeft(int robot_steps, float FEMANG, float TIBANG)
{
  RotationLeftTripod(Reignblaze, reignblaze_speed, robot_steps, FEMANG, TIBANG);
}

void RobotRotateRightSmooth(int robot_steps)
{
  RotationRightSmooth(Reignblaze, reignblaze_speed, robot_steps);
}

void RobotRotateLeftSmooth(int robot_steps)
{
  RotationLeftSmooth(Reignblaze, reignblaze_speed, robot_steps);
}
