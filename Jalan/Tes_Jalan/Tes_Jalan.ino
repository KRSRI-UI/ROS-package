#include "ax12Serial.h"         //  for Teensy
#include "BioloidSerial.h"      //  for Teensy
#include "ax12_rb.h"            //  custom Library
#include "rb_movement.h"        //  movement library

#define SERVO_DIRECTION_PIN -1  //  leave as is 
#define AX_BUS_UART Serial1     //  use UART1 (pin 0 and 1 on Teensy)
#define SERVO1_SPECIAL  19      //  We wish to reserve servo 1 so we can see servo reset
#define POMPA1 12

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


#define move_forward 1
#define move_reverse 2
#define move_rotate_right 3
#define move_rotate_left 4
#define move_rotate_right_smooth 5
#define move_rotate_left_smooth 6
#define reignblaze_speed 256

int Forward = 0, Backward = 0, Right = 0, Left = 0, Spray = 0;

enum moveDirection {forward, reverse, rotateleft, rotateright, rotateleftsmooth, rotaterightsmooth};
moveDirection myMoveDirection, lastMoveDirection;

void InitRobot()
{
  //======DEFAULT PROCEDURE=====================================================================================================================================
  while (!Serial && (millis() < 3000)) ;                        // Give time for Teensy and other USB arduinos to create serial port
  Serial.begin(38400);                                          // start off the serial port.
  delay(250);
  bioloid.begin(1000000, &AX_BUS_UART, SERVO_DIRECTION_PIN);    // begin communication with 1 Mbaud in chosen Serial
  delay(100);
  Serial.print("System Voltage in 10ths: ");
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
  //pinMode(UVTRON_PIN, INPUT);
  //attachInterrupt(UVTRON_PIN, CountUVPulse, FALLING);
  pinMode(LED_BUILTIN, OUTPUT);
  //pinMode(PROX, INPUT);
  //pinMode(pin_button_right, INPUT);
  //pinMode(pin_button_left, INPUT);
  //pinMode(pin_button_ok, INPUT);
  //pinMode(pin_button_return, INPUT);
  //pinMode(pin_button_right, INPUT);
  //pinMode(pin_button_start, INPUT);

  //pinMode(getsoundLED, OUTPUT);
  //pinMode(getsoundLED, OUTPUT);

  PompaOFF();
  //======REIGNBLAZE===========================================================================================================================================
  LF.init(coxalength, femurlength, tibialength, 512, 512, 512); //coxlength, femlength, tiblength, normal_pos1, normal_pos2, normal_pos3
  LM.init(coxalength, femurlength, tibialength, 512, 530, 512);
  LR.init(coxalength, femurlength, tibialength, 542, 502, 520);
  RR.init(coxalength, femurlength, tibialength, 512, 512, 512);
  RM.init(coxalength, femurlength, tibialength, 512, 505, 512);
  RF.init(coxalength, femurlength, tibialength, 512, 512, 512);
  //======REIGNBLAZE===========================================================================================================================================
  Reignblaze.NormalInit(128);
}

void PompaON()
{
  digitalWrite(POMPA1, LOW);
}

void PompaOFF()
{
  digitalWrite(POMPA1, HIGH);
}

void setup() {
  InitRobot();
  delay(3000);
  // put your setup code here, to run once:

}

int wiggleTime;
unsigned long prevWiggle = 0;

void loop() {
//  int prox = digitalRead(17);
//  if(prox==LOW)
//    RobotRotateLeft(1);
      
  RobotForward(1);
  //delay(1000);
//  RobotRotateLeft(5);
//  delay(100);
  //wiggleTime = 1000;
//  while (wiggleTime > 0)
//  {
//    wiggle();
//    wiggleTime--;
//    delay(1);
//
//    Serial.println("WIGGLE WIGGLE");
//  }
}
// put your main code here, to run repeatedly:
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
      ForwardTripodV2(Reignblaze, mySpeed);
      lastMoveDirection = myMoveDirection;
      break;

    case reverse:
      ReverseTripodV2(Reignblaze, mySpeed);
      lastMoveDirection = myMoveDirection;
      break;

    case rotateleft:
      RotationLeftTripodV2(Reignblaze, mySpeed);
      lastMoveDirection = myMoveDirection;
      break;

    case rotateright:
      RotationRightTripodV2(Reignblaze, mySpeed);
      lastMoveDirection = myMoveDirection;
      break;

    default:
      //lcd.clear();
      //lcd.print("cant determine dir");
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

void RobotForward(int robot_steps)
{
  ForwardTripod(Reignblaze, reignblaze_speed, robot_steps, 1);
  /*for (int i = 1; i <= robot_steps; i++)
    {
    record[steps_taken] = move_forward;
    steps_taken += 1;
    }*/
}

void RobotReverse(int robot_steps)
{
  ReverseTripod(Reignblaze, reignblaze_speed, robot_steps);
  /*for (int i = 1; i <= robot_steps; i++)
    {
    record[steps_taken] = move_reverse;
    steps_taken += 1;
    }*/
}

void RobotRotateRight(int robot_steps)
{
  RotationRightTripod(Reignblaze, reignblaze_speed, robot_steps);
  /*for (int i = 1; i <= robot_steps; i++)
    {
    record[steps_taken] = move_rotate_right;
    steps_taken += 1;
    }*/
}

void RobotRotateLeft(int robot_steps)
{
  RotationLeftTripod(Reignblaze, reignblaze_speed, robot_steps);
  /*for (int i = 1; i <= robot_steps; i++)
    {
    record[steps_taken] = move_rotate_left;
    steps_taken += 1;
    }*/
}

void RobotRotateRightSmooth(int robot_steps)
{
  RotationRightSmooth(Reignblaze, reignblaze_speed, robot_steps);
  /*for (int i = 1; i <= robot_steps; i++)
    {
    record[steps_taken] = move_rotate_right_smooth;
    steps_taken += 1;
    }*/
}

void RobotRotateLeftSmooth(int robot_steps)
{
  RotationLeftSmooth(Reignblaze, reignblaze_speed, robot_steps);
  /*for (int i = 1; i <= robot_steps; i++)
    {
    record[steps_taken] = move_rotate_left_smooth;
    steps_taken += 1;
    }*/
}

/*void Backtrack()
  {

  RotationRightTripod(Reignblaze, reignblaze_speed, 10);  //muter balik 180 derajat
  for (int i = steps_taken; i >= 0; i--)
  {
    if (record[steps_taken] == move_forward)
    {
      ForwardTripod(Reignblaze, reignblaze_speed, 1);
    }
    else if (record[steps_taken] == move_reverse)
    {
      ForwardTripod(Reignblaze, reignblaze_speed, 1);
    }
    else if (record[steps_taken] == move_rotate_left)
    {
      RotationLeftTripod(Reignblaze, reignblaze_speed, 1);
    }
    else if (record[steps_taken] == move_rotate_right)
    {
      RotationRightTripod(Reignblaze, reignblaze_speed, 1);
    }
    else if (record[steps_taken] == move_rotate_left_smooth)
    {
      RotationLeftSmooth(Reignblaze, reignblaze_speed, 1);
    }
    else if (record[steps_taken] == move_rotate_right_smooth)
    {
      RotationRightSmooth(Reignblaze, reignblaze_speed, 1);
    }
  }
  }*/
