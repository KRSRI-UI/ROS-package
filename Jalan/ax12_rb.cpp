/*
  ax12_rb.cpp - Reignblaze Servo library Sourcefile for Arduino.
  Dynamixel AX-12 / AX-18 Servo, Leg, and Inverse Kinematics library for Teensy 3.1/3.2
  Copyright (c) 2019 Irfan Budi Satria (KRPAI TRUI).  All right reserved.
*/

#include "BioloidSerial.h"
#include "ax12Serial.h"
#include "ax12_rb.h"
#include <SpeedTrig.h>

#define Pi 3.14159265359
#define Rad2Deg 180/Pi

//SERVO=============================================================PUBLIC METHODS==================================================
ax12_rb::ax12_rb()
{
}

void ax12_rb::attach(byte servoID)
{
  this->attach(servoID, _minAngle_DEFAULT, _maxAngle_DEFAULT);
}

void ax12_rb::attach(byte servoID, int minAngle, int maxAngle)
{
  _myServoID = servoID;
  _minAngle = minAngle;
  _maxAngle = maxAngle;
}

void ax12_rb::writeAngle(int angle)
{
  this->writeAngle(angle, _speed_DEFAULT);
}

void ax12_rb::writeAngle(int angle, int gSpeed)
{
  //SetGoalSpeed
  this->setSpeed(gSpeed);
  //SetGoalPosition
  float temp = ((float)angle * 2.8444) + _normalValue; 
  word _ang = (int)temp;
  //  word _ang = map(angle, -90, 90, 256, 768);
  _myServoGoalPos = _ang;
  ax12SetRegister2(_myServoID, AX_GOAL_POSITION_L, _myServoGoalPos);
  ax12ReadPacket(6);  // git the response...
//  Serial.print(" ID: ");
//  Serial.print(_myServoID, DEC);
//  Serial.print(" ");
//  Serial.println(_myServoGoalPos, DEC);
//  delay(1);
}

void ax12_rb::writeValue(int value, int gSpeed)
{
  //SetGoalSpeed
  this->setSpeed(gSpeed);
  //SetGoalPosition
  word _ang = value + (_normalValue - 512);
  _myServoGoalPos = _ang;
  ax12SetRegister2(_myServoID, AX_GOAL_POSITION_L, _myServoGoalPos);
  ax12ReadPacket(6);  // git the response...
//  Serial.print(" ID: ");
//  Serial.print(_myServoID, DEC);
//  Serial.print(" ");
//  Serial.println(_myServoGoalPos, DEC);
//  delay(1);
}

//SERVO=============================================================PUBLIC METHODS==================================================

//SERVO=============================================================PRIVATE METHODS==================================================
void ax12_rb::setSpeed(int gSpeed)
{
  //SetGoalSpeed
  _myServoGoalSpeed = gSpeed;
  ax12SetRegister2(_myServoID, AX_GOAL_SPEED_L, _myServoGoalSpeed);
  ax12ReadPacket(6);  // git the response...
//  Serial.print("Goal Speed: ");
//  Serial.println(_myServoGoalSpeed, DEC);
//  delay(1);
}
//SERVO=============================================================PRIVATE METHODS==================================================

//LEG=============================================================PUBLIC METHODS==================================================
Leg::Leg(byte COXID, byte FEMID, byte TIBID, int type)
{
  _COXA.attach(COXID);
  _FEMUR.attach(FEMID);
  _TIBIA.attach(TIBID);
  _type = type;
}

void Leg::init(float COXALENGTH, float FEMURLENGTH, float TIBIALENGTH, int COX_NORMAL_VAL, int FEM_NORMAL_VAL, int TIB_NORMAL_VAL)
{
  _COXALENGTH = COXALENGTH;
  _FEMURLENGTH = FEMURLENGTH;
  _TIBIALENGTH = TIBIALENGTH;
  _COXA._normalValue = COX_NORMAL_VAL;
  _FEMUR._normalValue = FEM_NORMAL_VAL;
  _TIBIA._normalValue = TIB_NORMAL_VAL;

  _COXA._normalAngle = map(COX_NORMAL_VAL, 256, 768, -90, 90);
  _FEMUR._normalAngle = map(FEM_NORMAL_VAL, 256, 768, -90, 90);
  _TIBIA._normalAngle = map(TIB_NORMAL_VAL, 256, 768, -90, 90);
}

void Leg::LegWriteVal(float COXVAL, float FEMVAL, float TIBVAL, int gSpeed)
{
  _COXA.writeValue(COXVAL, gSpeed);
  _FEMUR.writeValue(FEMVAL, gSpeed);
  _TIBIA.writeValue(TIBVAL, gSpeed);
}

void Leg::LegWriteAng(float COXANG, float FEMANG, float TIBANG, int gSpeed)
{
  _COXA.writeAngle(COXANG, gSpeed);
  _FEMUR.writeAngle(FEMANG, gSpeed);
  _TIBIA.writeAngle(TIBANG, gSpeed);
}

void Leg::FK(float COXANG, float FEMANG, float TIBANG)
{
  _currentX = this->FK_X(COXANG, FEMANG, TIBANG);
  _currentY = this->FK_Y(COXANG, FEMANG, TIBANG);
  _currentZ = this->FK_Z(COXANG, FEMANG, TIBANG);
}

void Leg::IK(float x, float y, float z)
{
  z = -z;
  float L = sqrt(sq(z) + (sq(x - _COXALENGTH)));
  _currentq1 = this->IK_Q1(x, y, z);
  _currentq2 = this->IK_Q2(x, y, z, L);
  _currentq3 = this-> IK_Q3(x, y, z, L);
}
//LEG=============================================================PUBLIC METHODS==================================================

//LEG=============================================================PRIVATE METHODS==================================================
float Leg::FK_X(float COXANG, float FEMANG, float TIBANG)
{
  float x = _COXALENGTH * SpeedTrig.cos(COXANG) + _FEMURLENGTH * SpeedTrig.cos(COXANG) * SpeedTrig.cos(FEMANG) + _TIBIALENGTH * SpeedTrig.cos(TIBANG - 90) * SpeedTrig.cos(COXANG)
            * SpeedTrig.cos(FEMANG) - _TIBIALENGTH * SpeedTrig.sin(TIBANG - 90) * SpeedTrig.cos(COXANG) * SpeedTrig.sin(FEMANG);
  return x;
}

float Leg::FK_Y(float COXANG, float FEMANG, float TIBANG)
{
  float y = _COXALENGTH * SpeedTrig.sin(COXANG) + _FEMURLENGTH * SpeedTrig.cos(FEMANG) * SpeedTrig.sin(COXANG) + _TIBIALENGTH * SpeedTrig.cos(TIBANG - 90) * SpeedTrig.cos(FEMANG)
            * SpeedTrig.sin(COXANG) - _TIBIALENGTH * SpeedTrig.sin(TIBANG - 90) * SpeedTrig.sin(COXANG) * SpeedTrig.sin(FEMANG);
  return y;
}

float Leg::FK_Z(float COXANG, float FEMANG, float TIBANG)
{
  float z = _FEMURLENGTH * SpeedTrig.sin(FEMANG) + _TIBIALENGTH * SpeedTrig.cos(TIBANG - 90) * SpeedTrig.sin(FEMANG) + _TIBIALENGTH * SpeedTrig.sin(TIBANG - 90) * SpeedTrig.cos(FEMANG);
  return z;
}

float Leg::IK_Q1(float x, float y, float z)
{
  float gamma = abs(Rad2Deg * SpeedTrig.atan2(y, x));
  return gamma;
}

float Leg::IK_Q2(float x, float y, float z, float L)
{
  float alpha2 = Rad2Deg * SpeedTrig.acos((sq(_TIBIALENGTH) - sq(_FEMURLENGTH) - sq(L)) / ((-2) * _FEMURLENGTH * L));
  float alpha1 = Rad2Deg * SpeedTrig.acos(z / L);
  float alpha = abs(alpha1 + alpha2) - 90;
  return alpha;
}

float Leg::IK_Q3(float x, float y, float z, float L)
{
  float beta = abs(Rad2Deg * SpeedTrig.acos((sq(L) - sq(_TIBIALENGTH) - sq(_FEMURLENGTH)) / ((-2) * _TIBIALENGTH * _FEMURLENGTH))) - 90;
  return beta;
}
//LEG=============================================================PRIVATE METHODS==================================================

//ROBOT=============================================================PUBLIC METHODS==================================================
Robot::Robot(Leg* LFptr, Leg* LMptr, Leg* LRptr, Leg* RRptr, Leg* RMptr, Leg* RFptr)
{
  _LF = LFptr;
  _LM = LMptr;
  _LR = LRptr;
  _RR = RRptr;
  _RM = RMptr;
  _RF = RFptr;
}

void Robot::NormalInit(int gSpeed)
{
  _LF->LegWriteVal(_LF->_COXA._normalValue, _LF->_FEMUR._normalValue, _LF->_TIBIA._normalValue, gSpeed);
  _LM->LegWriteVal(_LM->_COXA._normalValue, _LM->_FEMUR._normalValue, _LM->_TIBIA._normalValue, gSpeed);
  _LR->LegWriteVal(_LR->_COXA._normalValue, _LR->_FEMUR._normalValue, _LR->_TIBIA._normalValue, gSpeed);
  _RR->LegWriteVal(_RR->_COXA._normalValue, _RR->_FEMUR._normalValue, _RR->_TIBIA._normalValue, gSpeed);
  _RM->LegWriteVal(_RM->_COXA._normalValue, _RM->_FEMUR._normalValue, _RM->_TIBIA._normalValue, gSpeed);
  _RF->LegWriteVal(_RF->_COXA._normalValue, _RF->_FEMUR._normalValue, _RF->_TIBIA._normalValue, gSpeed);
}

//ROBOT=============================================================PUBLIC METHODS==================================================

//ROBOT=============================================================PRIVATE METHODS==================================================
