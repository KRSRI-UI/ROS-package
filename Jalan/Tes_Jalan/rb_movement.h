/*
  rb_movement.cpp - Reignblaze Movement Header (EXTERN) file for Arduino.
  Dynamixel AX-12 / AX-18 Servo, Leg, and Inverse Kinematics library for Teensy 3.1/3.2
  Copyright (c) 2019 Irfan Budi Satria (KRPAI TRUI).  All rights reserved.
*/

// ensure this library description is only included once
#ifndef RB_MOVEMENT_h
#define RB_MOVEMENT_h

void ResetSiklus();
void SiklusTripod(Robot myRobot, int gSpeed, int type, byte siklus, int tipe);
void InitJalan();
void ForwardTripod(Robot myRobot, int gSpeed, int steps, int tipe);
void ReverseTripod(Robot myRobot, int gSpeed, int steps);
void RotationRightTripod(Robot myRobot, int gSpeed, int steps);
void RotationLeftTripod(Robot myRobot, int gSpeed, int steps);

void ForwardTripodV2(Robot myRobot, int gSpeed);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
void ReverseTripodV2(Robot myRobot, int gSpeed);  
void RotationRightTripodV2(Robot myRobot, int gSpeed);
void RotationLeftTripodV2(Robot myRobot, int gSpeed);

void Lifting1(Robot myRobot, int gSpeed, int type);
void Lifting2(Robot myRobot, int gSpeed, int type);
void Propelling1A(Robot myRobot, int gSpeed, int type);
void Propelling2A(Robot myRobot, int gSpeed, int type);
void Propelling1B(Robot myRobot, int gSpeed, int type);
void Propelling2B(Robot myRobot, int gSpeed, int type);

void Propelling1AD(Robot myRobot, int gSpeed, int type);
void Propelling2AD(Robot myRobot, int gSpeed, int type);
void Propelling1BD(Robot myRobot, int gSpeed, int type);
void Propelling2BD(Robot myRobot, int gSpeed, int type);

void RotationRightSmooth(Robot myRobot, int gSpeed, int steps);
void RotationLeftSmooth(Robot myRobot, int gSpeed, int steps);


#endif
