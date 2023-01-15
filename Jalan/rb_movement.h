/*
  rb_movement.cpp - Reignblaze Movement Header (EXTERN) file for Arduino.
  Dynamixel AX-12 / AX-18 Servo, Leg, and Inverse Kinematics library for Teensy 3.1/3.2
  Copyright (c) 2019 Irfan Budi Satria (KRPAI TRUI).  All rights reserved.
*/

// ensure this library description is only included once
#ifndef RB_MOVEMENT_h
#define RB_MOVEMENT_h

void ResetSiklus();
void SiklusTripod(Robot myRobot, int gSpeed, int type, byte siklus, float FEMANG, float TIBANG);    //Berbeda dg rb_movement yg digunakan Tes_Jalan
void InitJalan();
void ForwardTripod(Robot myRobot, int gSpeed, int steps, float FEMANG, float TIBANG);               //Berbeda dg rb_movement yg digunakan Tes_Jalan
void ReverseTripod(Robot myRobot, int gSpeed, int steps, float FEMANG, float TIBANG);               //
void RotationRightTripod(Robot myRobot, int gSpeed, int steps, float FEMANG, float TIBANG);         //
void RotationLeftTripod(Robot myRobot, int gSpeed, int steps, float FEMANG, float TIBANG);          //

void ForwardTripodV2(Robot myRobot, int gSpeed, float FEMANG, float TIBANG);                        //                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
void ReverseTripodV2(Robot myRobot, int gSpeed, float FEMANG, float TIBANG);                        //
void RotationRightTripodV2(Robot myRobot, int gSpeed, float FEMANG, float TIBANG);                  //
void RotationLeftTripodV2(Robot myRobot, int gSpeed, float FEMANG, float TIBANG);                   //

void Lifting1(Robot myRobot, int gSpeed, int type);
void Lifting2(Robot myRobot, int gSpeed, int type);

//=======================start Perbedaan dg Tes_Jalan=======================//
void Propelling1AD(Robot myRobot, int gSpeed, int type, float FEMANG, float TIBANG);
void Propelling2AD(Robot myRobot, int gSpeed, int type, float FEMANG, float TIBANG);
void Propelling1BD(Robot myRobot, int gSpeed, int type, float FEMANG, float TIBANG);
void Propelling2BD(Robot myRobot, int gSpeed, int type, float FEMANG, float TIBANG);
//=======================end Perbedaan dg Tes_Jalan=======================//

void RotationRightSmooth(Robot myRobot, int gSpeed, int steps);
void RotationLeftSmooth(Robot myRobot, int gSpeed, int steps);


#endif
