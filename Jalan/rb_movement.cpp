/*
  rb_movement.cpp - Reignblaze Movement Sourcefile for Arduino.
  Dynamixel AX-12 / AX-18 Servo, Leg, and Inverse Kinematics library for Teensy 3.1/3.2
  Copyright (c) 2019 Irfan Budi Satria (KRPAI TRUI).  All rights reserved.
*/

#include "BioloidSerial.h"
#include "ax12Serial.h"
#include "ax12_rb.h"
#include <SpeedTrig.h>

#define N_FUNC 3

unsigned long WaktuMajuTripod = 300;            //Berbeda dg rb_movement yg digunakan Tes_Jalan
unsigned long previousMillisMajuTripod = 0;

unsigned long WaktuJalanPutar = 50;
unsigned long previousMillisJalanPutar = 0;

int siklusMaju = 0;
int siklusMundur = 0;
int siklusPutarKiri = 0;
int siklusPutarKanan = 0;
/*
   IMPORTANT NOTES

   RIGHT LEG
    -COXA: minus moves leg forward
    -FEMUR: minus moves leg up
    -TIBIA: minus moves leg outward

   LEFT LEG
    -COXA: plus moves leg forward
    -FEMUR: plus moves leg up
    -TIBIA: plus moves leg outward

*/

void ResetSiklus()
{
  siklusMaju = 0;
  siklusMundur = 0;
  siklusPutarKiri = 0;
  siklusPutarKanan = 0;
}

void Lifting1(Robot myRobot, int gSpeed, int type)
{
  switch (type)
  {
    case 1: //MAJU
      myRobot._RF->LegWriteAng(-11, -23, -27, gSpeed);
      myRobot._LM->LegWriteAng(11, 23, 27, gSpeed);
      myRobot._RR->LegWriteAng(-11, -23, -27, gSpeed);
      break;
    case 2: //MUNDUR
      myRobot._RF->LegWriteAng(11, -23, -27, gSpeed);
      myRobot._LM->LegWriteAng(-11, 23, 27, gSpeed);
      myRobot._RR->LegWriteAng(11, -23, -27, gSpeed);
      break;
    case 3: //PUTER KANAN
      myRobot._RF->LegWriteAng(11, -23, -27, gSpeed);
      myRobot._LM->LegWriteAng(11, 23, 27, gSpeed);
      myRobot._RR->LegWriteAng(11, -23, -27, gSpeed);
      break;
    case 4: //PUTER KIRI
      myRobot._RF->LegWriteAng(-11, -23, -27, gSpeed);
      myRobot._LM->LegWriteAng(-11, 23, 27, gSpeed);
      myRobot._RR->LegWriteAng(-11, -23, -27, gSpeed);
      break;
  }
}

void Lifting2(Robot myRobot, int gSpeed, int type)
{
  switch (type)
  {
    case 1:
      myRobot._LF->LegWriteAng(11, 23, 27, gSpeed);
      myRobot._RM->LegWriteAng(-11, -23, -27, gSpeed);
      myRobot._LR->LegWriteAng(11, 23, 27, gSpeed);
      break;
    case 2:
      myRobot._LF->LegWriteAng(-11, 23, 27, gSpeed);
      myRobot._RM->LegWriteAng(11, -23, -27, gSpeed);
      myRobot._LR->LegWriteAng(-11, 23, 27, gSpeed);
      break;
    case 3: //PUTER KANAN
      myRobot._LF->LegWriteAng(11, 23, 27, gSpeed);
      myRobot._RM->LegWriteAng(11, -23, -27, gSpeed);
      myRobot._LR->LegWriteAng(11, 23, 27, gSpeed);
      break;
    case 4: //PUTER KIRI
      myRobot._LF->LegWriteAng(-11, 23, 27, gSpeed);
      myRobot._RM->LegWriteAng(-11, -23, -27, gSpeed);
      myRobot._LR->LegWriteAng(-11, 23, 27, gSpeed);
      break;
  }
}

//=======================start Berbeda dg rb_movement yg digunakan Tes_Jalan=======================//
void Propelling1AD(Robot myRobot, int gSpeed, int type, float FEMANG, float TIBANG)
{
  int negFEMANG = -1 * FEMANG;
  int negTIBANG = -1 * TIBANG;
  switch (type)
  {
    case 1:
      myRobot._RF->LegWriteAng(-11, FEMANG, TIBANG, gSpeed);     //  pada Propelling1A:
      myRobot._LM->LegWriteAng(11, negFEMANG, negTIBANG, gSpeed);    //  negTIBANG >> 0    dan   TIBANG >> 0
      myRobot._RR->LegWriteAng(-11, FEMANG, TIBANG, gSpeed);     //  void Leg::LegWriteAng(float COXANG, float FEMANG, float TIBANG, int gSpeed) >>> dari ax12_rb.cpp
      break;
    case 2:
      myRobot._RF->LegWriteAng(11, FEMANG, TIBANG, gSpeed);      //
      myRobot._LM->LegWriteAng(-11, negFEMANG, negTIBANG, gSpeed);   //
      myRobot._RR->LegWriteAng(11, FEMANG, TIBANG, gSpeed);      //
      break;
    case 3: //PUTER KANAN
      myRobot._RF->LegWriteAng(11, FEMANG, TIBANG, gSpeed);      //
      myRobot._LM->LegWriteAng(11, negFEMANG, negTIBANG, gSpeed);    //
      myRobot._RR->LegWriteAng(11, FEMANG, TIBANG, gSpeed);      //
      break;
    case 4: //PUTER KIRI
      myRobot._RF->LegWriteAng(-11, FEMANG, TIBANG, gSpeed);     //
      myRobot._LM->LegWriteAng(-11, negFEMANG, negTIBANG, gSpeed);   //
      myRobot._RR->LegWriteAng(-11, FEMANG, TIBANG, gSpeed);     //
      break;
  }
}

void Propelling1BD(Robot myRobot, int gSpeed, int type, float FEMANG, float TIBANG)
{
  int negFEMANG = -1 * FEMANG;
  int negTIBANG = -1 * TIBANG;
  switch (type)
  {
    case 1:
      myRobot._RF->LegWriteAng(11, FEMANG, TIBANG, gSpeed);      //  pada Propelling1B:
      myRobot._LM->LegWriteAng(-11, negFEMANG, negTIBANG, gSpeed);   //  negTIBANG >> 0    dan   TIBANG >> 0
      myRobot._RR->LegWriteAng(11, FEMANG, TIBANG, gSpeed);      //
      break;
    case 2:
      myRobot._RF->LegWriteAng(-11, FEMANG, TIBANG, gSpeed);     //
      myRobot._LM->LegWriteAng(11, negFEMANG, negTIBANG, gSpeed);    //
      myRobot._RR->LegWriteAng(-11, FEMANG, TIBANG, gSpeed);     //
      break;
    case 3: //PUTER KANAN
      myRobot._RF->LegWriteAng(-11, FEMANG, TIBANG, gSpeed);     //
      myRobot._LM->LegWriteAng(-11, negFEMANG, negTIBANG, gSpeed);   //
      myRobot._RR->LegWriteAng(-11, FEMANG, TIBANG, gSpeed);     //
      break;
    case 4: //PUTER KIRI
      myRobot._RF->LegWriteAng(11, FEMANG, TIBANG, gSpeed);      //
      myRobot._LM->LegWriteAng(11, negFEMANG, negTIBANG, gSpeed);    //
      myRobot._RR->LegWriteAng(11, FEMANG, TIBANG, gSpeed);      //
      break;
  }
}

void Propelling2AD(Robot myRobot, int gSpeed, int type, float FEMANG, float TIBANG)
{
  int negFEMANG = -1 * FEMANG;
  int negTIBANG = -1 * TIBANG;
  switch (type)
  {
    case 1:
      myRobot._LF->LegWriteAng(11, negFEMANG, negTIBANG, gSpeed);    //  pada Propelling2A:
      myRobot._RM->LegWriteAng(-11, FEMANG, TIBANG, gSpeed);     //  negTIBANG >> 0    dan   TIBANG >> 0
      myRobot._LR->LegWriteAng(11, negFEMANG, negTIBANG, gSpeed);    //
      break;
    case 2:
      myRobot._LF->LegWriteAng(-11, negFEMANG, negTIBANG, gSpeed);   //
      myRobot._RM->LegWriteAng(11, FEMANG, TIBANG, gSpeed);      //
      myRobot._LR->LegWriteAng(-11, negFEMANG, negTIBANG, gSpeed);   //
      break;
    case 3: //PUTER KANAN
      myRobot._LF->LegWriteAng(11, negFEMANG, negTIBANG, gSpeed);    //
      myRobot._RM->LegWriteAng(11, FEMANG, TIBANG, gSpeed);      //
      myRobot._LR->LegWriteAng(11, negFEMANG, negTIBANG, gSpeed);    //
      break;
    case 4: //PUTER KIRI
      myRobot._LF->LegWriteAng(-11, negFEMANG, negTIBANG, gSpeed);   //
      myRobot._RM->LegWriteAng(-11, FEMANG, TIBANG, gSpeed);     //
      myRobot._LR->LegWriteAng(-11, negFEMANG, negTIBANG, gSpeed);   //
      break;
  }
}

void Propelling2BD(Robot myRobot, int gSpeed, int type, float FEMANG, float TIBANG)
{
  int negFEMANG = -1 * FEMANG;
  int negTIBANG = -1 * TIBANG;
  switch (type)
  {
    case 1:
      myRobot._LF->LegWriteAng(-11, negFEMANG, negTIBANG, gSpeed);    //  pada Propelling2B:
      myRobot._RM->LegWriteAng(11, FEMANG, TIBANG, gSpeed);       //  negTIBANG >> 0    dan   TIBANG >> 0
      myRobot._LR->LegWriteAng(-11, negFEMANG, negTIBANG, gSpeed);    //
      break;
    case 2:
      myRobot._LF->LegWriteAng(11, negFEMANG, negTIBANG, gSpeed);     //
      myRobot._RM->LegWriteAng(-11, FEMANG, TIBANG, gSpeed);      //
      myRobot._LR->LegWriteAng(11, negFEMANG, negTIBANG, gSpeed);     //
      break;
    case 3: //PUTER KANAN
      myRobot._LF->LegWriteAng(-11, negFEMANG, negTIBANG, gSpeed);    //
      myRobot._RM->LegWriteAng(-11, FEMANG, TIBANG, gSpeed);      //
      myRobot._LR->LegWriteAng(-11, negFEMANG, negTIBANG, gSpeed);    //
      break;
    case 4: //PUTER KIRI
      myRobot._LF->LegWriteAng(11, negFEMANG, negTIBANG, gSpeed);     //
      myRobot._RM->LegWriteAng(11, FEMANG, TIBANG, gSpeed);       //
      myRobot._LR->LegWriteAng(11, negFEMANG, negTIBANG, gSpeed);     //
      break;
  }
}
//=======================end Berbeda dg rb_movement yg digunakan Tes_Jalan=======================//

void WaveR(Robot myRobot, int gSpeed, byte siklus)
{
  switch (siklus)
  {
    case 0:
      myRobot._RF->LegWriteAng(23, -23, -27, gSpeed);
      break;

    case 1:
      myRobot._LF->LegWriteAng(0, 0, 0, gSpeed);
      myRobot._RF->LegWriteAng(23, 0, 0, gSpeed);
      myRobot._RM->LegWriteAng(23, -23, -27, gSpeed);
      break;

    case 2:
      myRobot._RF->LegWriteAng(0, 0, 0, gSpeed);
      myRobot._RM->LegWriteAng(23, 0, 0, gSpeed);
      myRobot._RR->LegWriteAng(23, -23, -27, gSpeed);
      break;

    case 3:
      myRobot._RM->LegWriteAng(0, 0, 0, gSpeed);
      myRobot._RR->LegWriteAng(23, 0, 0, gSpeed);
      myRobot._LR->LegWriteAng(23, 23, 27, gSpeed);
      break;

    case 4:
      myRobot._RR->LegWriteAng(0, 0, 0, gSpeed);
      myRobot._LR->LegWriteAng(23, 0, 0, gSpeed);
      myRobot._LM->LegWriteAng(23, 23, 27, gSpeed);
      break;

    case 5:
      myRobot._LR->LegWriteAng(0, 0, 0, gSpeed);
      myRobot._LM->LegWriteAng(23, 0, 0, gSpeed);
      myRobot._LF->LegWriteAng(23, 23, 27, gSpeed);
      break;

    case 6:
      myRobot._LM->LegWriteAng(0, 0, 0, gSpeed);
      myRobot._LF->LegWriteAng(23, 0, 0, gSpeed);
      break;
  }
}

void WaveL(Robot myRobot, int gSpeed, byte siklus)
{
  switch (siklus)
  {
    case 0:
      myRobot._RF->LegWriteAng(-23, -23, -27, gSpeed);
      break;

    case 1:
      myRobot._LF->LegWriteAng(0, 0, 0, gSpeed);
      myRobot._RF->LegWriteAng(-23, 0, 0, gSpeed);
      myRobot._RM->LegWriteAng(-23, -23, -27, gSpeed);
      break;

    case 2:
      myRobot._RF->LegWriteAng(0, 0, 0, gSpeed);
      myRobot._RM->LegWriteAng(-23, 0, 0, gSpeed);
      myRobot._RR->LegWriteAng(-23, -23, -27, gSpeed);
      break;

    case 3:
      myRobot._RM->LegWriteAng(0, 0, 0, gSpeed);
      myRobot._RR->LegWriteAng(-23, 0, 0, gSpeed);
      myRobot._LR->LegWriteAng(-23, 23, 27, gSpeed);
      break;

    case 4:
      myRobot._RR->LegWriteAng(0, 0, 0, gSpeed);
      myRobot._LR->LegWriteAng(-23, 0, 0, gSpeed);
      myRobot._LM->LegWriteAng(-23, 23, 27, gSpeed);
      break;

    case 5:
      myRobot._LR->LegWriteAng(0, 0, 0, gSpeed);
      myRobot._LM->LegWriteAng(-23, 0, 0, gSpeed);
      myRobot._LF->LegWriteAng(-23, 23, 27, gSpeed);
      break;

    case 6:
      myRobot._LM->LegWriteAng(0, 0, 0, gSpeed);
      myRobot._LF->LegWriteAng(-23, 0, 0, gSpeed);
      break;
  }
}

void SiklusTripod(Robot myRobot, int gSpeed, int type, byte siklus, float FEMANG, float TIBANG)     //Berbeda dg rb_movement yg digunakan Tes_Jalan
{
  switch (siklus)
  {
    case 0:
      //LIFTING1
      Lifting1(myRobot, gSpeed, type);
      //PROPELLING2BD                             //
      Propelling2BD(myRobot, gSpeed, type, FEMANG, TIBANG);       //
      break;
    case 1:
      //PROPELLING1AD                             //
      Propelling1AD(myRobot, gSpeed, type, FEMANG, TIBANG);       //
      break;
    case 2:
      //LIFTING2
      Lifting2(myRobot, gSpeed, type);
      //PROPELLING1BD                             //
      Propelling1BD(myRobot, gSpeed, type, FEMANG, TIBANG);       //
      break;
    case 3:
      //PROPELLING2AD                             //
      Propelling2AD(myRobot, gSpeed, type, FEMANG, TIBANG);       //
      break;
  }
}

void ForwardTripod(Robot myRobot, int gSpeed, int steps, float FEMANG, float TIBANG)  //Berbeda dg rb_movement yg digunakan Tes_Jalan
{
  for (int i = 0; i < steps; i++)
  {
    int j = 0;
    while (j < 4)
    {
      if (millis() - previousMillisMajuTripod >= WaktuMajuTripod)
      {
        Serial.print("Sekarang Siklus Maju ke - ");
        Serial.println(j);
        SiklusTripod(myRobot, gSpeed, 1, j, FEMANG, TIBANG);                  //Berbeda dg rb_movement yg digunakan Tes_Jalan
        previousMillisMajuTripod = millis();
        j++;
      }
    }
  }
}

void ForwardTripodV2(Robot myRobot, int gSpeed, float FEMANG, float TIBANG)
{
  if (millis() - previousMillisMajuTripod >= WaktuMajuTripod)
  {
    //Serial.print("Sekarang Siklus Maju ke - ");
    Serial.println(siklusMaju);
    SiklusTripod(myRobot, gSpeed, 1, siklusMaju, FEMANG, TIBANG);                //Berbeda dg rb_movement yg digunakan Tes_Jalan

    if (siklusMaju < 3)
    {
      siklusMaju++;
    }
    else
    {
      siklusMaju = 0;
    }
    previousMillisMajuTripod = millis();
  }
}

void ReverseTripod(Robot myRobot, int gSpeed, int steps, float FEMANG, float TIBANG)
{
  for (int i = 0; i < steps; i++)
  {
    int j = 0;
    while (j < 4)
    {
      if (millis() - previousMillisMajuTripod >= WaktuMajuTripod)
      {
        Serial.print("Sekarang Siklus Mundur ke - ");
        Serial.println(j);
        SiklusTripod(myRobot, gSpeed, 2, j, FEMANG, TIBANG);                    //Berbeda dg rb_movement yg digunakan Tes_Jalan
        previousMillisMajuTripod = millis();
        j++;
      }
    }
  }
}

void ReverseTripodV2(Robot myRobot, int gSpeed, float FEMANG, float TIBANG)
{
  if (millis() - previousMillisMajuTripod >= WaktuMajuTripod)
  {
    Serial.print("Sekarang Siklus Mundur ke - ");
    Serial.println(siklusMundur);
    SiklusTripod(myRobot, gSpeed, 2, siklusMundur, FEMANG, TIBANG);              //Berbeda dg rb_movement yg digunakan Tes_Jalan

    if (siklusMundur < 3)
    {
      siklusMundur++;
    }

    else
    {
      siklusMundur = 0;
    }

    previousMillisMajuTripod = millis();
  }
}

void RotationRightTripod(Robot myRobot, int gSpeed, int steps, float FEMANG, float TIBANG)
{
  for (int i = 0; i < steps; i++)
  {
    int j = 0;

    while (j < 4)
    {
      if (millis() - previousMillisJalanPutar >= WaktuJalanPutar)
      {
        Serial.print("Sekarang Siklus Putar Kanan ke - ");
        Serial.println(j);
        SiklusTripod(myRobot, gSpeed, 3, j, FEMANG, TIBANG);                     //Berbeda dg rb_movement yg digunakan Tes_Jalan
        previousMillisJalanPutar = millis();
        j++;
      }
    }
  }
}

void RotationRightTripodV2(Robot myRobot, int gSpeed, float FEMANG, float TIBANG)
{
  if (millis() - previousMillisMajuTripod >= WaktuMajuTripod)
  {
    Serial.print("Sekarang Siklus Kanan ke - ");
    Serial.println(siklusMaju);
    SiklusTripod(myRobot, gSpeed, 3, siklusMaju, FEMANG, TIBANG);                //Berbeda dg rb_movement yg digunakan Tes_Jalan

    if (siklusMaju < 3)
    {
      siklusMaju++;
    }
    else
    {
      siklusMaju = 0;
    }
    previousMillisMajuTripod = millis();
  }
}

void RotationLeftTripod(Robot myRobot, int gSpeed, int steps, float FEMANG, float TIBANG)
{
  for (int i = 0; i < steps; i++)
  {
    int j = 0;

    while (j < 4)
    {
      if (millis() - previousMillisJalanPutar >= WaktuJalanPutar)
      {
        Serial.print("Sekarang Siklus Putar Kiri ke - ");
        Serial.println(j);
        SiklusTripod(myRobot, gSpeed, 4, j, FEMANG, TIBANG);                     //Berbeda dg rb_movement yg digunakan Tes_Jalan
        previousMillisJalanPutar = millis();
        j++;
      }
    }
  }
}

void RotationLeftTripodV2(Robot myRobot, int gSpeed, float FEMANG, float TIBANG)
{
  if (millis() - previousMillisMajuTripod >= WaktuMajuTripod)
  {
    Serial.print("Sekarang Siklus Kiri ke - ");
    Serial.println(siklusMaju);
    SiklusTripod(myRobot, gSpeed, 4, siklusMaju, FEMANG, TIBANG);                //Berbeda dg rb_movement yg digunakan Tes_Jalan

    if (siklusMaju < 3)
    {
      siklusMaju++;
    }
    else
    {
      siklusMaju = 0;
    }
    previousMillisMajuTripod = millis();
  }
}

void RotationRightSmooth(Robot myRobot, int gSpeed, int steps)
{
  for (int i = 0; i < steps; i++)
  {
    int j = 0;
    while (j < 7)
    {
      if (millis() - previousMillisJalanPutar >= WaktuJalanPutar)
      {
        Serial.print("Sekarang Siklus Putar Kanan ke - ");
        Serial.println(j);
        WaveR(myRobot, gSpeed, j);
        previousMillisJalanPutar = millis();
        j++;
      }
    }
  }
}

void RotationLeftSmooth(Robot myRobot, int gSpeed, int steps)
{
  for (int i = 0; i < steps; i++)
  {
    int j = 0;
    while (j < 7)
    {
      if (millis() - previousMillisJalanPutar >= WaktuJalanPutar)
      {
        Serial.print("Sekarang Siklus Putar Kiri ke - ");
        Serial.println(j);
        WaveL(myRobot, gSpeed, j);
        previousMillisJalanPutar = millis();
        j++;
      }
    }
  }
}
