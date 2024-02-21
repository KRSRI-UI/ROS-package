/*
  ax12_rb.h - Reignblaze Servo library for Arduino.
  Copyright (c) 2019 Irfan Budi Satria (KRPAI TRUI).  All right reserved.
*/

// ensure this library description is only included once
#ifndef AX12_RB_h
#define AX12_RB_h

// library interface description
class ax12_rb
{
    // user-accessible "public" interface
  public:
    ax12_rb();
    void attach(byte servoID);
    void attach(byte servoID, int minAngle, int maxAngle);
    void setSpeed(int gSpeed);
    void writeAngle(int angle);
    void writeAngle(int angle, int gSpeed);
    void writeValue(int value, int gSpeed);

    int           _normalValue;
    int           _normalAngle;
    byte          _myServoID;

    //non user accessible
  private:
    word          _myVoltage;
    char          _aszCmdLine[80];
    uint8_t       _iszCmdLine;
    boolean       _fTrackServos = false;

    // Values to use for servo position...
    word          _myServoGoalPos;
    word          _myServoGoalSpeed;

    //Specific Values
    int           _minAngle;
    int           _maxAngle;

    int           _minAngle_DEFAULT = 0;
    int           _maxAngle_DEFAULT = 1023;

    int           _speed_DEFAULT = 256;

    //
    bool          _isActive;
};

class Leg
{
  public:
    Leg(byte COXID, byte FEMID, byte TIBID, int type);
    void init(float COXALENGTH, float FEMURLENGTH, float TIBIALENGTH, int COX_NORMAL_ANG, int FEM_NORMAL_ANG, int TIB_NORMAL_ANG);
    void FK(float COXANG, float FEMANG, float TIBANG);
    void IK(float x, float y, float z);

    void LegWriteAng(float COXANG, float FEMANG, float TIBANG, int gSpeed);
    void LegWriteVal(float COXVAL, float FEMVAL, float TIBVAL, int gSpeed);
    
    ax12_rb _COXA;
    ax12_rb _FEMUR;
    ax12_rb _TIBIA;
    int _type;

    float _COXALENGTH;
    float _FEMURLENGTH;
    float _TIBIALENGTH;

    float _currentX;
    float _currentY;
    float _currentZ;

    float _targetX;
    float _targetY;
    float _targetZ;

    float _currentq1;
    float _currentq2;
    float _currentq3;

    float _targetq1;
    float _targetq2;
    float _targetq3;

  private:
    float FK_X(float COXANG, float FEMANG, float TIBANG);
    float FK_Y(float COXANG, float FEMANG, float TIBANG);
    float FK_Z(float COXANG, float FEMANG, float TIBANG);

    float IK_Q1(float x, float y, float z);
    float IK_Q2(float x, float y, float z, float L);
    float IK_Q3(float x, float y, float z, float L);
};

class Robot {

  public:
    Robot(Leg* LFptr, Leg* LMptr, Leg* LRptr, Leg* RRptr, Leg* RMptr, Leg* RFptr);
    void NormalInit(int gSpeed);
    Leg *_LF, *_LM, *_LR, *_RR, *_RM, *_RF;
};
#endif
