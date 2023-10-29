#ifndef motorProtection
#define motorProtection

#include "Arduino.h"
#include <SimpleFOC.h>


class mpClass{

  public:
    //Declare all Functions
    void begin();
    void loop();
    void reset();
    float readVbus();
    float readTemp();
    void doMotorProtectCommand(char *command);
    float mpPID(float SP, float PV, float P, float I, float D, float Min, float Max, float Cycle);

    void linkMotor(BLDCMotor*);
    void linkEncoder(Encoder*);
    void linkBLDCDriver6PWM(BLDCDriver6PWM*);
    void linkLowsideCurrentSense(LowsideCurrentSense*);

    float brake_P = 1;
    float brake_I = 10;
    float brake_D = 0;
    float brake_Min = 0;
    float brake_Max = 250;
    float brake_Cycle = 0;
    unsigned long update = 500;

    PIDController brakePID{brake_P,brake_I,brake_D,10000, brake_Max};

    float brakeEngageOffset = 0.25;
    float brakeEngageMaxOffset = 2.0;
    float brakeEngageMaxDisableOffset = 3.0;
    float brakeCutOffPeriod = 250;

    float underVDelayPeriod = 10;
    float underVoffset = 2.5;

    float temperature; 
    float tempCutOff = 60.0;
    float tempDelayPeriod = 500;

    float stallDelayPeriod = 100;
    float stallCurrentOffset = 0.1;
    float stallVoltageOffset = 0.1;

    float overCurrentOffset = 0.05;
    float overVoltageOffset = 0.05;
    float overCVDelayPeriod = 100;

    int mVelCnt = 6;
    float mVelOffset = 1;
    float mVelPeriod = 250;

    int mAngleCnt = 8;
    float mAngleOffset = 0.1;
    float mAnglePeriod = 500;

    int mCurrentQcnt = 10;
    float mCurrentQoffset = 1.5;
    float mCurrentQperiod = 100;

    int mCurrentDcnt = 10;
    float mCurrentDoffset = 1.5;
    float mCurrentDperiod = 100;

    int mVoltageQcnt = 10;
    float mVoltageQoffset = 1.5;
    float mVoltageQperiod = 200;

    int mVoltageDcnt = 10;
    float mVoltageDoffset = 1.5;
    float mVoltageDperiod = 200;

    class internal{
      public:
        bool oscillating(float input, int cnt, float offset, float period);
        float input;
        int cnt;
        float offset;
        float period;
      private:
        bool out;
        bool outNew;
        bool init;
        float oldValue;
        int cntUp;
        int cntDown;
        bool dirUp;
        bool dirDown;
        bool dirUpOld;
        bool dirDownOld;
        bool osOscillating;
    };

  private:

    float VBus;
    float pwmPID;
    float pwmDuty;

    bool motorVoscillation; 
    bool motorAoscillation; 
    bool currentQoscillation;
    bool currentDoscillation;
    bool voltageQoscillation;
    bool voltageDoscillation;

    byte brakeMode;
    #define brakeOff 1
    #define brakeEngage 2
    #define brakeEngageMax 3
    #define brakeEngageMaxDisableMotor 4 

    BLDCMotor* motor;
    Encoder* encoder;
    BLDCDriver6PWM* driver;
    LowsideCurrentSense* currentSense;



        

};
#endif