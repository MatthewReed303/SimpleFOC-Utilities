#ifndef __TRAPEZOIDAL_PLANNER__H
#define __TRAPEZOIDAL_PLANNER__H
#include <SimpleFOC.h>

class TrapezoidalPlanner
{
public:
    TrapezoidalPlanner(int);
    void doTrapezoidalPlannerCommand(char *command);
    void linkMotor(BLDCMotor*);
    void runPlannerOnTick();
    bool isPlannerMoving();

    //Homing Parameters
    void begin();
    void loop();
    float mm2rads(float mm, float mmPerRev);
    float rads2mm(float rads, float mmPerRev);

    #define CW false
    #define CCW true
    #define limitSW false
    #define currentSW true

    #define targetRAD false
    #define targetMM true

    byte homingState;
    #define start 1
    #define stop 2
    #define moveOffset 3
    #define waitMoveOffset 4
    #define calibrateMaxDistance 5
    #define calibrateMaxDistanceDone 6
    #define failed 7
    #define done 8
    #define exit 9

    bool enableHoming = false;
    bool beginHoming = false;
    bool startupHoming = false;
    bool homingActive = false;
    bool homingDone = false;
    bool homingDoneToMove = true; //This flag disables G code movement until homing is done
    bool homingPinState =false; 
    bool homingMode = limitSW;  
    bool homingDirection = CW;
    float homingVel = 2; //Rads
    float homingTimeout = 5000; //Millis
    float homingCurrent = 1; //Motor Amps current.Q
    float homingOffset = 10; //mm

    float loopPeriod = 500; //Micros

    bool targetMode = targetRAD;
    float mmPerRevolution = 50; //mm
    float maxTravelDistance = 0; //mm use - sign to set direction 

    bool calibrationActive;
    bool calibrateMaxDistanceEnabled;

    float Vmax_;    // # Velocity max (rads/s)
    float Amax_;    // # Acceleration max (rads/s/s)
    float Dmax_;    // # Decelerations max (rads/s/s)
    
private:
    BLDCMotor* motor;
    unsigned long plannerTimeStap;
    int plannerPeriod; // 1000 / this number = Hz, i.e. 1000 / 100 = 10Hz, 1000 / 10 = 100Hz, 1000 / 5 = 200Hz, 1000 / 1 = 1000hZ
    float Y_;
    float Yd_;
    float Ydd_;
    float Tf_, Xi_, Xf_, Vi_, Ar_, Dr_, Vr_, Ta_, Td_, Tv_, yAccel_;
    unsigned long plannerStartingMovementTimeStamp;
    bool isTrajectoryExecuting;
    float sign(float val);
    float sign_hard(float val);
    void executeNewPosition(float commandPossition);
    bool calculateTrapezoidalPathParameters(float Xf, float Xi, float Vi, float Vmax, float Amax, float Dmax);
    void startExecutionOfPlannerTo(float newPos);
    void computeStepValuesForCurrentTime(float currentTrajectoryTime);

    float setNewTarget;
    float oldVelocityLimit;
    bool oldHomingDirection;
};

#endif