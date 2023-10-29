## **Trapezoidal Trajectory Planner with G-Code**

### **This library uses the SimpleFOC commander interface with custom commands**

There is built in homing using pin or current sensing (If configured in SimpleFOC) The is also a function to find your travel range (run this once and then hard code the distance)

### **Commands:**

**G0** go to 0 (home)

**G100** go to 100 (radians or mm)

**GV100** set velocity to 100mm/s

**GA500** set acceleration and deceleration to 500mm/s/s

**GH** begin homing 

**GC** start max distance calibration

**GD** return current distance from home in mm

**GS** stop homing

**GR** set mode to radians

**GM** set mode to metric mm

To use the calibrate max distance, run homing first **GH** and once done then run calibration **GC**

enable simpleFOC debug to get serial console readout `SimpleFOCDebug::enable(&Serial);`

### **How to use:**


```
#include "G-Code\TrapezoidalPlanner\TrapezoidalPlanner.h"

TrapezoidalPlanner planner(1);

void doPlanner(char *cmd){
  planner.doTrapezoidalPlannerCommand(cmd);
}
```

**setup:**

```
planner.linkMotor(&motor);
command.add('G', doPlanner, (char*)"Motion Planner");

//Homing Paremeters
planner.enableHoming = true;
planner.startupHoming = false;
planner.homingDoneToMove = true; //This flag disables G code movement until homing is done 
planner.homingMode = currentSW;  //use current limit switch
planner.homingDirection = CW;
planner.homingVel = 2; //Rads
planner.homingTimeout = 5000; //Millis
planner.homingCurrent = 1.5; //Motor Amps current.Q
planner.homingOffset = 10; //mm The offset amount from the limit switch
planner.loopPeriod = 1000; //Micros
planner.targetMode = targetMM;
planner.mmPerRevolution = 50; //Measure linear distance for one full rotation for mmPerRev ie: target = 6.2831 measure the moved distance 
planner.maxTravelDistance = 150; //mm use - sign to set direction use calibration to get exact value
planner.Vmax_ = 100;    //Velocity
planner.Amax_ = 500;    //Acceleration
planner.Dmax_ = 500;    //Deceleration
```

**loop:**

```
//Call after motor.loopFOC(); and motor.move(); 
planner.loop();
planner.runPlannerOnTick();
```