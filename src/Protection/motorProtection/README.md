## **Motor Protections**

### **This library uses the SimpleFOC commander interface with custom commands**

There is a range of functionality to help shutdown a motor if it becomes unstable, use at your own risk!

In order to fully use this you need current sensing, Bus Voltage Sensing and a braking resistor, you need to build an H-Bridge to dump high bus voltage through the resistor

### **Commands:**

**PR** Reboot microcontroller (used for testing and debugging)

**PD** disable motor

**PE** enable motor

**PV** read bus Voltage

**PT** read temperature

enable simpleFOC debug to get serial console readout `SimpleFOCDebug::enable(&Serial);`

### **How to use:**


```
#include "Protection\motorProtection\motorProtection.h"

//Motor Protection Instance 
mpClass mProtection;

void doProtection(char *cmd){
  mProtection.doMotorProtectCommand(cmd);
}
```

**setup:**

```
//Motor Protection Settings
//Link all objects for motor protection class
mProtection.linkMotor(&motor);
//mProtection.linkEncoder(&encoder);
mProtection.linkBLDCDriver6PWM(&driver);
mProtection.linkLowsideCurrentSense(&currentSense);
//Protection update rate in microseconds
mProtection.update = 500;
//Brake PID Paremeters
mProtection.brake_P = 1;
mProtection.brake_I = 10;
mProtection.brake_D = 0;
mProtection.brake_Min = 0;
mProtection.brake_Max = 252;
mProtection.brake_Cycle = 250;
//Brake Trigger (driver power supply + Offset amount above vbus)
mProtection.brakeEngageOffset = 0.5;
mProtection.brakeEngageMaxOffset = 2.0;
mProtection.brakeEngageMaxDisableOffset = 3.0;
mProtection.brakeCutOffPeriod = 500;
//VBus Under Voltage
mProtection.underVDelayPeriod = 1;
mProtection.underVoffset = 1.5;
//Temperature Settings
mProtection.tempCutOff = 60.0;
mProtection.tempDelayPeriod = 500;
//Stall Detection
mProtection.stallCurrentOffset = 0.1;
mProtection.stallVoltageOffset = 0.1;
mProtection.stallDelayPeriod = 100;
//Over Current/Voltage Protection
mProtection.overCurrentOffset = 0.15;
mProtection.overVoltageOffset = 0.5;
mProtection.overCVDelayPeriod = 100;
//Oscillating Detection
//Shaft Velocity 
mProtection.mVelCnt = 1;
mProtection.mVelOffset = 0.5;
mProtection.mVelPeriod = 500;
//Shaft Angle
mProtection.mAngleCnt = 1;
mProtection.mAngleOffset = 0.1;
mProtection.mAnglePeriod = 500;
//Current Q
mProtection.mCurrentQcnt = 2;
mProtection.mCurrentQoffset = 0.5;
mProtection.mCurrentQperiod = 500;
//Current D
mProtection.mCurrentDcnt = 2;
mProtection.mCurrentDoffset = 0.5;
mProtection.mCurrentDperiod = 500;
//Voltage Q
mProtection.mVoltageQcnt = 2;
mProtection.mVoltageQoffset = 0.5;
mProtection.mVoltageQperiod = 500;
//Voltage D
mProtection.mVoltageDcnt = 2;
mProtection.mVoltageDoffset = 0.5;
mProtection.mVoltageDperiod = 500;
//Add Cammand for Motor Protection
command.add('P', doProtection, (char*)"Motor Protection");
//Call Motor Protection Setup
mProtection.begin();
```

**loop:**

```
//Call after motor.loopFOC(); and motor.move(); 
//Motor Protection Loop
mProtection.loop();
```