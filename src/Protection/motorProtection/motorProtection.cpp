#include "motorProtection.h"

//Motor Protections will disable motor 
//Will self reset fault condition if in safe condition and allow you to enable motor again
//Motor Protection functions Include:
//Bus Overvoltage Braking
//Temperature Protection
//Stall Protection
//Over Current / Voltage Protection
//Oscillation Protection

//If using Trap planner add to runPlannerOnTick function
//Bypass Claculations if Motor Disabled
//if(!motor->enabled) return;

//Link all objects from main ( not all are needed but here for future )
void mpClass::linkMotor(BLDCMotor *motorReference){
  motor = motorReference;
}
void mpClass::linkEncoder(Encoder *encoderReference){
  encoder = encoderReference;
}
void mpClass::linkBLDCDriver6PWM(BLDCDriver6PWM *driverReference){
  driver = driverReference;
}
void mpClass::linkLowsideCurrentSense(LowsideCurrentSense *currentSenseReference){
  currentSense = currentSenseReference;
}

//Reset Arduino Function
void(* resetFunc) (void) = 0; //declare reset function @ address 0

//Create instance for function call per object
mpClass::internal VClass; 
mpClass::internal AClass; 
mpClass::internal CQClass;
mpClass::internal CDClass;
mpClass::internal VQClass;
mpClass::internal VDClass;

void mpClass::doMotorProtectCommand(char *mpCommand){
  // Parse this string for vals
  String commandSrt = String(mpCommand);
  float commandValue;
  switch (mpCommand[0]){
  case 'P':
    // Remove V so can convert to a float
    commandSrt = commandSrt.substring(1);
    commandValue = commandSrt.toFloat();
    brakePID.P = commandValue;
    brakePID.reset();
    Serial.printf("Brake P Gain: %f \n", brakePID.P);
  break;
  case 'I':
    // Remove V so can convert to a float
    commandSrt = commandSrt.substring(1);
    commandValue = commandSrt.toFloat();
    brakePID.I = commandValue;
    brakePID.reset();
    Serial.printf("Brake I Gain: %f \n", brakePID.I);
  break;
  case 'R':
    reset();
  break;
  case 'D':
    motor->disable();
  break;
  case 'E':
    motor->enable();
  break;
  case 'V':
    Serial.printf("Bus Voltage Reading: %f \n", readVbus());
  break;
  case 'T':
    Serial.printf("Temperature: %f \n", readTemp());
  break;
  }
}

void mpClass::reset(){

  //TODO add other boards
  #if defined(STM32G4xx)
    NVIC_SystemReset(); //STM32 Reboot
  #elif defined(__AVR_Atmega32U4__)
    resetFunc();
  #endif 
}

bool mpClass::internal::oscillating(float input, int cnt, float offset, float period){
  if (!init){
    oldValue = fabs(input);
    init = true;
  }
  if (fabs(input) > (oldValue + offset)){
    dirUp = true;
    dirDown = false;
  }
  else if (fabs(input) < (oldValue - offset)){
    dirUp = false;
    dirDown = true;
  }
  else {
    dirUp = false;
    dirDown = false;
  }
  oldValue = fabs(input);

  if (dirUp && !dirUpOld) { 
    cntUp++;
  }
  else if (dirDown && !dirDownOld) {
    cntDown++;
  }
  dirUpOld = dirUp; 
  dirDownOld = dirDown; 

  if (cntUp >= cnt && cntDown >= cnt){
    if(!osOscillating){
      out = true;
      osOscillating = true;
    }
    else{
      out = false;
    }
  }
  else{
    out = false;
  } 
  static long delayOscillating = millis();
  if (millis() - delayOscillating > period) {
    cntUp = 0;
    cntDown = 0;
    out = false;
    osOscillating = false;
    delayOscillating = millis(); 
  }
  return out;
}

float mpClass::readVbus(){
  float vbus = _readADCVoltageLowSide(A_VBUS, currentSense->params);
  #if defined(STM32G4xx)            
    vbus = vbus * 10.7711;
  #endif
  return vbus;
}

float mpClass::readTemp(){
  float tADC = _readADCVoltageLowSide(A_TEMPERATURE, currentSense->params);            
  static const float ResistorBalance = 4700.0;  
  static const float Beta  = 3425.0F;
  static const float RoomTempI = 1.0F/298.15F; //[K]
  static const float Rt = ResistorBalance * ((3.3F / tADC)-1);
  static const float R25 = 10000.0F;
  float temp = 1.0F/((log(Rt/R25)/Beta)+RoomTempI);
	temp = temp - 273.15;
  return temp;
}

void mpClass::begin() {
  driver->voltage_power_supply = readVbus();
  SIMPLEFOC_DEBUG("Driver Power Supply Voltage: ", driver->voltage_power_supply);
  motor->voltage_limit = driver->voltage_power_supply;
  motor->PID_current_q.limit = motor->voltage_limit;
  motor->PID_current_d.limit = motor->voltage_limit;
  //Update PID Values
  brakePID.P = brake_P;
  brakePID.I = brake_I;
  brakePID.D = brake_D;
  brakePID.limit = brake_Max_PWM;
}

void mpClass::loop(){

  //Read Bus voltage
  VBus = readVbus();
  //Call PID
  //pwmPID = mpPID(driver->voltage_power_supply, VBus, brake_P, brake_I, brake_D, brake_Min, brake_Max, brake_Cycle);
  pwmPID = brakePID(driver->voltage_power_supply - VBus);
   
  //Disable Motor If bus voltage to high, otherwise let's dump it through brake resistor
  static unsigned long previousMicros;
  if (micros() - previousMicros >= update) {
    previousMicros = micros(); 

    static bool osBrake;
    static bool osBrakeMode;
    static bool osBrakeModeOff;
    static bool osBrakePWM;
    static bool osBrakeMax;
    static bool osBrakeSafe;
    static bool osPwmTimeout;
    if (VBus > (driver->voltage_power_supply + brakeEngageMaxDisableOffset) && (motor->shaft_velocity > 0.01) && motor->enabled) {
      static long delay = millis();
      if (millis() - delay > brakeCutOffPeriod && !osBrake) {
        brakeMode = brakeEngageMaxDisableMotor;
        SIMPLEFOC_DEBUG("Bus Voltage to high, Disable Motor: ", VBus);
        osBrake = true;
        osBrakeModeOff = false;
        osBrakeSafe = false;
        osPwmTimeout = false;
        delay = millis(); 
      }
    }
    else if (VBus > (driver->voltage_power_supply + brakeEngageMaxOffset) && (motor->shaft_velocity > 0.01) && motor->enabled) {
      //if(!osBrakeMax){
      //  brakeMode = brakeEngageMax;
      //  SIMPLEFOC_DEBUG("Bus Voltage High, Max PWM: ", VBus);
      //  osBrakeMax = true;
      //  osBrakeModeOff = false;
      //  osBrakeSafe = false;
      //  osPwmTimeout = false;
      //}
      static long delay = micros();
      if (micros() - delay > brakeVbusDelay && !osBrakeMax) {
        brakeMode = brakeEngageMax;
        SIMPLEFOC_DEBUG("Bus Voltage High, Max PWM: ", VBus);
        osBrakeMax = true;
        osBrakeModeOff = false;
        osBrakeSafe = false;
        osPwmTimeout = false;
        delay = micros(); 
      }
    }
    else if (VBus > (driver->voltage_power_supply + brakeEngageOffset) && (motor->shaft_velocity > 0.01) && motor->enabled) {
      if(!osBrakePWM){
        brakeMode = brakeEngage;
        //SIMPLEFOC_DEBUG("Bus Voltage High, PID PWM: ", VBus);
        osBrakePWM = true;
        osBrakeModeOff = false;
        osBrakeSafe = false;
        osPwmTimeout = false;
      }
    }

    if (VBus <= (driver->voltage_power_supply + 0.25) && (motor->shaft_velocity < 0.01)) {
      static long delay = millis();
      if (millis() - delay > brakeSafePeriod) {
        brakeMode = brakeOff;
        osBrakeMode = false;
        osBrake = false;
        osBrakePWM = false;
        osBrakeMax = false;
        delay = millis(); 
      }
    }
    else if (pwmDuty > 0) {
      static long delay = millis();
      if (millis() - delay > pwmTimeout) {
        brakeMode = pwmTimeoutBrakeOff;
        osBrake = false;
        osBrakePWM = false;
        osBrakeMax = false;
        delay = millis(); 
      }
    }
    switch(brakeMode)
    {
      case brakeOff:
        pwmDuty = 0;
        if(!osBrakeModeOff){
          SIMPLEFOC_DEBUG("Bus Voltage Safe, Switch off Brake: ", VBus);
          osBrakeModeOff = true;
        }
      break;

      case brakeEngage:
        pwmDuty = fabs(pwmPID);
        if(!osBrakeMode){
          //SIMPLEFOC_DEBUG("Brake PWM Duty Cycle: ", pwmDuty);
          osBrakeMode = true;
        }
      break;

      case brakeEngageMax:
        pwmDuty = brake_Max_PWM;
        if(!osBrakeMode){
          SIMPLEFOC_DEBUG("Brake MAX PWM Duty Cycle: ", pwmDuty);
          osBrakeMode = true;
        }
      break;

      case brakeEngageMaxDisableMotor:
        pwmDuty = brake_Max_PWM;
        motor->disable();
        if(!osBrakeMode){
          SIMPLEFOC_DEBUG("Brake MAX Over Voltage Disable Motor: ", pwmDuty);
          osBrakeMode = true;
        }   
      break;

      case pwmTimeoutBrakeOff:
        pwmDuty = 0;
        //motor->disable();
        if(!osBrakeMode){
          SIMPLEFOC_DEBUG("Brake PWM Timeout, shutdown before burning out braking resistor: ", pwmDuty);
          osBrakeMode = true;
        }
      break;
    }

    //Block PWM to Brake if Motor Disabled
    if (motor->enabled) {
      analogWriteFrequency(2000);
      analogWriteResolution(12);
      analogWrite(PA15_ALT1, pwmDuty);
      //TODO get the _writeDutyCycle1PWM working
      //Should not be using analogWrite
      //void* pwm1params;
      //pwm1params = _configure1PWM(1000, A_PWM);
      //_writeDutyCycle1PWM(0.5, pwm1params);
    }

    //VBus Under Voltage
    static bool osUnderVoltage;
    if (VBus < (driver->voltage_power_supply - underVoffset)){
      static long delay = millis();
      if (millis() - delay > underVDelayPeriod && !osUnderVoltage) {
        SIMPLEFOC_DEBUG("VBus Under Voltage, Disable Motor: ", VBus);
        motor->disable();
        delay = millis(); 
        osUnderVoltage = true;
      } 
    }
    else{
      osUnderVoltage = false;
    }

    static bool osTemp;
    temperature = readTemp();
    if (temperature > tempCutOff){
      static long delay = millis();
      if (millis() - delay > tempDelayPeriod && !osTemp) {
        motor->disable();
        SIMPLEFOC_DEBUG("Driver Over Temperature, Disable Motor: ", temperature);
        delay = millis();
        osTemp = true; 
      }
    }
    else{
      osTemp = false;
    }

    //Detect if motor has stalled and shutoff controller
    static bool osStall;
    if (fabs(motor->current.q) >= (motor->current_limit - stallCurrentOffset)) {
      if (fabs(motor->shaft_velocity) < 0.05){
        static long delay = millis();
        if (millis() - delay > stallDelayPeriod && !osStall) {
          SIMPLEFOC_DEBUG("Motor Stalled Current, Disable Motor: ", motor->current.q);
          motor->disable();
          delay = millis(); 
          osStall = true;
        }
      }
    }
    else if (fabs(motor->voltage.q) >= (motor->voltage_limit - stallVoltageOffset)) {
      if (fabs(motor->shaft_velocity) < 0.05){
        static long delay = millis();
        if (millis() - delay > stallDelayPeriod && !osStall) {
          SIMPLEFOC_DEBUG("Motor Stalled Voltage, Disable Motor: ", motor->voltage.q);
          motor->disable();
          delay = millis(); 
          osStall = true;
        }
      }
    }
    else{
      osStall = false;
    }

    //Over Current and Over Voltage Protection
    static bool osOverCV;
    if ((fabs(motor->current.q) >= (motor->current_limit + overCurrentOffset))){
      static long delay = millis();
      if (millis() - delay > overCVDelayPeriod && !osOverCV) {
        SIMPLEFOC_DEBUG("Motor Over Current, Disable Motor: ", motor->current.q);
        motor->disable();
        delay = millis(); 
        osOverCV = true;
      } 
    }
    else if (fabs(motor->voltage.q) >= (motor->voltage_limit + overVoltageOffset)){
      static long delay = millis();
      if (millis() - delay > overCVDelayPeriod && !osOverCV) {
        SIMPLEFOC_DEBUG("Motor Over Voltage, Disable Motor: ", motor->voltage.q);
        motor->disable();
        delay = millis(); 
        osOverCV = true;
      } 
    }
    else{
      osOverCV = false;
    }

    //Detect if Motor is in an uncontrollable state ie oscillating
    motorVoscillation = VClass.oscillating(VClass.input = motor->shaft_velocity, VClass.cnt = mVelCnt, VClass.offset = mVelOffset, VClass.period = mVelPeriod);
    motorAoscillation = AClass.oscillating(AClass.input = motor->shaft_angle, AClass.cnt = mAngleCnt, AClass.offset = mAngleOffset, AClass.period = mAnglePeriod);
    currentQoscillation = CQClass.oscillating(CQClass.input = motor->current.q, CQClass.cnt = mCurrentQcnt, CQClass.offset = mCurrentQoffset, CQClass.period = mCurrentQperiod);
    currentDoscillation = CDClass.oscillating(CDClass.input = motor->current.d, CDClass.cnt = mCurrentDcnt, CDClass.offset = mCurrentDoffset, CDClass.period = mCurrentDperiod);
    voltageQoscillation = VQClass.oscillating(VQClass.input = motor->voltage.q, VQClass.cnt = mVoltageQcnt, VQClass.offset = mVoltageQoffset, VQClass.period = mVoltageQperiod);
    voltageDoscillation = VDClass.oscillating(VDClass.input = motor->voltage.d, VDClass.cnt = mVoltageDcnt, VDClass.offset = mVoltageDoffset, VDClass.period = mVoltageDperiod);
    if (motorVoscillation   ||
        motorAoscillation   ||
        currentQoscillation || 
        currentDoscillation ||
        voltageQoscillation ||
        voltageDoscillation){
      SIMPLEFOC_DEBUG("Motor Oscillating, Disable Motor");
      motor->disable();
    }
    if (motorVoscillation) SIMPLEFOC_DEBUG("Motor Shaft Velocity Oscillating");
    else if (motorAoscillation) SIMPLEFOC_DEBUG("Motor Shaft Angle Oscillating");
    else if (currentQoscillation) SIMPLEFOC_DEBUG("Motor current Q Oscillating");
    else if (currentDoscillation) SIMPLEFOC_DEBUG("Motor current D Oscillating");
    else if (voltageQoscillation) SIMPLEFOC_DEBUG("Motor voltage Q Oscillating");
    else if (voltageDoscillation) SIMPLEFOC_DEBUG("Motor voltage D Oscillating");

    //Reset PIDs to allow stable restart of the motor and set voltage values to 0
    static bool osReset;
    if (!motor->enabled){
      if (fabs(motor->shaft_velocity) < 0.01 && !osReset){
        static long delay = millis();
        if (millis() - delay > 500) {
          motor->PID_current_q.reset();
          motor->PID_current_d.reset();
          motor->PID_velocity.reset();
          motor->P_angle.reset();
          motor->voltage.q = 0;
          motor->voltage.d = 0;
          motor->current.q = 0;
          motor->current.d = 0;
          if (motor->controller == MotionControlType::angle){
            motor->target = motor->shaft_angle;
          }
          else{
            motor->target = 0;
          }
          delay = millis();
          osReset = true; 
        }
      }
    }
    else{
      osReset = false;
    }
  }
  //Lets update target to angle incase motor have been moved before renable 
  static bool osEnabled;
  static bool init;
  if(!init){
    motor->sensor_offset = motor->sensor_offset + motor->shaft_angle; //Keep track of offset! Never overwrite
    motor->target = 0.0;
    init = true;
  }
  else if(motor->enabled){
    if(!osEnabled){
      if (motor->controller == MotionControlType::angle){
        motor->target = motor->shaft_angle;
      }
      else{
        motor->target = 0.0;
      }
      osEnabled = true;
    }
  }
  else{
    osEnabled = false;
  } 
}
