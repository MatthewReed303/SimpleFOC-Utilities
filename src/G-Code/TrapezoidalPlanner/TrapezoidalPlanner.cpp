#include <Arduino.h>
#include "TrapezoidalPlanner.h"

//#define __debug

TrapezoidalPlanner::TrapezoidalPlanner(int tickPeriod ){
    plannerPeriod = tickPeriod;
    isTrajectoryExecuting = false;
    return;
}

void TrapezoidalPlanner::linkMotor(BLDCMotor *motorReference){
    motor = motorReference;
}

bool TrapezoidalPlanner::isPlannerMoving(){
    return isTrajectoryExecuting;
}
 
float TrapezoidalPlanner::mm2rads(float mm, float mmPerRev){
    float rads = (_2PI / mmPerRev) * mm;
    return rads;
}
float TrapezoidalPlanner::rads2mm(float rads, float mmPerRev){
    float mm = (rads / _2PI) * mmPerRev;
    mm = round(mm * 100) / 100; //Round to 2x Decimal Places
    return mm;
}

void TrapezoidalPlanner::doTrapezoidalPlannerCommand(char *gCodeCommand){
    #ifdef __debug
        Serial.print("GGode command: G");
        Serial.println(gCodeCommand);
    #endif
    
    // Parse this string for vals
    String commandSrt = String(gCodeCommand);
    float commandValue;
    switch (gCodeCommand[0]){
    case 'V':
        // Remove V so can convert to a float
        commandSrt = commandSrt.substring(1);
        commandValue = commandSrt.toFloat();
        if (targetMode){ //If mm selected
            Serial.printf("Max Velocity mm/s: %f \n", commandValue);
            commandValue = mm2rads(commandValue, mmPerRevolution);  
        }
        else {
            Serial.printf("Max Velocity radians/s: %f \n", commandValue);
        }
        Vmax_ = commandValue;
        // Try calling the planner to use this new velocity value
        // We have to use the current pos, vel, and accel
        // calc_plan_trapezoidal_path_at_start(Xf_, Xi_, Vi_, Vmax_, Amax_, Dmax_);
        calculateTrapezoidalPathParameters(Xf_, motor->shaft_angle, motor->shaft_velocity, Vmax_, Amax_, Dmax_);
        #ifdef __debug
            Serial.print("User wants velocity change. Vmax_: ");
            Serial.println(Vmax_);
        #endif
        break;
    case 'A':
        // Remove A so can convert to a float
        commandSrt = commandSrt.substring(1);
        commandValue = commandSrt.toFloat();
        if (targetMode){ //If mm selected
            Serial.printf("Max Acceleration mm/s/s: %f \n", commandValue);
            commandValue = mm2rads(commandValue, mmPerRevolution);  
        }
        else{
            Serial.printf("Max Acceleration radians/s/s: %f \n", commandValue);
        }
        Amax_ = commandValue;
        Dmax_ = Amax_;
        // Try calling the planner to use this new acceleration value
        // calc_plan_trapezoidal_path_at_start(Xf_, Xi_, Vi_, Vmax_, Amax_, Dmax_);
        calculateTrapezoidalPathParameters(Xf_, motor->shaft_angle, motor->shaft_velocity, Vmax_, Amax_, Dmax_);
        #ifdef __debug
            Serial.print("User wants acceleration change. Amax_: ");
            Serial.println(Amax_);
        #endif 
        break;
    case 'L':
        //  TODO  TODO TODO TODO
        //  TODO  TODO TODO TODO
        break;
    case 'C':
        //Start Max Distance Calibration    
        homingState = calibrateMaxDistance;
        Serial.printf("Claibrating Max Distance Selected: %i \n", calibrateMaxDistance);
        break;
    case 'D':
        //Return Current distance in mm
        Serial.printf("Current Distance in mm: %f \n", rads2mm(motor->shaft_angle, mmPerRevolution));
        break;
    case 'H':
        //Start Homing Sequnce   
        homingState = start;
        Serial.printf("Homing Start Selected: %i \n", start);
        break;
    case 'S':
        //Stop Homing Sequnce   
        homingState = stop;
        Serial.printf("Homing Stop Selected: %i \n", stop);
        break;
    case 'R':
        //Set to Radians
        targetMode = targetRAD;
        Serial.printf("TargetMode Radians: %s \n", targetMode ? "true" : "false");
        break;
    case 'M':
        //Set to Metric mm
        targetMode = targetMM;
        Serial.printf("TargetMode Metric: %s \n", targetMode ? "true" : "false");
        break;
    default:
        commandValue = commandSrt.toFloat();
        #ifdef __debug
            Serial.print("Move to new position (rads): ");
            Serial.println(commandValue);
        #endif 
        // We start moving to the new position
        executeNewPosition(commandValue);
        break;
    }
}

void TrapezoidalPlanner::executeNewPosition(float commandPossition){
    if (homingDone){ //If homing has been done force travel limits
        static float commandPossitionRads;
        static float commandPossitionMM;
        commandPossitionRads = mm2rads(commandPossition, mmPerRevolution);
        commandPossitionMM = commandPossition; //If homing done, means we are already in mm
        //Lets check our homing direction and constrain the movements in the correct direction
        if (homingDirection){ //CW = False, CCW = True
            if (commandPossitionMM < 0 || commandPossitionMM > maxTravelDistance){
                Serial.printf("Invalid Move, Out of Range mm: %f \n", commandPossitionMM);
            }
            else{
                Serial.printf("Move To mm: %f \n", commandPossitionMM);
                startExecutionOfPlannerTo(commandPossitionRads);
            }
        }
        else{
            if (commandPossitionMM > 0 || commandPossitionMM < maxTravelDistance){
                Serial.printf("Invalid Move, Out of Range mm: %f \n", commandPossitionMM);
            }
            else{
                Serial.printf("Move To mm: %f \n", commandPossitionMM);
                startExecutionOfPlannerTo(commandPossitionRads);
            }
        }    
    }
    else if (!homingDoneToMove){
        if (targetMode){ //If mm selected convert to mm
            Serial.printf("No constraints, Move To mm: %f \n", commandPossition);
            commandPossition = mm2rads(commandPossition, mmPerRevolution);
        }
        else {
            Serial.printf("No constraints, Move To radans: %f \n", commandPossition);
        }
        startExecutionOfPlannerTo(commandPossition); 
    }
    else{
        Serial.printf("Invalid, Please Home and Set Max travel Distance or set homingDoneToMove = false \n");
    }
}

void TrapezoidalPlanner::runPlannerOnTick(){
    //Bypass Claculations if Motor Disabled
    if(!motor->enabled || homingActive || calibrationActive) return;
    // This should get entered 100 times each second (100Hz)
    if ((unsigned long)(millis() - plannerTimeStap) > plannerPeriod){
        plannerTimeStap = millis();
        // see if we are in a move or not
        if (isTrajectoryExecuting){
            // we are in a move, let's calc the next position
            float timeSinceStartingTrajectoryInSeconds = (millis() - plannerStartingMovementTimeStamp) / 1000.0f;
            computeStepValuesForCurrentTime(timeSinceStartingTrajectoryInSeconds);
            motor->target = Y_;

            // see if we are done with our move
            if (timeSinceStartingTrajectoryInSeconds >= Tf_){
                // we are done with move
                // motor.monitor_downsample = 0; // disable monitor
                #ifdef __debug
                    Serial.println("Done with move");
                #endif 
                isTrajectoryExecuting = false;
            }
        }
    }
}

float TrapezoidalPlanner::sign(float val){
    if (val < 0)
        return -1.0f;
    if (val == 0)
        return 0.0f;
    // if val > 0:
    return 1.0f;
}

float TrapezoidalPlanner::sign_hard(float val){
    if (val < 0)
        return -1.0f;
    return 1.0f;
}

bool TrapezoidalPlanner::calculateTrapezoidalPathParameters(float Xf, float Xi, float Vi, float Vmax, float Amax, float Dmax){
    float dX = Xf - Xi;                          // Distance to travel
    float stop_dist = (Vi * Vi) / (2.0f * Dmax); // Minimum stopping distance
    float dXstop = sign(Vi) * stop_dist;         // Minimum stopping displacement
    float s = sign_hard(dX - dXstop);            // Sign of coast velocity (if any)
    Ar_ = s * Amax;                              // Maximum Acceleration (signed)
    Dr_ = -s * Dmax;                             // Maximum Deceleration (signed)
    Vr_ = s * Vmax;                              // Maximum Velocity (signed)

    // If we start with a speed faster than cruising, then we need to decel instead of accel
    // aka "double deceleration move" in the paper
    if ((s * Vi) > (s * Vr_)){
        Ar_ = -s * Amax;
    }

    // Time to accel/decel to/from Vr (cruise speed)
    Ta_ = (Vr_ - Vi) / Ar_;
    Td_ = -Vr_ / Dr_;

    // Integral of velocity ramps over the full accel and decel times to get
    // minimum displacement required to reach cuising speed
    float dXmin = 0.5f * Ta_ * (Vr_ + Vi) + 0.5f * Td_ * Vr_;

    // Are we displacing enough to reach cruising speed?
    if (s * dX < s * dXmin){
        // Short move (triangle profile)
        Vr_ = s * sqrt((Dr_ * sq(Vi) + 2 * Ar_ * Dr_ * dX) / (Dr_ - Ar_));
        Ta_ = max(0.0f, (Vr_ - Vi) / Ar_);
        Td_ = max(0.0f, -Vr_ / Dr_);
        Tv_ = 0.0f;
    }
    else{
        // Long move (trapezoidal profile)
        Tv_ = (dX - dXmin) / Vr_;
    }

    // Fill in the rest of the values used at evaluation-time
    Tf_ = Ta_ + Tv_ + Td_;
    Xi_ = Xi;
    Xf_ = Xf;
    Vi_ = Vi;
    yAccel_ = Xi + Vi * Ta_ + 0.5f * Ar_ * sq(Ta_); // pos at end of accel phase

    #ifdef __debug
        Serial.println("--------------------------------- ");
        Serial.println(" Calculated trapezoidal Values:   ");
        Serial.println("     Tf: " + String(Tf_));
        Serial.println("     Ta: " + String(Ta_));
        Serial.println("     Tv: " + String(Tv_));
        Serial.println("     Td: " + String(Td_));
        Serial.println("     --------------------- ");
        Serial.println("     Ar: " + String(Ar_));
        Serial.println("     Vr: " + String(Vr_));
        Serial.println("     Dr: " + String(Dr_));
        Serial.println("     --------------------- ");
        Serial.println("     Xf: " + String(Xf));
        Serial.println("     Xi: " + String(Xi));
    #endif

    return true;
}

void TrapezoidalPlanner::startExecutionOfPlannerTo(float newPos){

    if(!motor->enabled || homingActive || calibrationActive) return;
    // set our global of the new position
    Xf_ = newPos;
    // At this poin we are atarting to move following the trapezoidal profile
    plannerStartingMovementTimeStamp = millis();
    // take the position from SimpleFOC and set it to our start position
    Xi_ = motor->shaft_angle;
    // TODO: If we are being asked to move but are already in motion, we should go with the current velocity, position, and acceleration
    // and keep moving.
    // Now we need to do our calcs before we start moving
    calculateTrapezoidalPathParameters(Xf_, Xi_, Vi_, Vmax_, Amax_, Dmax_);
    motor->target = Y_; // could possibly put this in the method above
    // Tell user how long this move will take
    #ifdef __debug
        Serial.println("Starting to move to a new position");
        Serial.print("Time to complete move (secs):");
        Serial.println(Tf_);
        // Velocity and Accel of move
        Serial.print("Velocity for move: ");
        Serial.print(Vmax_);
        Serial.print(", Acceleration: ");
        Serial.println(Amax_);
    #endif
    // set our global bool so the tick method knows we have a move in motion
    isTrajectoryExecuting = true;
}
void TrapezoidalPlanner::computeStepValuesForCurrentTime(float currentTrajectoryTime){
    // Step_t trajStep;
    if (currentTrajectoryTime < 0.0f){ 
        // Initial Condition
        Y_ = Xi_;
        Yd_ = Vi_;
        Ydd_ = 0.0f;
        //Serial.println("--- Initial Stage ---");
        #ifdef __debug
            Serial.print(String(currentTrajectoryTime) + ",");
            Serial.print(Y_, 5);
            Serial.println();
        #endif
    }
    else if (currentTrajectoryTime < Ta_){ 
        // Accelerating
        Y_ = Xi_ + Vi_ * currentTrajectoryTime + 0.5f * Ar_ * sq(currentTrajectoryTime);
        Yd_ = Vi_ + Ar_ * currentTrajectoryTime;
        Ydd_ = Ar_;
        #ifdef __debug
        // Serial.print("Accelerating  ");
            Serial.print(String(currentTrajectoryTime) + ",");
            Serial.print(Y_, 5);
            Serial.println();
        #endif
    }
    else if (currentTrajectoryTime < Ta_ + Tv_){
        // Coasting
        Y_ = yAccel_ + Vr_ * (currentTrajectoryTime - Ta_);
        Yd_ = Vr_;
        #ifdef __debug
            // Serial.print("Coastting ");
            Serial.print(String(currentTrajectoryTime) + ",");
            Serial.print(Y_, 5);
            Serial.println();
        #endif
    }
    else if (currentTrajectoryTime < Tf_){ 
        // Deceleration
        float td = currentTrajectoryTime - Tf_;
        Y_ = Xf_ + 0.5f * Dr_ * sq(td);
        Yd_ = Dr_ * td;
        Ydd_ = Dr_;
        // Serial.print("Decelerating  ");
        #ifdef __debug
            // Serial.print("Accelerating  ");
            Serial.print(String(currentTrajectoryTime) + ",");
            Serial.print(Y_, 5);
            Serial.println();
        #endif
    }
    else if (currentTrajectoryTime >= Tf_){ // Final Condition
        Y_ = Xf_;
        Yd_ = 0.0f;
        Ydd_ = 0.0f;
        #ifdef __debug
            //Serial.print("--- Final Stage ---");
            Serial.print(String(currentTrajectoryTime) + ",");
            Serial.print(Y_, 5);
            Serial.println();
        #endif
    }
    else{
        // TODO: Error halnling here
    }
    return;
}

//Homing loop
void TrapezoidalPlanner::loop(){
    //If start up failed or motor disabled do not execute homing
    if(!motor->enabled) return;

    static unsigned long previousMicros;
    if (micros() - previousMicros >= loopPeriod) {
        previousMicros = micros();
        //Check if homing is enabled and homing mode is in mm do not home in rads
        if (enableHoming && targetMode) {
            static bool init;
            if(startupHoming && !init){
                init = true;
                homingState = start;
            }
            //Homing Sequence
            static bool osStart;
            static bool osCalibration;
            static bool osMoveOffset;
            switch(homingState){
                case start:
                    if (!osStart){
                        if (!calibrateMaxDistanceEnabled) SIMPLEFOC_DEBUG("Starting Homing Sequence");
                        if (motor->velocity_limit != homingVel){
                            oldVelocityLimit = motor->velocity_limit; //Record original velocity limit
                        }
                        osStart = true;
                        homingActive = true;
                    }
                    motor->velocity_limit = homingVel;
                    motor->controller = MotionControlType::velocity;
                    if(homingDirection){ //True = CCW
                        motor->target = (homingVel * -1); //CCW
                    }
                    else { //False = CW
                        motor->target = homingVel; //CW
                    }
                    if (homingMode){
                        if (fabs(motor->current.q) >= homingCurrent) {
                            if (fabs(motor->shaft_velocity) < 10){
                                static long delay = millis();
                                if (millis() - delay > 10) {
                                    delay = millis();
                                    SIMPLEFOC_DEBUG("Current Limit Switch Activated: ", motor->current.q);
                                    motor->target = 0; 
                                    homingState = moveOffset;
                                }
                            }                           
                        }
                    }
                    else if (homingPinState){
                        SIMPLEFOC_DEBUG("Limit Switch Activated: ", homingPinState);
                        motor->target = 0;
                        homingState = moveOffset;
                    }
                    //static long delayTimeout = millis();
                    //if (millis() - delayTimeout > homingTimeout) {
                    //    delayTimeout = millis();
                    //    SIMPLEFOC_DEBUG("Homing timed out no limit found");
                    //    motor->target = motor->shaft_angle;
                    //    motor->controller = MotionControlType::angle;
                    //    //homingState = failed;
                    //} 
                break;
                case moveOffset:
                    if (!osMoveOffset){
                        motor->target = motor->shaft_angle;
                        motor->P_angle.limit = homingVel;
                        motor->controller = MotionControlType::angle;
                    }
                    if(homingDirection){ //CCW = True (Nevative Angle Value)
                        static float setNewTargetCCW;
                        if (!osMoveOffset){
                            static float mm2radsOffset;
                            mm2radsOffset = mm2rads(homingOffset, mmPerRevolution);
                            setNewTargetCCW = motor->shaft_angle + mm2radsOffset;
                            motor->target = setNewTargetCCW;
                            osMoveOffset = true;
                        }
                        if (motor->shaft_angle > setNewTargetCCW - 0.01 && motor->shaft_angle < setNewTargetCCW + 0.01){ //Within, will be hard to be exact without very good tuning
                            if (!calibrateMaxDistanceEnabled){
                                motor->sensor_offset = motor->sensor_offset + motor->shaft_angle;; //New Zero Point
                                motor->target = 0;
                                homingState = waitMoveOffset;
                            }
                            else{
                                SIMPLEFOC_DEBUG("Calibration Max Distance Offset Complete: ", rads2mm(motor->shaft_angle, mmPerRevolution));
                                homingState = calibrateMaxDistanceDone;
                            }
                        }
                    } 
                    else{ //CW = False (Positive Angle Value)
                        static float setNewTargetCW;
                        if (!osMoveOffset){
                            static float mm2radsOffset;
                            mm2radsOffset = mm2rads(homingOffset, mmPerRevolution);
                            setNewTargetCW = motor->shaft_angle - mm2radsOffset;
                            motor->target = setNewTargetCW;
                            osMoveOffset = true;
                        }
                        if (motor->shaft_angle > setNewTargetCW - 0.01 && motor->shaft_angle < setNewTargetCW + 0.01){ //Within, will be hard to be exact without very good tuning
                            if (!calibrateMaxDistanceEnabled){
                                motor->sensor_offset = motor->sensor_offset + motor->shaft_angle; //New Zero Point
                                motor->target = 0;
                                homingState = waitMoveOffset;                                                          
                            }
                            else{
                                SIMPLEFOC_DEBUG("Calibration Max Distance Offset Complete: ", rads2mm(motor->shaft_angle, mmPerRevolution));
                                homingState = calibrateMaxDistanceDone;
                            }
                        }
                    }
                break;

                case waitMoveOffset:
                    if (motor->shaft_angle > 0 - 0.01 && motor->shaft_angle < 0 + 0.01){ //Wait for sensor_offset to update, shaft angle should now be 0 to confirm homing is done
                        motor->target = 0;
                        SIMPLEFOC_DEBUG("Homing Offset Complete set new Zero: ", motor->shaft_angle);
                        homingState = done;
                    }
                break;

                case calibrateMaxDistance:
                    //Use homing steps to set calibration distance
                    if (!osCalibration){
                        SIMPLEFOC_DEBUG("Calibrate Max Distance Started");
                        if (motor->velocity_limit != homingVel){
                            oldVelocityLimit = motor->velocity_limit; //Record original velocity limit
                        }
                        oldHomingDirection = homingDirection; //Record original homing direction
                        calibrationActive = true;
                        osCalibration = true;
                    }
                    osMoveOffset = false;
                    motor->P_angle.limit = homingVel;
                    //If homing has been done, return to home posistion and measure to end of travel 
                    if (homingDone){
                        if(homingDirection){
                            homingDirection = CW_Homing; //Let's change the sensing direction opposite 
                        }
                        else {
                            homingDirection = CCW_Homing; //Let's change the sensing direction opposite
                        }
                        //Return Home before we start our measurement
                        static bool osReturnHome;  
                        if (!osReturnHome){
                            SIMPLEFOC_DEBUG("Return Home before we begin measuremnts, distance from home: ", motor->shaft_angle);
                            motor->target = 0;
                            osReturnHome = true;
                        }
                        if (motor->shaft_angle > 0 - 0.01 && motor->shaft_angle < 0 + 0.01){ //We are back at home / Zero point
                            calibrateMaxDistanceEnabled = true;
                            homingState = start;
                            osReturnHome = false;
                        }
                    }
                    else{ 
                        homingState = start;
                    }
                break;
                case calibrateMaxDistanceDone:
                    maxTravelDistance = rads2mm(motor->shaft_angle, mmPerRevolution); //Remember to hardcode this value in startup!
                    SIMPLEFOC_DEBUG("Done Calculating Max Distance: ", maxTravelDistance);
                    //Restore to original values
                    motor->velocity_limit = oldVelocityLimit;
                    motor->P_angle.limit = oldVelocityLimit;
                    homingDirection = oldHomingDirection;
                    homingState = exit;
                break;  
                case stop:
                    SIMPLEFOC_DEBUG("Homing has been stopped");
                    if (motor->controller == MotionControlType::angle){
                        motor->target = motor->shaft_angle;
                    }
                    else {
                        motor->target = 0;
                    }
                    homingState = exit;  
                break;  
                case failed:
                    SIMPLEFOC_DEBUG("Homing has failed, no limits found or timeout");
                    if (motor->controller == MotionControlType::angle){
                        motor->target = motor->shaft_angle;
                    }
                    else {
                        motor->target = 0;
                    }
                    homingState = exit;    
                break;
                case done:
                    SIMPLEFOC_DEBUG("Homing Complete!");
                    homingDone = true;
                    if (!calibrationActive){
                        motor->velocity_limit = oldVelocityLimit; //Restore Velocity Limit
                        motor->P_angle.limit = oldVelocityLimit;
                        motor->target = 0;
                        homingState = exit;
                    }
                    else{
                        homingState = calibrateMaxDistance;
                    }
                break;
                case exit:
                    osStart = false; //Reset Oneshot
                    osCalibration = false;
                    osMoveOffset = false;
                    homingActive = false;
                    calibrationActive = false;
                    calibrateMaxDistanceEnabled = false;
                    return;
                break;

            }
        }
    }
}   
