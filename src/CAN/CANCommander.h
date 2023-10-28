#ifndef CANCOMMANDER_H
#define CANCOMMANDER_H

#include "communication/Commander.h"
#include "CANDriver.h"

class CANCommander : public Commander
{
  public:

    CANCommander(CANDriver &can, char eol = '\n', bool echo = false);
    void runWithCAN();
    CANDriver* can_driver = nullptr;  
    
  private:
    void print(const float number);
    void print(const int number);
    void print(const char* message);
    void print(const __FlashStringHelper *message);
    void print(const char message);
    void println(const float number);
    void println(const int number);
    void println(const char* message);
    void println(const __FlashStringHelper *message);
    void println(const char message);

};

#endif