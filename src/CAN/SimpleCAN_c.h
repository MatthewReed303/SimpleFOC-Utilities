#pragma once

#include <Arduino.h>
#include "SimpleCAN.h"

#if defined(STM32F4xx)

extern "C" void CAN1_RX0_IRQHandler(void); 

void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(SimpleCAN::_hcan);
}

#endif
