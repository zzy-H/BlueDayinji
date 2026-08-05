#ifndef __PRINT_TEST_H__
#define __PRINT_TEST_H__

#include <Arduino.h>

extern volatile uint8_t PaperSta;
extern float batVoltage;
extern int PRINTER_BATTERY;
extern uint32_t PowerONTime;

void Paper_Test(void);
void Battery_Config(void);
void BatteryPower(void);
void Print_test(void);

#endif
