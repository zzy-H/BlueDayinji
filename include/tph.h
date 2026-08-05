#ifndef __TPH_H__
#define __TPH_H__

#include <Arduino.h>
#include <main.h>
#include <SPI.h>

#define PRINT_TIME    3000
#define PRINT_TIME_   100
#define PRINT_DATA_MAX  30000
#define IMG_LINES    200
#define IMG_BYTES    (IMG_LINES * 48)

extern SPIClass printerSPI;
extern uint8_t motorPos;
extern volatile uint8_t onprint;
extern volatile uint8_t PaperSta;
extern uint32_t PowerONTime;
extern uint8_t printData[];
extern uint16_t printDataCount;
extern uint8_t printDatacache[];
extern uint8_t heat_density;
extern uint16_t printpin;
extern const uint8_t shuzu16to2jingzhi[256];

void Printer_Config(void);
void goFront1(uint16_t wait);
void goFront(uint16_t steps, uint16_t wait);
void sendData(uint8_t *data);
void clearSTB(void);
void clearData(void);
void startPrint(void);

#endif
