#ifndef __MAIN_H
#define __MAIN_H

#include <Arduino.h>

// === SPI 引脚（打印机头）===
#define PIN_LAT     12
#define PIN_CLK     15
#define PIN_MOSI    13
#define PIN_MISO    -1

// === 电机引脚 ===
#define PIN_MOTOR_AP  22
#define PIN_MOTOR_AM  23
#define PIN_MOTOR_BP  19
#define PIN_MOTOR_BM  21

// === 打印头选通 & 电源 ===
#define PIN_VHEN    17
#define PIN_STB1    14
#define PIN_STB2    27
#define PIN_STB3    26
#define PIN_STB4    25
#define PIN_STB5    33
#define PIN_STB6    32

// === 其他 ===
#define PIN_PTEST   35
#define PIN_BATV    34
#define BUTTON      5
#define LED_PIN     18
#define LED_ON()    digitalWrite(LED_PIN, LOW)
#define LED_OFF()   digitalWrite(LED_PIN, HIGH)

#include <BluetoothSerial.h>
extern BluetoothSerial SerialBT;

#endif
