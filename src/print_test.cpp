#include <print_test.h>
#include <tph.h>
#include <main.h>
#include <img.h>

float batVoltage = 0.0f;
int PRINTER_BATTERY = 0;

void Battery_Config(void) {
    analogSetPinAttenuation(PIN_BATV, ADC_11db);
}

void BatteryPower(void) {
    int adc = 0;
    for(int i=0;i<10;i++) adc+=analogRead(PIN_BATV);
    adc/=10;
    batVoltage=(adc*3.6f*2.0f)/4095.0f;
    PRINTER_BATTERY=(int)((batVoltage-3.0f)/1.2f*100);
    if(PRINTER_BATTERY>100) PRINTER_BATTERY=100;
    if(PRINTER_BATTERY<0) PRINTER_BATTERY=0;
    Serial.printf("BAT:%.2fV %d%%\r\n",batVoltage,PRINTER_BATTERY);
}

void Print_test(void) {
    printDataCount=0;
    for(uint32_t i=0;i<IMG_BYTES&&i<PRINT_DATA_MAX;i++) {
        printData[i]=img_data[i];
        printDataCount++;
    }
    Serial.printf("[TEST] Load %lu bytes\n",printDataCount);
    startPrint();
    goFront(20,800);
    digitalWrite(PIN_MOTOR_AP,0);
    digitalWrite(PIN_MOTOR_AM,0);
    digitalWrite(PIN_MOTOR_BP,0);
    digitalWrite(PIN_MOTOR_BM,0);
}
