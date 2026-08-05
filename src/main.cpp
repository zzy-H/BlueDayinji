#include <Arduino.h>
#include <main.h>
#include <led.h>
#include <motor.h>
#include <tph.h>
#include <print_test.h>
#include "bluetooth_protocol.h"

BluetoothSerial SerialBT;

extern uint8_t gotStartByte;
extern uint8_t readpos;
extern PackHeader packHeader;
extern uint8_t dataPack_read[];
extern uint16_t dataPack_read_pos;
extern CRC32_Class crc32;

static uint16_t recv_count = 0;
static uint16_t recv_total = 0;

void setup() {
    Led_Config();
    Printer_Config();
    Battery_Config();
    Serial.begin(115200);

    printerSPI.begin(PIN_CLK, PIN_MISO, PIN_MOSI, -1);

    crc32.init(CRC_KEY);
    SerialBT.begin("MaoPaperang");
    delay(100);
    Serial.println("INIT OK");
}

void loop() {
    while (SerialBT.available()) {
        uint8_t c = SerialBT.read();
        PowerONTime = millis();
        if (!gotStartByte) {
            if (c == START_BYTE) { gotStartByte=1; readpos=0; packHeader.dataLen=0; }
            continue;
        }
        switch (readpos) {
            case 0: packHeader.packType=c; readpos=1; break;
            case 1: packHeader.packIndex=c; readpos=2; break;
            case 2: packHeader.dataLen=c; readpos=3; break;
            case 3:
                packHeader.dataLen |= (uint16_t)c << 8;
                if (packHeader.dataLen==0||packHeader.dataLen>=2048) { readpos=8; }
                else { recv_count=0; recv_total=packHeader.dataLen; dataPack_read_pos=0; readpos=4; }
                break;
            case 4:
                if (packHeader.packType==PRINT_DATA||packHeader.packType==PRINT_DATA_COMPRESS) {
                    if (printDataCount<PRINT_DATA_MAX) printData[printDataCount++]=c;
                } else {
                    if (dataPack_read_pos<2048) dataPack_read[dataPack_read_pos++]=c;
                }
                recv_count++;
                if (recv_count>=recv_total) readpos=8;
                break;
            default:
                readpos++;
                if (readpos>=13) { if(c==END_BYTE) paperang_process_data(); gotStartByte=0; readpos=0; }
                break;
        }
    }
    static bool was=false;
    bool now=SerialBT.hasClient();
    if(now!=was){was=now; if(now) Serial.println("BT CONNECTED!");}
    static uint32_t last_bat=0;
    if(millis()-last_bat>5000){last_bat=millis(); BatteryPower();}
    static uint8_t btn=0;
    if(digitalRead(BUTTON)==0){if(!btn){delay(30); if(digitalRead(BUTTON)==0){btn=1;LED_ON();Print_test();LED_OFF();}}}
    else btn=0;
    delay(1);
}
