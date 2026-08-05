#include <motor.h>
#include <tph.h>

SPIClass printerSPI(HSPI);
SPISettings printerSPISettings = SPISettings(1000000, SPI_MSBFIRST, SPI_MODE0);

uint8_t motorPos = 0;
volatile uint8_t onprint = 0;
volatile uint8_t PaperSta = 1;
uint32_t PowerONTime = 0;

uint8_t printData[PRINT_DATA_MAX] __attribute__((aligned(4))) = {0};
uint16_t printDataCount = 0;
uint8_t printDatacache[48] = {0};
uint8_t heat_density = 80;
uint16_t printpin = 24;

const uint8_t shuzu16to2jingzhi[256] = {
    0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4,1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,
    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
    3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,4,5,5,6,5,6,6,7,5,6,6,7,6,7,7,8
};

static const uint8_t motorTable[8][4] = {
    {1,0,0,0},{1,0,1,0},{0,0,1,0},{0,1,1,0},
    {0,1,0,0},{0,1,0,1},{0,0,0,1},{1,0,0,1}
};

void clearSTB(void) {
    digitalWrite(PIN_STB1,0); digitalWrite(PIN_STB2,0);
    digitalWrite(PIN_STB3,0); digitalWrite(PIN_STB4,0);
    digitalWrite(PIN_STB5,0); digitalWrite(PIN_STB6,0);
}

void clearData(void) {
    for(int i=0;i<48;i++) printDatacache[i]=0;
}

void Printer_Config(void) {
    pinMode(PIN_MOTOR_AP, OUTPUT);
    pinMode(PIN_MOTOR_AM, OUTPUT);
    pinMode(PIN_MOTOR_BP, OUTPUT);
    pinMode(PIN_MOTOR_BM, OUTPUT);
    pinMode(PIN_VHEN, OUTPUT);        
    pinMode(PIN_STB1, OUTPUT);
    pinMode(PIN_STB2, OUTPUT);
    pinMode(PIN_STB3, OUTPUT);
    pinMode(PIN_STB4, OUTPUT);
    pinMode(PIN_STB5, OUTPUT);
    pinMode(PIN_STB6, OUTPUT);
    // 去掉 pinMode(PIN_CLK, OUTPUT) 和 pinMode(PIN_DI, OUTPUT)
    pinMode(PIN_LAT, OUTPUT);
    pinMode(PIN_PTEST, INPUT);

    digitalWrite(PIN_LAT,1);
    digitalWrite(PIN_VHEN,0);          
    digitalWrite(PIN_MOTOR_AP,0);
    digitalWrite(PIN_MOTOR_AM,0);
    digitalWrite(PIN_MOTOR_BP,0);
    digitalWrite(PIN_MOTOR_BM,0);
    clearSTB();
}


void goFront1(uint16_t wait) {
    digitalWrite(PIN_MOTOR_AP, motorTable[motorPos][0]);
    digitalWrite(PIN_MOTOR_AM, motorTable[motorPos][1]);
    digitalWrite(PIN_MOTOR_BP, motorTable[motorPos][2]);
    digitalWrite(PIN_MOTOR_BM, motorTable[motorPos][3]);
    if(motorPos==0) motorPos=8;
    --motorPos;
    if(wait>0) delayMicroseconds(wait);
}

void goFront(uint16_t steps, uint16_t wait) {
    for(uint16_t i=0;i<steps;i++) goFront1(wait);
}

void sendData(uint8_t *data) {
    printerSPI.beginTransaction(printerSPISettings);
    for(uint8_t i=0;i<48;i++) {
        printerSPI.transfer(data[i]);
    }
    printerSPI.endTransaction();
    digitalWrite(PIN_LAT, 0);
    delayMicroseconds(1);
    digitalWrite(PIN_LAT, 1);
}

void startPrint(void) {
    onprint=1;
    PowerONTime=millis();
    Serial.printf("[INFO] 打印 %u 行\n", printDataCount/48);
    digitalWrite(PIN_VHEN,1);
    delayMicroseconds(5000);

    for(uint32_t pointer=0;pointer<printDataCount;pointer+=48) {
        int cache=-1;
        while(cache!=0) {
            if(cache==-1) cache=0;
            int pinnumber=0;
            int i;
            for(i=cache;i<48;i++) {
                pinnumber+=shuzu16to2jingzhi[printData[pointer+i]];
                if(pinnumber<=printpin) {
                    printDatacache[i]=printData[pointer+i];
                    if(i==47) cache=0;
                } else { cache=i; break; }
            }
            sendData(printDatacache);
            clearData();
            if(pinnumber!=0) {
                digitalWrite(PIN_STB1,1); digitalWrite(PIN_STB2,1);
                digitalWrite(PIN_STB3,1); digitalWrite(PIN_STB4,1);
                digitalWrite(PIN_STB5,1); digitalWrite(PIN_STB6,1);
                delayMicroseconds(PRINT_TIME * heat_density / 100);
                clearSTB();
                delayMicroseconds(PRINT_TIME_);
            }
        }
        goFront1(800); goFront1(800); goFront1(800); goFront1(0);
        yield();
    }

    digitalWrite(PIN_MOTOR_AP,0); digitalWrite(PIN_MOTOR_AM,0);
    digitalWrite(PIN_MOTOR_BP,0); digitalWrite(PIN_MOTOR_BM,0);
    clearSTB(); clearData();
    printDataCount=0;
    digitalWrite(PIN_VHEN,0);
    Serial.println("[INFO] 打印完成");
    onprint=0;
}
