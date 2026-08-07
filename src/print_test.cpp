#include <print_test.h>
#include <tph.h>
#include <main.h>
#include <img.h>

/* ==================================================================
 * 测试打印 + 电池检测（print_test.cpp）
 *
 * 职责：
 *   1. Print_test：按按键打印内置测试图片（img.cpp 的 img_data）
 *   2. BatteryPower/Battery_Config：电池电压采样 + 电量百分比计算
 * ================================================================== */

float batVoltage = 0.0f;   // 电池电压（V）
int PRINTER_BATTERY = 0;   // 电量百分比（0~100）

/* 电池 ADC 引脚配置：11db 衰减（ESP32 可测 0~3.6V 量程） */
void Battery_Config(void) {
    analogSetPinAttenuation(PIN_BATV, ADC_11db);
}

/* 电池电量检测（每 5 秒调一次）
 * 原理：ADC 采样 → 10 次平均 → 按分压比换算电压 → 映射电量百分比
 * 硬件：3.7V 锂电经分压（×2）到 ESP32 ADC（3.6V 量程，4095 分辨率） */
void BatteryPower(void) {
    int adc = 0;
    for(int i=0;i<10;i++) adc+=analogRead(PIN_BATV);   // 采样 10 次
    adc/=10;                                            // 取平均（去抖动）
    // 电压换算：ADC 值 × (3.6V/4095) × 分压比 2.0
    batVoltage=(adc*3.6f*2.0f)/4095.0f;
    // 电量百分比：3.0V(0%) ~ 4.2V(100%) 线性映射，除以 1.2 是 4.2-3.0 的跨度
    PRINTER_BATTERY=(int)((batVoltage-3.0f)/1.2f*100);
    if(PRINTER_BATTERY>100) PRINTER_BATTERY=100;   // 限幅
    if(PRINTER_BATTERY<0) PRINTER_BATTERY=0;
    Serial.printf("BAT:%.2fV %d%%\r\n",batVoltage,PRINTER_BATTERY);
}

/* 打印内置测试图片（按键触发或 PRINT_TEST_PAGE 命令触发）
 * 把 img.cpp 里的 img_data（图片像素数组）装入打印缓冲，再调 startPrint */
void Print_test(void) {
    printDataCount=0;
    // 拷贝图片数据到打印缓冲（不超过 PRINT_DATA_MAX 上限）
    for(uint32_t i=0;i<IMG_BYTES&&i<PRINT_DATA_MAX;i++) {
        printData[i]=img_data[i];
        printDataCount++;
    }
    Serial.printf("[TEST] Load %lu bytes\n",printDataCount);
    startPrint();              // 开始打印
    goFront(20,800);           // 打印完再送纸 20 步（把纸吐出来）
    digitalWrite(PIN_MOTOR_AP,0);   // 停电机
    digitalWrite(PIN_MOTOR_AM,0);
    digitalWrite(PIN_MOTOR_BP,0);
    digitalWrite(PIN_MOTOR_BM,0);
}
