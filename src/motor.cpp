#include <motor.h>
#include <tph.h>
#include <main.h>

/* ==================================================================
 * 步进电机控制（motor.cpp）
 *
 * 职责：送纸/走纸控制
 *   - Motor_Feed：走指定行数（每行 8 拍）
 *   - 实际步进逻辑（goFront1/goFront）在 tph.cpp（因为打印时也要走纸）
 * ================================================================== */

/* 走纸：走指定行数
 * @param lines 行数（每行走 8 拍——比打印时的 4 拍多，用于快速进纸/切纸） */
void Motor_Feed(uint8_t lines) {
    for(int i=0;i<lines;i++)
        for(int s=0;s<8;s++) goFront1(800);   // 每行 8 拍，每拍间隔 800µs
    // 走完停电机（防止线圈持续通电发热）
    digitalWrite(PIN_MOTOR_AP,0);
    digitalWrite(PIN_MOTOR_AM,0);
    digitalWrite(PIN_MOTOR_BP,0);
    digitalWrite(PIN_MOTOR_BM,0);
}
