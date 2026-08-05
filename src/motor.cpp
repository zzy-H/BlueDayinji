#include <motor.h>
#include <tph.h>
#include <main.h>

void Motor_Feed(uint8_t lines) {
    for(int i=0;i<lines;i++)
        for(int s=0;s<8;s++) goFront1(800);
    digitalWrite(PIN_MOTOR_AP,0);
    digitalWrite(PIN_MOTOR_AM,0);
    digitalWrite(PIN_MOTOR_BP,0);
    digitalWrite(PIN_MOTOR_BM,0);
}
