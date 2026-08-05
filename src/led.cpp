#include <Arduino.h>
#include <led.h>
#include <main.h>

void Led_Config(void) {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
}
