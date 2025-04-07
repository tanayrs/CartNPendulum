#include <Wire.h>

#define encoderPin1 2
#define encoderPin2 3

volatile long wheelTicks = 0;
volatile bool lastState;

void encoderISR() {
    bool state = digitalRead(encoderPin1);
    if (state != lastState) {
        if (digitalRead(encoderPin2) != state) {
            wheelTicks++;
        } else {
            wheelTicks--;
        }
    }
    lastState = state;
}

void setup() {
    Serial.begin(9600);
    pinMode(encoderPin1, INPUT_PULLUP);
    pinMode(encoderPin2, INPUT_PULLUP);
    
    lastState = digitalRead(encoderPin1);
    attachInterrupt(digitalPinToInterrupt(encoderPin1), encoderISR, CHANGE);
}

void loop() {
    Serial.print("wheelTicks: ");
    Serial.println(wheelTicks);
    delay(100);
}
