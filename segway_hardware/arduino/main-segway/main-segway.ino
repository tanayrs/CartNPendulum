/******************************************************************************************************************

Segway Balancing Code
By: Vishwas Gajera, Tanay Srinivasa, Jia Bhargava

Functions Called in Setup and Loop are Defined in func.ino:
        - void read_imu()                : Reads roll and pitch from MPU6050 Through Wire
        - void calculate_IMU_error()     : Used to Calibrate MPU6050

**** Encoder Pins ****
Pin 1 - 2
Pin 2 - 3

**** Motor Pins ****
dir_pin 8
pwm_pin 6

**** IMU Pins ****
SCL - A4
SDA - A5
******************************************************************************************************************/

#include "constants.h"

void setup() {
        Serial.begin(9600);

        pinMode(13, OUTPUT);
        digitalWrite(13, HIGH);

        Wire.begin();                      // Initialize comunication
        Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
        Wire.write(0x6B);                  // Talk to the register 6B
        Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
        Wire.endTransmission(true);        //end the transmission

        pinMode(encoderPin1, INPUT_PULLUP);
        pinMode(encoderPin2, INPUT_PULLUP);
        wheelData.update(0, 0, 0);

        sgnRoll = 0;
        sgnPrevRoll = 0;

        prev_time = millis();
        deadband_sign = 1;
        prev_input_sign = 1;
}

void loop() {
        read_imu();
        updateEncoderData();
        encoderValue = wheelEnc.read();
        
        lastt=t;
        t=micros();
        dt=(t-lastt)/1000000;

        // position_controller();
        tilt_controller();
        // deadband_test();
        writeToMotor();

        // if (last_broadcast - millis() > 50){
        //         logData();
        //         last_broadcast = millis();
        // }
        // printState();
        logData();

        while(loopTimeMicros < loopTimeConstant)
                delayMicroseconds(10);
}