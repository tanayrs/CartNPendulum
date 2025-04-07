#include <Wire.h>
#include <Encoder.h>
#include <TrivikramEncoder.h>
#include <CytronMotorDriver.h>
#include <elapsedMillis.h>

/* MPU Initialization*/
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
float t,lastt,dt;
float looptime_print;
int c = 0;   //counter for imu error calculation

/* Loop Time Definition */
float LoopTimer;

/* Encoder Parameters */
#define wheelMotorPPR 2264/4
#define pi 3.141592
#define wheelRadius 0.0525

//these pins can not be changed 2/3 are special pins (Interrupt Capable)
#define loopTimeConstant 10000 // In micros
elapsedMicros loopTimeMicros;
const double loopTimeConstSec = loopTimeConstant*1e-6f;
double sampling_time = loopTimeConstSec;
double vel_cutoff_freq = 1000;
#define encoderPin1 2
#define encoderPin2 3
Encoder wheelEnc(encoderPin1, encoderPin2);
EncoderDataProcessor wheelData(wheelMotorPPR, 0, true, false, vel_cutoff_freq, sampling_time); 
CytronMD wheelMotor(PWM_DIR, 6, 8);

// Positon and velocity
float position;
float velocity;
float last_position;

volatile int lastEncoded = 0;
volatile long encoderValue = 0;
volatile long encoder_x=0;

long lastencoderValue = 0;

int lastMSB = 0;
int lastLSB = 0;

int last_broadcast = 0;

// Deadband for Motor Inputs //
float deadzone=0;

/* PID Setup */

// Offsets For Balance Position //
float target_roll=0;
float target_enc=0;
float roll_offset=-0.6;
float int_pos = 0;

// Offsets for Position Controller //
float target_position = 0.1;
float last_position_error = 0;

// Error Initialisation //
float error_roll=0;
float last_error_roll=0;
float error_enc=0;
float last_error_enc=0;

/* PID Setup */

// Gains for Roll //
float Kp=25;//5;//16;           // theta gain
float Kd=0;//1.2;//1.5;      // theta dot gain
float Ki=0;

// Gains for Encoder //
float Kd_wheel=0;//4.5;            // x_dot gain

// Gains for Roll //
float Kp_pos=1;//5;//16;           // theta gain
float Kd_pos=0;//1.2;//1.5;      // theta dot gain
float Ki_pos=0;

// Error Function Initialisation //
float u=0;
float int_lean=0;

// PWM Value Initialisation //
float pwm=0;

// Sign of Roll and Previous Roll for Integral Wind-Up //
int sgnRoll, sgnPrevRoll;

float prev_time;
int deadband_sign, prev_input_sign;

/* Motor Pin Definition */
#define dir_pin 8
#define pwm_pin 6

const int wheelStaticInc = 110;
const int wheelStaticDec = -106.1;
const int wheelKineticInc = -81;
const int wheelKineticDec = 110.72;