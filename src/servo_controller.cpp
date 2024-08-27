#include "pinout.h"
#include "Arduino.h"
#include "ESP32Servo.h"
#include "servo_controller.h"
#include "Fusion.h"

double servXmid = 35;  // middle from 0
double servYmid = 25;

double servXlim = 20;
double servYlim = 20;



Servo servo1;
Servo servo2;
Servo servo3;

void servoSetup() {
    // put code that only needs to run once here
    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);

    servo1.setPeriodHertz(50);
    servo1.attach(SERVO_1, 500, 2400);

    servo2.setPeriodHertz(50);
    servo2.attach(SERVO_2, 500, 2400);

    // servo3.setPeriodHertz(50);
    // servo3.attach(SERVO_3, 500, 2400);

    servo1.write(servXmid);
    servo2.write(servYmid);
}

int servo1Pos = 0;
int servoDir = 1;

uint64_t lastServoUpdateTimeMillis = 0;
uint64_t servoUpdateIntervalMillis = 50;

// Does Thrust Vector Control
void servoLoop(float yaw, float pitch, float roll) {
    // put code that needs to run continuously here (called every time from main's loop function)

    float amplitude = (90 - abs(pitch)) / 90;
    // map yaw to x and y
    float x = amplitude * sin(yaw * PI / 180);
    float y = amplitude * cos(yaw * PI / 180);

    deflectServX(x * servXlim);
    deflectServY(y * servYlim);


}

int b = 1;
int c = -1;
int ha = 1;

void deflectServX(double d){
    if(d > servXlim){
        servo1.write(servXmid + servXlim);
    }
    else if(d < -servXlim){
        servo1.write(servXmid - servXlim);
    }
    else{
        servo1.write(servXmid + d);
    }
}

void deflectServY(double d){
    if(d > servYlim){
        servo2.write(servYmid + servYlim);
    }
    else if(d < -servYlim){
        servo2.write(servYmid - servYlim);
    }
    else{
        servo2.write(servYmid + d);
    }
}

void dance(){
    deflectServX(15 * b);
    deflectServY(15 * c);
    
    if(ha == 1){
        b = b * -1;
    }
    else{
        c = c * -1;
    }
    ha = ha * -1;
    
}