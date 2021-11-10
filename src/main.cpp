// RemoteXY select connection mode and include library
#define REMOTEXY_MODE__SOFTSERIAL

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <SPI.h>

// RemoteXY connection settings
#define REMOTEXY_SERIAL_RX 2
#define REMOTEXY_SERIAL_TX 3
#define REMOTEXY_SERIAL_SPEED 9600

#include <SoftwareSerial.h>
#include <RemoteXY.h>

// RemoteXY PIN settings
#define PIN_FORWARD_LEFT 7
#define PIN_BACKWARD_LEFT 9
#define PIN_FORWARD_RIGHT 8
#define PIN_BACKWARD_RIGHT 10

// RemoteXY configurate
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,7,0,14,0,90,0,13,178,0,
  130,2,41,34,19,23,138,4,0,47,
  36,7,18,207,26,5,32,35,3,30,
  30,138,26,31,65,114,90,1,9,9,
  67,1,45,53,11,4,207,26,11,1,
  0,5,12,16,16,137,31,226,172,134,
  0,1,0,5,37,16,16,137,31,226,
  172,135,0,1,0,79,12,16,16,137,
  31,226,172,134,0,1,0,79,37,16,
  16,137,31,226,172,135,0 };

// this structure defines all the variables and events of your control interface
struct {
    // input variables
  int8_t max_speed; // =0..100 slider position
  int8_t joystick_1_x; // =-100..100 x-coordinate joystick position
  int8_t joystick_1_y; // =-100..100 y-coordinate joystick position
  uint8_t forward_left; // =1 if button pressed, else =0
  uint8_t backward_left; // =1 if button pressed, else =0
  uint8_t forward_right; // =1 if button pressed, else =0
  uint8_t backward_right; // =1 if button pressed, else =0

    // output variables
  uint8_t led_1_r; // =0..255 LED Red brightness
  uint8_t led_1_g; // =0..255 LED Green brightness
  uint8_t led_1_b; // =0..255 LED Blue brightness
  char max_speed_text[11];  // string UTF8 end zero

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);


void setup() {
  RemoteXY_Init();

  pinMode (PIN_FORWARD_LEFT, OUTPUT);
  pinMode (PIN_BACKWARD_LEFT, OUTPUT);
  pinMode (PIN_FORWARD_RIGHT, OUTPUT);
  pinMode (PIN_BACKWARD_RIGHT, OUTPUT);

  // pinMode(7, INPUT);
  // pinMode(9, INPUT);
  // pinMode(8, INPUT);
  // pinMode(10, INPUT);

  RemoteXY.max_speed = 100;

  AFMS.begin();
}

void loop() {
  RemoteXY_Handler();

  digitalWrite(PIN_FORWARD_LEFT, (RemoteXY.forward_left==0)?LOW:HIGH);
  digitalWrite(PIN_BACKWARD_LEFT, (RemoteXY.backward_left==0)?LOW:HIGH);
  digitalWrite(PIN_FORWARD_RIGHT, (RemoteXY.forward_right==0)?LOW:HIGH);
  digitalWrite(PIN_BACKWARD_RIGHT, (RemoteXY.backward_right==0)?LOW:HIGH);

  int slider_pos = RemoteXY.max_speed;

  RemoteXY.led_1_g = 255;

  // set the speed based on slider
  uint8_t speed = ceil(slider_pos * 2.55);
  sprintf(RemoteXY.max_speed_text, "%u", slider_pos);

  // pin 7 and 9 are the left side motors
  if (digitalRead(7) == HIGH) {
    myMotor1->setSpeed(speed);
    myMotor1->run(FORWARD);
    myMotor4->setSpeed(speed);
    myMotor4->run(FORWARD);

  } else if (digitalRead(9) == HIGH) {
    myMotor1->setSpeed(speed);
    myMotor1->run(BACKWARD);
    myMotor4->setSpeed(speed);
    myMotor4->run(BACKWARD);
  } else {
    myMotor1->setSpeed(0);
    myMotor1->run(RELEASE);
    myMotor4->setSpeed(0);
    myMotor4->run(RELEASE);

  }
  // pin 8 and 10 are the right side motors
  if (digitalRead(8) == HIGH) {
    myMotor2->setSpeed(speed);
    myMotor2->run(FORWARD);
    myMotor3->setSpeed(speed);
    myMotor3->run(FORWARD);

  } else if (digitalRead(10) == HIGH) {
    myMotor2->setSpeed(speed);
    myMotor2->run(BACKWARD);
    myMotor3->setSpeed(speed);
    myMotor3->run(BACKWARD);
  } else {
    myMotor2->setSpeed(0);
    myMotor2->run(RELEASE);
    myMotor3->setSpeed(0);
    myMotor3->run(RELEASE);

  }
}
