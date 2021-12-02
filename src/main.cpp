// RemoteXY select connection mode and include library
#define REMOTEXY_MODE__HARDSERIAL

#include <RemoteXY.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <SPI.h>

// RemoteXY connection settings
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 9600

#define PIN_FORWARD_LEFT 2
#define PIN_BACKWARD_LEFT 7
#define PIN_FORWARD_RIGHT 4
#define PIN_BACKWARD_RIGHT 8

// RemoteXY configurate
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,5,0,14,0,81,0,13,178,0,
  130,2,41,19,19,38,138,4,0,47,
  21,7,33,38,26,65,114,90,1,9,
  9,67,1,45,53,11,4,38,26,11,
  1,0,5,12,16,16,137,31,226,172,
  134,0,1,0,5,37,16,16,137,31,
  226,172,135,0,1,0,79,12,16,16,
  137,31,226,172,134,0,1,0,79,37,
  16,16,137,31,226,172,135,0 };

// this structure defines all the variables and events of your control interface
struct {
    // input variables
  int8_t max_speed; // =0..100 slider position
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

  AFMS.begin();

  pinMode(PIN_FORWARD_LEFT, OUTPUT);
  pinMode(PIN_BACKWARD_LEFT, OUTPUT);
  pinMode(PIN_FORWARD_RIGHT, OUTPUT);
  pinMode(PIN_BACKWARD_RIGHT, OUTPUT);

  // set initial speed to 50%
  RemoteXY.max_speed = 50;
  // random light on the topright of UI. no functionality at the moment
  // set to green
  RemoteXY.led_1_g = 255;
}

void loop() {
  RemoteXY_Handler();

  digitalWrite(PIN_FORWARD_LEFT, (RemoteXY.forward_left==0)?LOW:HIGH);
  digitalWrite(PIN_BACKWARD_LEFT, (RemoteXY.backward_left==0)?LOW:HIGH);
  digitalWrite(PIN_FORWARD_RIGHT, (RemoteXY.forward_right==0)?LOW:HIGH);
  digitalWrite(PIN_BACKWARD_RIGHT, (RemoteXY.backward_right==0)?LOW:HIGH);

  int slider_pos = RemoteXY.max_speed;

  // set the speed based on slider
  uint8_t speed = ceil(slider_pos * 2.55);
  // set speed text on UI
  sprintf(RemoteXY.max_speed_text, "%u", slider_pos);

  if (digitalRead(PIN_FORWARD_LEFT) == HIGH) {
    myMotor3->setSpeed(speed);
    myMotor3->run(FORWARD);
    myMotor4->setSpeed(speed);
    myMotor4->run(FORWARD);
  } else if (digitalRead(PIN_BACKWARD_LEFT) == HIGH) {
    myMotor3->setSpeed(speed);
    myMotor3->run(BACKWARD);
    myMotor4->setSpeed(speed);
    myMotor4->run(BACKWARD);
  } else {
    myMotor3->run(RELEASE);
    myMotor4->run(RELEASE);
  }

  if (digitalRead(PIN_FORWARD_RIGHT) == HIGH) {
    myMotor2->setSpeed(speed);
    myMotor2->run(FORWARD);
    myMotor1->setSpeed(speed);
    myMotor1->run(FORWARD);
  } else if (digitalRead(PIN_BACKWARD_RIGHT) == HIGH) {
    myMotor2->setSpeed(speed);
    myMotor2->run(BACKWARD);
    myMotor1->setSpeed(speed);
    myMotor1->run(BACKWARD);
  } else {
    myMotor2->run(RELEASE);
    myMotor1->run(RELEASE);
  }
}
