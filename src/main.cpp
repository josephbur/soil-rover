// RemoteXY select connection mode and include library
#define REMOTEXY_MODE__HARDSERIAL

#include <RemoteXY.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <SPI.h>
#include <Servo.h>
#include "Adafruit_seesaw.h"

// RemoteXY connection settings
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 9600

#define PIN_FORWARD_LEFT 2
#define PIN_BACKWARD_LEFT 7
#define PIN_FORWARD_RIGHT 4
#define PIN_BACKWARD_RIGHT 8

#define SERVO_DELAY 1000
#define SERVO_SMALL_DELAY 10
#define SERVO_DEFAULT_ANGLE 180

#define SOIL_SENSOR_MAX 2000
#define SOIL_SENSOR_MIN 200

// prototypes
void goTo(int);
void goToRev(int, int);
void goToFor(int, int);

// RemoteXY configurate
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,7,0,59,0,196,0,13,178,0,
  130,2,28,15,19,38,5,4,0,34,
  14,7,36,6,26,65,114,90,1,9,
  9,67,1,32,49,11,4,6,26,11,
  1,0,5,12,16,16,5,31,226,172,
  134,0,1,0,5,37,16,16,5,31,
  226,172,135,0,1,0,79,12,16,16,
  5,31,226,172,134,0,1,0,79,37,
  16,16,5,31,226,172,135,0,2,1,
  57,54,18,8,6,5,16,16,83,69,
  78,83,79,82,32,65,82,77,0,83,
  84,79,87,0,130,2,56,15,19,38,
  5,4,0,62,14,7,36,6,26,67,
  1,60,49,11,4,6,26,11,129,0,
  32,54,12,4,6,83,112,101,101,100,
  0,129,0,44,12,14,3,6,87,101,
  116,32,77,101,116,101,114,0,66,132,
  44,1,13,10,6,24,67,0,59,2,
  14,5,6,5,11,67,1,0,0,12,
  6,38,5,11,67,1,0,5,12,6,
  38,5,11 };

// this structure defines all the variables and events of your control interface
struct {
    // input variables
  int8_t max_speed; // =0..100 slider position
  uint8_t forward_left; // =1 if button pressed, else =0
  uint8_t backward_left; // =1 if button pressed, else =0
  uint8_t forward_right; // =1 if button pressed, else =0
  uint8_t backward_right; // =1 if button pressed, else =0
  uint8_t arm_engage; // =1 if switch ON and =0 if OFF
  int8_t arm_slider; // =0..100 slider position

    // output variables
  uint8_t led_1_r; // =0..255 LED Red brightness
  uint8_t led_1_g; // =0..255 LED Green brightness
  uint8_t led_1_b; // =0..255 LED Blue brightness
  char max_speed_text[11];  // string UTF8 end zero
  char arm_pos_text[11];  // string UTF8 end zero
  int8_t wet_meter; // =0..100 level position
  char wet_text[11];  // string UTF8 end zero
  char temperature_F[11];  // string UTF8 end zero
  char temperature_C[11];  // string UTF8 end zero

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

// DC motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

// RC servo
Servo myServo;

// Soil & Temperature Sensor
Adafruit_seesaw ss;

// servo write() helpers to avoid too much current draw
// due to torque
void goTo(int dest) {
  int curr = myServo.read();
  if (dest > curr) {
    goToFor(dest, curr);
  } else goToRev(dest, curr);
}

void goToRev(int dest, int curr) {
  for (int i = curr; i > dest; i--) {
    myServo.write(i - 1);
    delay(SERVO_SMALL_DELAY);
  }
}

void goToFor(int dest, int curr) {
  for (int i = curr; i < dest; i++) {
    myServo.write(i + 1);
    delay(SERVO_SMALL_DELAY);
  }
}

void setLED(uint8_t red, uint8_t green, uint8_t blue) {
  RemoteXY.led_1_r = red;
  RemoteXY.led_1_g = green;
  RemoteXY.led_1_b = blue;
}

void setup() {
  // Debug serial
  // Serial.begin(115200);

  RemoteXY_Init();

  // initialize motor controller
  AFMS.begin();
  // initialize soil sensor controller
  ss.begin(0x36);

  pinMode(PIN_FORWARD_LEFT, OUTPUT);
  pinMode(PIN_BACKWARD_LEFT, OUTPUT);
  pinMode(PIN_FORWARD_RIGHT, OUTPUT);
  pinMode(PIN_BACKWARD_RIGHT, OUTPUT);

  // set initial arm pos to 171 degrees
  RemoteXY.arm_slider = SERVO_DEFAULT_ANGLE / 1.8;
  myServo.attach(9); // attaches the servo on pin 9 to the servo object
  myServo.write(SERVO_DEFAULT_ANGLE);
  delay(SERVO_DELAY);

  // set initial speed to 50%
  RemoteXY.max_speed = 50;
}

void loop() {
  RemoteXY_Handler();

  digitalWrite(PIN_FORWARD_LEFT, (RemoteXY.forward_left==0)?LOW:HIGH);
  digitalWrite(PIN_BACKWARD_LEFT, (RemoteXY.backward_left==0)?LOW:HIGH);
  digitalWrite(PIN_FORWARD_RIGHT, (RemoteXY.forward_right==0)?LOW:HIGH);
  digitalWrite(PIN_BACKWARD_RIGHT, (RemoteXY.backward_right==0)?LOW:HIGH);

  // temperature reading
  float tempC = ss.getTemp();
  // convert to fahrenheit
  float tempF = ((tempC / 5) * 9) + 32;
  // soil sensor reading
  uint16_t capread = ss.touchRead(0);

  // temp char arrays for float conversion
  char tF[10];
  char tC[10];

  // float conversion because arduino hates floats
  dtostrf(tempF, 5, 2, tF);
  dtostrf(tempC, 5, 2, tC);

  // push converted floats to UI
  sprintf(RemoteXY.temperature_F, "%s *F", tF);
  sprintf(RemoteXY.temperature_C, "%s *C", tC);

  // set wet meter
  RemoteXY.wet_meter = (int8_t)(capread / (SOIL_SENSOR_MAX / 100));
  // set wet text
  sprintf(RemoteXY.wet_text, "%u", capread);

  // set the speed based on slider
  uint8_t speed = floor(RemoteXY.max_speed * 2.55);
  // set speed text on UI
  sprintf(RemoteXY.max_speed_text, "%u", RemoteXY.max_speed);

  // set the arm position based on slider
  uint8_t pos = floor(RemoteXY.arm_slider * 1.80);

  // to avoid touching the ground with the sensor
  if (pos < 5) {
    pos = 5;
  }

  // set arm position text on UI
  sprintf(RemoteXY.arm_pos_text, "%u", pos);

  // only control arm when it is engaged
  if (RemoteXY.arm_engage == 1) {
    // when engaged, indicate with LED color change
    setLED(0, 132, 80); // spanish green
    // if a positional change happened, move the arm
    if (pos != myServo.read()) {
      goTo(pos);
    }
  } else {
    setLED(184, 29, 19); // carnelian red
    if (pos != SERVO_DEFAULT_ANGLE) {
      // default arm position
      RemoteXY.arm_slider = SERVO_DEFAULT_ANGLE / 1.8;
      goTo(SERVO_DEFAULT_ANGLE);
    }
  }

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
    // myMotor1->run(FORWARD); // change for faulty motor block
    myMotor1->run(BACKWARD);
  } else if (digitalRead(PIN_BACKWARD_RIGHT) == HIGH) {
    myMotor2->setSpeed(speed);
    myMotor2->run(BACKWARD);
    // myMotor1->setSpeed(speed);
    // myMotor1->run(BACKWARD); // change for faulty motor block
    myMotor1->run(RELEASE);
  } else {
    myMotor2->run(RELEASE);
    myMotor1->run(RELEASE);
  }
}
