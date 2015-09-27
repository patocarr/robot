/*
  Rover Robot
 */
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

#define trigPin 31
#define echoPin 30

#define IR1 22
#define IR2 23
#define IR3 24
#define IR4 25
#define IR5 26
#define IR6 27

#define LED 13

#define Motor1 1
#define Motor2 2
#define Motor3 3
#define Motor4 4

class Motor {
  int speed;
  int motorPin;
  int interval;
  unsigned long prevMillis;

  // Create the motor shield object with the default I2C address
  Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
  Adafruit_DCMotor *myMotor;

  public:
  Motor(uint8_t m, int interval){
    // Select which motor
    myMotor = AFMS.getMotor(m);
    speed = 0;
    prevMillis = 0;
  }

  void update(int newspeed, int dir, unsigned long currMillis){
    if (currMillis - prevMillis >= interval){
      prevMillis = currMillis;
      speed = newspeed;
      myMotor->setSpeed(speed);
      if (dir == 1){
        myMotor->run(FORWARD);
      } else {
        myMotor->run(BACKWARD);
      }
    }
  }

  void begin(void){
    AFMS.begin();
  }

};

int usound() {

  long duration, distance;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(6);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration/58;

  return(distance);
}

int ir() {
  int ir [6];
  int cosx [6]={-100, -60, -20, 20, 60, 100};
  int res=0;
  int ir_en_cnt=0;
  for (int i=0; i<6; i++){
    ir[i] = digitalRead(IR1+i);
    //res |= ir[i]<<i;
    res += cosx[i]*(ir[i] ? 0 : 1);
    if (!ir[i]) ir_en_cnt++;
    Serial.print(ir[i]);
    Serial.print("  ");
  }
  if (ir_en_cnt>0) res = res/ir_en_cnt;
  Serial.print(res);
  Serial.print("  ");
  return res;
}

//------------------------------------------------------------------------------
// Public variables

Motor brMotor(Motor2, 10);
Motor blMotor(Motor1, 10);
Motor frMotor(Motor3, 10);
Motor flMotor(Motor4, 10);

enum state_enum {IDLE, MOVING, PAUSE, STOP} state = IDLE;
unsigned long pauseMillis;
int direction=1;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  
  // initialize digital pin 13 as an output.
  pinMode(LED, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize motor shield
  brMotor.begin();
  blMotor.begin();
  frMotor.begin();
  flMotor.begin();

  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);
}

void loop() {
  int distance, lspeed, rspeed;
  int ir_sensors;
  unsigned long currMillis = millis();

  ir_sensors=ir();
  distance=usound();
  lspeed = ir_sensors<0 ? distance*(100+ir_sensors)/100: distance;
  rspeed = ir_sensors>0 ? distance*(100-ir_sensors)/100: distance;

  switch (state) {
    case IDLE:
      rspeed = lspeed = 0;
      state = MOVING;
      break;
    case MOVING:
      if (distance < 10) {
        pauseMillis = currMillis;
        state = PAUSE;
      }
      break;
    case PAUSE:
      rspeed = lspeed = 0;
      if (currMillis - pauseMillis > 1000){
        state = MOVING;
        direction = -direction;
      }
      break;
    case STOP:
      rspeed = lspeed = 0;
      break;
  }

  Serial.print(" State=");
  Serial.print((int)state);
  Serial.print(" Speed L:R=");
  Serial.print(direction*lspeed);
  Serial.print(":");
  Serial.print(rspeed);

  brMotor.update(rspeed, direction, currMillis);
  frMotor.update(rspeed, direction, currMillis);
  blMotor.update(lspeed, direction, currMillis);
  flMotor.update(lspeed, direction, currMillis);

  if (distance <15) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
  Serial.print(" ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(50);
}

/* vim: set tabstop=2 shiftwidth=2 expandtab: */

