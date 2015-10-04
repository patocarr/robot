/*
  Rover Robot
 */
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <PID_v1.h>
#include <LiquidCrystal.h>
#include <Wire.h>

#define trigPin 31
#define echoPin 30

#define IR1 22
#define IR2 23
#define IR3 24
#define IR4 25
#define IR5 26
#define IR6 27

#define LED 13

#define LCD_RED 3
#define LCD_GREEN 5
#define LCD_BLUE 6

#define Motor1 1
#define Motor2 2
#define Motor3 3
#define Motor4 4

#define NUM_USAMPLES 10

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

  void update(int newspeed, unsigned long currMillis){
    int newdir;
    if (currMillis - prevMillis >= interval){
      prevMillis = currMillis;
      if (newspeed>=0){
        newdir = 1;
      } else {
        newdir = -1;
      }
      speed = abs(newspeed);
      myMotor->setSpeed(speed);
      if (newdir == 1){
        myMotor->run(FORWARD);
      } else if (newdir == -1) {
        myMotor->run(BACKWARD);
      } else {
        myMotor->run(RELEASE);
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
  if (ir_en_cnt>0) {
    res = res/ir_en_cnt;
  } else {
    res = 0;
  }
  Serial.print(" Res:");
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
int direction=1, prevdirection;
unsigned long usMillis;

// Distance PID variables
double dSetpoint, dInput, dOutput;
double dKp=3, dKi=5, dKd=2;
PID distPID(&dInput, &dOutput, &dSetpoint, dKp, dKi, dKd, REVERSE);

int dist_arr[NUM_USAMPLES];

// Follower PID variables
double fSetpoint, fInput, fOutput;
double fKp=3, fKi=5, fKd=2;
PID followPID(&fInput, &fOutput, &fSetpoint, fKp, fKi, fKd, DIRECT);

// LCD
LiquidCrystal lcd(8, 9, 10, 11, 12, 13);

int LCD_brightness = 255;

void LCD_setBacklight(uint8_t r, uint8_t g, uint8_t b) {
  // normalize the red LED - its brighter than the rest!
  r = map(r, 0, 255, 0, 100);
  g = map(g, 0, 255, 0, 150);
 
  r = map(r, 0, 255, 0, LCD_brightness);
  g = map(g, 0, 255, 0, LCD_brightness);
  b = map(b, 0, 255, 0, LCD_brightness);
 
  // common anode so invert!
  r = map(r, 0, 255, 255, 0);
  g = map(g, 0, 255, 255, 0);
  b = map(b, 0, 255, 255, 0);
  analogWrite(LCD_RED, r);
  analogWrite(LCD_GREEN, g);
  analogWrite(LCD_BLUE, b);
}

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

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  pinMode(LCD_RED, OUTPUT);
  pinMode(LCD_GREEN, OUTPUT);
  pinMode(LCD_BLUE, OUTPUT);
  LCD_brightness = 100;
  LCD_setBacklight(162, 40, 255); // light purple

  dSetpoint = 50;
  distPID.SetMode(AUTOMATIC);
  distPID.SetOutputLimits(-80,80);
  distPID.SetSampleTime(200);

  fSetpoint = 0;
  followPID.SetMode(AUTOMATIC);
  followPID.SetOutputLimits(-50,50);
  followPID.SetSampleTime(200);
}

void loop() {
  int distance, lspeed, rspeed;
  int ir_sensors;
  int dist_avg, dist_acc;
  unsigned long currMillis = millis();

  ir_sensors=ir();

  if (currMillis - usMillis > 10){
    distance=usound();
    usMillis = currMillis;

    for (int i=0; i<NUM_USAMPLES-1; i++){
      dist_arr[i+1] = dist_arr[i];
    }
    dist_arr[0] = distance;
    dist_avg = 0;
    for (int i=0; i<NUM_USAMPLES; i++){
      dist_avg += dist_arr[i];
    }
    dist_avg /= NUM_USAMPLES;
  }

  dInput=dist_avg;
  distPID.Compute();

  fInput=ir_sensors;
  followPID.Compute();

  lspeed = dOutput * (fOutput> 40? 0.2: .5) - fOutput;
  rspeed = dOutput * (fOutput<-40? 0.2: .5) + fOutput;

  Serial.print(" dOutput=");
  Serial.print(dOutput);
  Serial.print(" fOutput=");
  Serial.print(fOutput);

  lcd.setCursor(0,0);
  lcd.print("dOutput=");
  lcd.print(dOutput);
  lcd.setCursor(0,1);
  lcd.print("fOutput=");
  lcd.print(fOutput);


//  switch (state) {
//    case IDLE:
//      rspeed = lspeed = 0;
//      state = MOVING;
//      break;
//    case MOVING:
//      if (distance < 10) {
//        pauseMillis = currMillis;
//        prevdirection = direction;
//        state = PAUSE;
//      }
//      break;
//    case PAUSE:
//      rspeed = lspeed = 0;
//      direction = 0;
//      if (currMillis - pauseMillis > 1000){
//        state = MOVING;
//        direction = -prevdirection;
//      }
//      break;
//    case STOP:
//      rspeed = lspeed = 0;
//      break;
//  }

//  Serial.print(" State=");
//  Serial.print((int)state);
  Serial.print(" Speed L:R=");
  Serial.print(lspeed);
  Serial.print(":");
  Serial.print(rspeed);

  brMotor.update(rspeed, currMillis);
  frMotor.update(rspeed, currMillis);
  blMotor.update(lspeed, currMillis);
  flMotor.update(lspeed, currMillis);

  if (distance <15) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
  Serial.print(" ");
  Serial.print(dist_avg);
  Serial.println(" cm");

  //delay(50);
}

/* vim: set tabstop=2 shiftwidth=2 expandtab: */

