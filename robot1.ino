/*
  Rover Robot
 */
#include <string.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <PID_v1.h>
#include <LiquidCrystal.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include <SPI.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BluefruitLE_UART.h>
#include "BluefruitConfig.h"

uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);

extern uint8_t packetbuffer[];

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
  Adafruit_MotorShield AFMS;
  Adafruit_DCMotor *myMotor;

  public:
  Motor(uint8_t m, int interval)
    : AFMS()
  {
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
//      myMotor->setSpeed(speed);
//      if (newdir == 1){
//        myMotor->run(FORWARD);
//      } else if (newdir == -1) {
//        myMotor->run(BACKWARD);
//      } else {
        myMotor->run(RELEASE);
//      }
    }
  }

  void begin(void){
    AFMS.begin();
  }

};

class uSound {
  long duration, distance, interval;
  unsigned long prevMillis;
  int dist_arr[NUM_USAMPLES];

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

  public:
  uSound(int interval=10){
    duration, distance = 0;
    for (int i=0; i<NUM_USAMPLES-1; i++){
      dist_arr[i]=0;
    }
  }

  int get_dist(unsigned long currMillis){
    int dist_avg;
    if (currMillis - prevMillis >= interval){
      prevMillis = currMillis;
      distance=usound();

      for (int i=0; i<NUM_USAMPLES-1; i++){
        dist_arr[i+1] = dist_arr[i];
      }
      dist_arr[0] = distance;
      dist_avg = 0;
      for (int i=0; i<NUM_USAMPLES; i++){
        dist_avg += dist_arr[i];
      }
      dist_avg /= NUM_USAMPLES;
      distance = dist_avg;
    }
    return distance;
  }
};

class Bluetooth
{
  int initialized, connected;
  unsigned long prevMillis;
  long interval;

  public:

  Adafruit_BluefruitLE_UART ble;

  Bluetooth()
    : ble (Serial2, BLUEFRUIT_UART_MODE_PIN)
  {
    interval = 200;
    connected = 0;
  };

  int begin(void)
  {
    ble.echo(false);
    initialized = ble.begin(VERBOSE_MODE);
    if (initialized)
    {
      ble.setMode(BLUEFRUIT_MODE_DATA);
      ble.verbose(false);
      ble.info();
    }
    return initialized;
  }
  
  int isInitialized(void)
  {
    return initialized;
  }

  int isConnected(void)
  {
    unsigned long currMillis = millis();
    if (currMillis - prevMillis >= interval)
    {
      prevMillis = currMillis;
      ble.setMode(BLUEFRUIT_MODE_COMMAND);
      connected = ble.isConnected();
      ble.setMode(BLUEFRUIT_MODE_DATA);
    }
    return connected;
  }

  void disconnect(void)
  {
    ble.end();
  }
};

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
    //Serial.print(ir[i]);
    //Serial.print("  ");
  }
  if (ir_en_cnt>0) {
    res = res/ir_en_cnt;
  } else {
    res = 0;
  }
  //Serial.print(" Res:");
  //Serial.print(res);
  //Serial.print("  ");
  return res;
}

//------------------------------------------------------------------------------
// Public variables

Motor brMotor(Motor2, 10);
Motor blMotor(Motor1, 10);
Motor frMotor(Motor3, 10);
Motor flMotor(Motor4, 10);

// Distance PID variables
double dSetpoint, dInput, dOutput;
double dKp=3, dKi=5, dKd=2;
PID distPID(&dInput, &dOutput, &dSetpoint, dKp, dKi, dKd, REVERSE);

uSound usound(10);

// Follower PID variables
double fSetpoint, fInput, fOutput;
double fKp=3, fKi=5, fKd=2;
PID followPID(&fInput, &fOutput, &fSetpoint, fKp, fKi, fKd, DIRECT);

// LCD
LiquidCrystal lcd(8, 9, 10, 11, 12, 13);

Bluetooth blue;

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
  Serial.begin(115200);

  // initialize digital pin 13 as an output.
  pinMode(LED, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize motor shield
  brMotor.begin();
  blMotor.begin();
  frMotor.begin();
  flMotor.begin();

  // Set IR sensor pins
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);

  // Set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  pinMode(LCD_RED, OUTPUT);
  pinMode(LCD_GREEN, OUTPUT);
  pinMode(LCD_BLUE, OUTPUT);
  LCD_brightness = 100;
  LCD_setBacklight(162, 40, 255); // light purple

  // Set up PID for distance
  dSetpoint = 30;
  distPID.SetMode(AUTOMATIC);
  distPID.SetOutputLimits(-80,80);
  distPID.SetSampleTime(200);

  // Set up PID for light following
  fSetpoint = 0;
  followPID.SetMode(AUTOMATIC);
  followPID.SetOutputLimits(-50,50);
  followPID.SetSampleTime(200);

  // Enable Bluetooth module
  if (blue.begin()){
    Serial.println( "Initialized Bluefruit" );
  }

}

void loop() {
  int distance, lspeed, rspeed;
  int ir_sensors;
  unsigned long currMillis = millis();
  uint8_t len;

  // Get average measured distance
  dInput=usound.get_dist(currMillis);
  distPID.Compute();

  // Get IR sensors
  ir_sensors=ir();
  fInput=ir_sensors;
  followPID.Compute();

  // Merge IR & distance outputs and transform into wheel speed
  lspeed = dOutput * (fOutput> 40? 0.2: .5) - fOutput;
  rspeed = dOutput * (fOutput<-40? 0.2: .5) + fOutput;

  lcd.setCursor(0,0);
  lcd.print("dOut    ");
  lcd.setCursor(5,0);
  lcd.print((int)dOutput);
  lcd.setCursor(0,1);
  lcd.print("fOut    ");
  lcd.setCursor(5,1);
  lcd.print((int)fOutput);

  brMotor.update(rspeed, currMillis);
  frMotor.update(rspeed, currMillis);
  blMotor.update(lspeed, currMillis);
  flMotor.update(lspeed, currMillis);

  if (distance <15) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }

  if (blue.isConnected())
  {
    lcd.setCursor(9,1);
    lcd.print("BLE On ");
    len = readPacket(&blue.ble, BLE_READPACKET_TIMEOUT);
    if (len>0) {

      // Color
      if (packetbuffer[1] == 'C') {
        uint8_t red = packetbuffer[2];
        uint8_t green = packetbuffer[3];
        uint8_t blue = packetbuffer[4];
        Serial.print ("RGB #");
        if (red < 0x10) Serial.print("0");
        Serial.print(red, HEX);
        if (green < 0x10) Serial.print("0");
        Serial.print(green, HEX);
        if (blue < 0x10) Serial.print("0");
        Serial.println(blue, HEX);
        LCD_setBacklight(red, green, blue);
      }

      // Buttons
      if (packetbuffer[1] == 'B') {
        uint8_t buttnum = packetbuffer[2] - '0';
        boolean pressed = packetbuffer[3] - '0';
        Serial.print ("Button "); Serial.print(buttnum);
        if (pressed) {
          Serial.println(" pressed");
        } else {
          Serial.println(" released");
        }
      }

    }
  } else if (blue.isInitialized()) {
    lcd.setCursor(9,1);
    lcd.print("BLE Off");
  } else { // Bluefruit not detected
    lcd.setCursor(9,1);
    lcd.print("BLE N/A");
  }
}


/* vim: set tabstop=2 shiftwidth=2 expandtab: */

