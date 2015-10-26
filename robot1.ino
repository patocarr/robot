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

#define PACKET_ACC_LEN                  (15)
#define PACKET_GYRO_LEN                 (15)
#define PACKET_MAG_LEN                  (15)
#define PACKET_QUAT_LEN                 (19)
#define PACKET_BUTTON_LEN               (5)
#define PACKET_COLOR_LEN                (6)
#define PACKET_LOCATION_LEN             (15)

// Size of the read buffer for incoming packets
#define READ_BUFSIZE                    (20)

/* Buffer to hold incoming characters */
uint8_t packetbuffer[READ_BUFSIZE+1];

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
  int trigPin;
  int echoPin;

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
  uSound(int interval=10)
    : trigPin(31), echoPin(30)
  {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
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
  Adafruit_BluefruitLE_UART ble;

  public:

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

  /**************************************************************************/
  /*!
    @brief  Waits for incoming data and parses it
   */
  /**************************************************************************/
  uint8_t readPacket(uint16_t timeout) 
  {
    uint16_t origtimeout = timeout, replyidx = 0;

    memset(packetbuffer, 0, READ_BUFSIZE);

    while (timeout--) {
      if (replyidx >= 20) break;
      if ((packetbuffer[1] == 'A') && (replyidx == PACKET_ACC_LEN))
        break;
      if ((packetbuffer[1] == 'G') && (replyidx == PACKET_GYRO_LEN))
        break;
      if ((packetbuffer[1] == 'M') && (replyidx == PACKET_MAG_LEN))
        break;
      if ((packetbuffer[1] == 'Q') && (replyidx == PACKET_QUAT_LEN))
        break;
      if ((packetbuffer[1] == 'B') && (replyidx == PACKET_BUTTON_LEN))
        break;
      if ((packetbuffer[1] == 'C') && (replyidx == PACKET_COLOR_LEN))
        break;
      if ((packetbuffer[1] == 'L') && (replyidx == PACKET_LOCATION_LEN))
        break;

      while (ble.available()) {
        char c =  ble.read();
        if (c == '!') {
          replyidx = 0;
        }
        packetbuffer[replyidx] = c;
        replyidx++;
        timeout = origtimeout;
      }

      if (timeout == 0) break;
      delay(1);
    }

    packetbuffer[replyidx] = 0;  // null term

    if (!replyidx)  // no data or timeout 
      return 0;
    if (packetbuffer[0] != '!')  // doesn't start with '!' packet beginning
      return 0;

    // check checksum!
    uint8_t xsum = 0;
    uint8_t checksum = packetbuffer[replyidx-1];

    for (uint8_t i=0; i<replyidx-1; i++) {
      xsum += packetbuffer[i];
    }
    xsum = ~xsum;

    // Throw an error message if the checksum's don't match
    if (xsum != checksum)
    {
      //Serial.print("Checksum mismatch in packet : ");
      //printHex(packetbuffer, replyidx+1);
      return 0;
    }

    // checksum passed!
    return replyidx;
  }

  void disconnect(void)
  {
    ble.end();
  }
};

class Display
{
  int LCD_brightness;
  LiquidCrystal lcd;
  uint8_t LCD_RED;
  uint8_t LCD_GREEN;
  uint8_t LCD_BLUE;

  public:
  Display ()
  : lcd(8, 9, 10, 11, 12, 13), LCD_RED(3), LCD_GREEN(5), LCD_BLUE(6)
  {
    LCD_brightness = 100;
    pinMode(LCD_RED, OUTPUT);
    pinMode(LCD_GREEN, OUTPUT);
    pinMode(LCD_BLUE, OUTPUT);
    set_bklight(162, 40, 255); // light purple
    // Set up the LCD's number of columns and rows:
    lcd.begin(16, 2);
  }

  void dout(int dOutput)
  {
    lcd.setCursor(0,0);
    lcd.print("dOut    ");
    lcd.setCursor(5,0);
    lcd.print((int)dOutput);
  }

  void fout(int fOutput)
  {
    lcd.setCursor(0,1);
    lcd.print("fOut    ");
    lcd.setCursor(5,1);
    lcd.print((int)fOutput);
  }
  
  void ble_on(void)
  {
    lcd.setCursor(9,1);
    lcd.print("BLE On ");
  }

  void ble_off(void)
  {
    lcd.setCursor(9,1);
    lcd.print("BLE Off");
  }

  void ble_na(void)
  {
    lcd.setCursor(9,1);
    lcd.print("BLE N/A");
  }

  void set_bklight(uint8_t r, uint8_t g, uint8_t b) {
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

// Follower PID variables
double fSetpoint, fInput, fOutput;
double fKp=3, fKi=5, fKd=2;
PID followPID(&fInput, &fOutput, &fSetpoint, fKp, fKi, fKd, DIRECT);

uSound usound(10);

Bluetooth blue;

Display lcd;


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);

  // initialize digital pin 13 as an output.
  pinMode(LED, OUTPUT);

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

  lcd.dout(dOutput);
  lcd.fout(fOutput);

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
    lcd.ble_on();
    len = blue.readPacket(BLE_READPACKET_TIMEOUT);
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
        lcd.set_bklight(red, green, blue);
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
    lcd.ble_off();
  } else { // Bluefruit not detected
    lcd.ble_na();
  }
}


/* vim: set tabstop=2 shiftwidth=2 expandtab: */



