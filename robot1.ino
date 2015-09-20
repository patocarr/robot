/*
  Rover Robot
 */

#define trigPin 31
#define echoPin 30

#define IR1 22
#define IR2 23
#define IR3 24
#define IR4 25
#define IR5 26
#define IR6 27

#define LED 13

#define rightMotor 3
#define leftMotor 4

class Motor {
  int speed;
  int motorPin;
  int interval;
  unsigned long prevMillis;

  public:
  Motor(int pin, int interval){
    motorPin = pin;
    pinMode(motorPin, OUTPUT);
    speed = 0;
    prevMillis = 0;
  }

  void update(int newspeed, unsigned long currMillis){
    if (currMillis - prevMillis >= interval){
            prevMillis = currMillis;
            speed = newspeed;
            analogWrite(motorPin, speed);
    }
  }
};

Motor lMotor(leftMotor, 10);
Motor rMotor(rightMotor, 10);

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  
  // initialize digital pin 13 as an output.
  pinMode(LED, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);
}

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
  int res=0;
  for (int i=0; i<6; i++){
    ir[i] = digitalRead(IR1+i);
    res = res | ir[i]<<i;
    Serial.print(ir[i]);
    Serial.print("  ");
  }
  Serial.print(res);
  Serial.print("  ");
  return res;
}

void loop() {
  int distance;
  int ir_sensors;
  unsigned long currMillis = millis();
  ir_sensors=ir();
  distance=usound();
  rMotor.update(distance, currMillis);
  lMotor.update(distance, currMillis);

  if (distance <10) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
  Serial.print(distance);
  Serial.println(" cm");

  delay(50);
}

