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
  unsigned long prevMillis;

  public:
  Motor(int pin){
    motorPin = pin;
    pinMode(motorPin, OUTPUT);
    speed = 0;
    prevMillis = 0;
  }

  void Update(int newspeed, unsigned long currMillis){
    if (currMillis - prevMillis >= 10){
            prevMillis = currMillis;
            speed = newspeed;
            analogWrite(motorPin, speed);
    }
  }
};

Motor lMotor(leftMotor);
Motor rMotor(rightMotor);

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
  distance = (duration/2) / 29.1;

  if (distance <10) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
  Serial.print(distance);
  Serial.println(" cm");
  return(distance);
}

void ir() {
  int ir [6];
  for (int i=0; i<6; i++){
    ir[i] = digitalRead(IR1+i);
    Serial.print(ir[i]);
    Serial.print("  ");
  }
}

void loop() {
  int distance;
  unsigned long currMillis = millis();
  ir();
  distance=usound();
  rMotor.Update(distance, currMillis);
  delay(50);
}
