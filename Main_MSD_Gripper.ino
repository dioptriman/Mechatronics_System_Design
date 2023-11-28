#include <ESP32Servo.h>

const int encoderPinA = 34;   // Interrupt pin for encoder channel A
const int encoderPinB = 35;

const int encoderPinAHort = 18;  
const int encoderPinBHort = 19;

volatile long encoderCount = 0;
volatile long encoderCountHort = 0;

const int motorPin1 = 17;
const int motorPin2 = 16;

const int motorPin3 = 22;
const int motorPin4 = 23;

bool motorstopvert_1 = false;

bool motorstophort_1 = false;

Servo myservo;

int currentState = 0;

bool closingServo = false;

void IRAM_ATTR ISR(){
  int A = digitalRead(encoderPinA);
  int B = digitalRead(encoderPinB);
  if((A == HIGH) != (B==LOW)){
    encoderCount--;
  }
  else {
    encoderCount++;
  }
}

void IRAM_ATTR ISR2(){
  int A_2 = digitalRead(encoderPinAHort);
  int B_2 = digitalRead(encoderPinAHort);
  if((A_2 == HIGH) != (B_2==LOW)){
    encoderCountHort--;
  }
  else {
    encoderCountHort++;
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  
  pinMode(encoderPinAHort, INPUT_PULLUP);
  pinMode(encoderPinBHort, INPUT_PULLUP);
  
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

  myservo.attach(15);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinAHort), ISR2, RISING);
}

void servoClose(){
  myservo.write(0);
}

void servoOpen(){
  myservo.write(110);
}

void updateEncoderCount() {
  int A = digitalRead(encoderPinA);
  int B = digitalRead(encoderPinB);
  if ((A == HIGH) != (B == LOW)) {
    encoderCount--;
  } else {
    encoderCount++;
  }
}

void motorStop() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}

void motorHortStop() {
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
}

void loop() {
  switch (currentState) {
    case 0:
      servoClose();
      delay(500);
      currentState++;
      break;

    case 1:
      if (encoderCount < -2500) {
        motorStop(); // Stop the motor
        currentState++;
      } else {
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
      }

      break;
    
    case 2:
      if(encoderCountHort < -1550){
        motorHortStop();
        currentState++;
      } else {
        digitalWrite(motorPin3, HIGH);
        digitalWrite(motorPin4, LOW);
      }

      break;

    case 3:
      if(encoderCount > 0){
        motorStop();
        currentState++;
      } else {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
      }

      break;

    case 4:
      servoOpen();
      delay(1500);
      currentState++;
      break;
    
    case 5: 
      servoClose();
      delay(1500);
      currentState++;
      break;

    case 6:
      if (encoderCount < -2500) {
        motorStop(); // Stop the motor
        currentState++;
      } else {
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
      }
      break;
    
    case 7:
       if(encoderCountHort < -3050){
        motorHortStop();
        currentState++;
      } else {
        digitalWrite(motorPin3, HIGH);
        digitalWrite(motorPin4, LOW);
      }
      break;

    case 8:
      if(encoderCount > 0){
            motorStop();
            currentState++;
      } else {
            digitalWrite(motorPin1, LOW);
            digitalWrite(motorPin2, HIGH);
      }

      break;

    case 9:
     servoOpen();
     delay(1500);
     break;

    // Other cases...

    default:
      break;
  }





  Serial.println(encoderCountHort);
}


void motorUp(){
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
}

void motorCenterLeft(){
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
}


void motorStopHort(){
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
}



void servoLock(){
  myservo.write(0);
}

void motorGoUp(){
}

void motorGoCenter(){

}

