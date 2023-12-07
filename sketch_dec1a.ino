#include <ESP32Servo.h>
#include "driver/ledc.h"

const int encoderPinA = 16;   // Interrupt pin for encoder channel A
const int encoderPinB = 17;

const int encoderPinAHort = 18;  
const int encoderPinBHort = 19;

volatile long encoderCount = 0;
volatile long encoderCountHort = 0;

const int motorPin1 = 33;
const int motorPin2 = 32;

const int motorPin3 = 26;
const int motorPin4 = 27;

const int resolution = 8;      // PWM resolution, adjust as needed
const int frequency = 1000; 

bool motorstopvert_1 = false;

bool motorstophort_1 = false;

Servo myservo;

int currentState = 0;

bool closingServo = false;

const int encoderMainPinA = 34;   // Interrupt pin for encoder channel A
const int encoderMainPinB = 35;   // Interrupt pin for encoder channel B
const int motorMainPin = 23;      // PWM pin for controlling the motor
const int motorMainPin2 = 13;

volatile long encoderMainCount = 0; // Current encoder count
long targetPosition = -600;      // Desired target position
float Kp = 15;                 // Proportional gain
float Ki = 1.25;                // Integral gain
float Kd = 0.75;                // Derivative gain

unsigned long prevTime = 0;    // Previous time for calculating dt
long prevError = 0;            // Previous error for calculating derivative
float integral = 0;            // Integral of the error

const int ledcChannel = 1;     // LEDC channel for PWM
const int ledcChannel2 = 2;
// const int resolution = 8;      // PWM resolution, adjust as needed
// const int frequency = 1000;    // PWM frequency in Hz
const int pwmMax = 150;        // Maximum PWM value

int enc_to_dist;

int state;

int pwmValue;

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

void IRAM_ATTR ISRMain() {
  // Reading the current state of encoder A and B
  int A_3 = digitalRead(encoderMainPinA);
  int B_3 = digitalRead(encoderMainPinB);
  // If the state of A changed, it means the encoder has been rotated
  if ((A_3 == HIGH) != (B_3 == LOW)) {
    encoderMainCount--;
  } else {
    encoderMainCount++;
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

  pinMode(encoderMainPinA, INPUT_PULLUP);
  pinMode(encoderMainPinB, INPUT_PULLUP);

  ledcSetup(ledcChannel, 1000, 8);
  ledcSetup(ledcChannel2, 1000, 8);
  ledcAttachPin(motorMainPin, ledcChannel);
  ledcAttachPin(motorMainPin2, ledcChannel2);

  attachInterrupt(digitalPinToInterrupt(encoderMainPinA), ISRMain, RISING);

  enc_to_dist = 0;


  myservo.attach(22);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinAHort), ISR2, RISING);
}

void servoClose(){
  myservo.write(0);
}

void servoOpen(){
  myservo.write(180);
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
    unsigned long currentTime = millis();
    unsigned long dt = currentTime - prevTime;

    enc_to_dist = ((encoderMainCount / 1000.0) * 45.9);

    // Calculate the error
    long error = targetPosition - enc_to_dist;

    // Calculate the integral term with anti-windup
    integral += error * dt;
    integral = constrain(integral, -150, 150);

    // Calculate the derivative term
    long dError = (error - prevError) / dt;

    // Calculate the control output using PID
    int output = Kp * error + Ki * integral - Kd * dError;
    switch (currentState) {
      case 0:
        servoClose();
        delay(1000);
        currentState++;
        break;

      
      case 1:
        if (encoderCount > 1000) {
          motorStop(); // Stop the motor
          currentState++;
        } else {
          digitalWrite(motorPin1, LOW);
          digitalWrite(motorPin2, HIGH);
        }

        break;
      
      case 2:
        if(encoderCountHort < -1600){
          motorHortStop();
          currentState++;
        } else {
          digitalWrite(motorPin3, HIGH);
          digitalWrite(motorPin4, LOW);
        }

        break;

      case 3:
        if(encoderCount < 0){
          motorStop();
          currentState++;
        } else {
          digitalWrite(motorPin1, HIGH);
          digitalWrite(motorPin2, LOW);
        }

        break;

      case 4:
        // closingServo = false;
        servoOpen();
        delay(500);
        currentState++;
        break;

      case 5:
        if (enc_to_dist > -600) {
          pwmValue = map(constrain(-output, -150, 150), -150, 150, 0, pwmMax);
          ledcWrite(ledcChannel, pwmValue);
        } else {
          delay(500); // Delay before stopping
          currentState++; // Move to the next state for stopping
          targetPosition = 0; // Set target for returning to zero
        }
        break;
      
      case 6: // Stop
        ledcWrite(ledcChannel, 0);
        delay(500); // Delay before changing direction
        currentState++; // Move to the next state for returning to zero
        break;
      
      case 7: 
        // closingServo = true;
        servoClose();
        delay(1500);
        currentState++;
        break;

      case 8:

        if (encoderCount > 1000) {
          motorStop(); // Stop the motor
          currentState++;
        } else {
          digitalWrite(motorPin1, LOW);
          digitalWrite(motorPin2, HIGH);
        }
        break;
      
      case 9:
        if(encoderCountHort < -3050){
          motorHortStop();
          currentState++;
        } else {
          digitalWrite(motorPin3, HIGH);
          digitalWrite(motorPin4, LOW);
        }
        break;

      case 10:

        if(encoderCount < 0){
              motorStop();
              currentState++;
        } else {
              digitalWrite(motorPin1, HIGH);
              digitalWrite(motorPin2, LOW);
        }

        break;

      case 11:
      //  closingServo = false;
        servoOpen();
        delay(500);
        currentState++;
        break;


      case 12:
          if (enc_to_dist < 0) {
          pwmValue = map(constrain(output, -150, 150), -150, 150, 0, pwmMax);
          ledcWrite(ledcChannel2, pwmValue);
        } else {
          delay(500); // Delay before stopping
          currentState++; // Move to the next state for stopping after reaching zero
        }
      
      case 13:
        ledcWrite(ledcChannel2, 0);
        delay(500); // Delay before changing direction
        currentState ++; // Restart the sequence by moving forward
        targetPosition = -600; // Set target for the forward movement
        break;
      // Other cases...

      default:
        break;
  }

  // if(closingServo){
  //   servoClose();
  // }else{
  //   servoOpen();
  // }


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

