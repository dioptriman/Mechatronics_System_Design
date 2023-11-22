#include "driver/ledc.h"

const int encoderPinA = 32;   // Interrupt pin for encoder channel A
const int encoderPinB = 33;   // Interrupt pin for encoder channel B
const int motorPin = 12;      // PWM pin for controlling the motor
const int motorPin2 = 14;

volatile long encoderCount = 0; // Current encoder count
long targetPosition = 600;      // Desired target position
float Kp = 15;                 // Proportional gain
float Ki = 1.25;                // Integral gain
float Kd = 0.75;                // Derivative gain

unsigned long prevTime = 0;    // Previous time for calculating dt
long prevError = 0;            // Previous error for calculating derivative
float integral = 0;            // Integral of the error

const int ledcChannel = 0;     // LEDC channel for PWM
const int ledcChannel2 = 1;
const int resolution = 8;      // PWM resolution, adjust as needed
const int frequency = 1000;    // PWM frequency in Hz
const int pwmMax = 75;        // Maximum PWM value

int enc_to_dist;

int state;

int pwmValue;


void IRAM_ATTR ISR() {
  // Reading the current state of encoder A and B
  int A = digitalRead(encoderPinA);
  int B = digitalRead(encoderPinB);
  // If the state of A changed, it means the encoder has been rotated
  if ((A == HIGH) != (B == LOW)) {
    encoderCount--;
  } else {
    encoderCount++;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  ledcSetup(ledcChannel, frequency, resolution);
  ledcSetup(ledcChannel2, frequency, resolution);
  ledcAttachPin(motorPin, ledcChannel);
  ledcAttachPin(motorPin2, ledcChannel2);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), ISR, RISING);

  enc_to_dist = 0;
  state = 0;
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long dt = currentTime - prevTime;

  enc_to_dist = ((encoderCount / 1000.0) * 45.9);

  // Calculate the error
  long error = targetPosition - enc_to_dist;

  // Calculate the integral term with anti-windup
  integral += error * dt;
  integral = constrain(integral, -75, 75);

  // Calculate the derivative term
  long dError = (error - prevError) / dt;

  // Calculate the control output using PID
  int output = Kp * error + Ki * integral - Kd * dError;

  // Use switch-case to handle different sequential movements
  switch(state) {
    case 0: // Move forward
      if (enc_to_dist < 300) {
        pwmValue = map(constrain(output, -75, 75), -75, 75, 0, pwmMax);
        ledcWrite(ledcChannel, pwmValue);
      } else {
        delay(1000); // Delay before stopping
        state = 1; // Move to the next state for stopping
        targetPosition = 0; // Set target for returning to zero
      }
      break;
    case 1: // Stop
      ledcWrite(ledcChannel, 0);
      delay(1000); // Delay before changing direction
      state = 2; // Move to the next state for returning to zero
      break;
    case 2: // Return to zero
      if (enc_to_dist > 0) {
        pwmValue = map(constrain(-output, -75, 75), -75, 75, 0, pwmMax);
        ledcWrite(ledcChannel2, pwmValue);
      } else {
        delay(1000); // Delay before stopping
        state = 3; // Move to the next state for stopping after reaching zero
      }
      break;
    case 3: // Stop after reaching zero
      ledcWrite(ledcChannel2, 0);
      delay(1000); // Delay before changing direction
      state = 0; // Restart the sequence by moving forward
      targetPosition = 300; // Set target for the forward movement
      break;
    default:
      break;
  }

  // Update previous values
  prevError = error;
  prevTime = currentTime;
  delay(10);
}




