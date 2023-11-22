const int encoderPinA = 18;   // Interrupt pin for encoder channel A
const int encoderPinB = 19;

volatile long encoderCount = 0;

const int motorPin1 = 22;
const int motorPin2 = 23;

bool motorstop = false;


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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  // myservo.attach(22);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), ISR, RISING);
}

void loop() {
  // // put your main code here, to run repeatedly:
  if(!motorstop && encoderCount < 1630){
    // servoClose();
    motorUp();
    if(encoderCount >=  1630){
      motorStop();
    }
  }
  else if(motorStop && encoderCount >= 1630){
    motorStop();
  }


  Serial.println(encoderCount);
}

void motorUp(){
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
}

void motorStop(){
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}

// void servoClose(){
//   myservo.write(0);
//   delay(1500);
//   myservo.write(90);
//   delay(1500);
// }
