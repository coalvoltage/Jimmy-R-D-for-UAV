
/*
 * Encoder example sketch
 * by Andrew Kramer
 * 1/1/2016
 *
 * Records encoder ticks for each wheel
 * and prints the number of ticks for
 * each encoder every 500ms
 *
 */
 
// pins for the encoder inputs
#define RH_ENCODER_A 3 
#define RH_ENCODER_B 5
#define LH_ENCODER_A 2
#define LH_ENCODER_B 4

//Motor B
const int motorPin1  = 10; // Pin  7 of L293
const int motorPin2  = 9;  // Pin  2 of L293

const int motorSpeed = 156-60;

// variables to store the number of encoder pulses
// for each motor
volatile signed long leftCount = 0;
volatile signed long rightCount = 0;
 
void setup() {
  //pinMode(LH_ENCODER_A, INPUT);
  //pinMode(LH_ENCODER_B, INPUT);
  //pinMode(RH_ENCODER_A, INPUT);
  //pinMode(RH_ENCODER_B, INPUT);

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  
  // initialize hardware interrupts
  //attachInterrupt(0, leftEncoderEvent, CHANGE);
  //attachInterrupt(1, rightEncoderEvent, CHANGE);
  
  Serial.begin(9600);
}
 
void loop() {
  analogWrite(motorPin1, motorSpeed);
  analogWrite(motorPin2, LOW);
  //Serial.print("Right Count: ");
  //Serial.println(rightCount);
  //Serial.print("Left Count: ");
  //Serial.println(leftCount);
  //Serial.println();
  delay(500);
}
 
// encoder event for the interrupt call
void leftEncoderEvent() {
  if (digitalRead(LH_ENCODER_A) == HIGH) {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCount++;
    } else {
      leftCount--;
    }
  } else {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCount--;
    } else {
      leftCount++;
    }
  }
}
 
// encoder event for the interrupt call
void rightEncoderEvent() {
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  } else {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  }
}
