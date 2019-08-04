int LFmotorEn = 1; //digital
int LFmotorM1 = 2; //PWM
int LFmotorM2 = 3; //PWM
int LBmotorEn = 4; //digital
int LBmotorM1 = 5; //PWM
int LBmotorM2 = 6; //PWM
int RFmotorEn = 7; //digital
int RFmotorM1 = 8; //PWM
int RFmotorM2 = 9; //PWM
int RBmotorEn = 10; //digital
int RBmotorM1 = 11; //PWM
int RBmotorM2 = 12; //PWM
int LFencoder1 = 13; //analog
int LFencoder2 = 14; //analog
int LBencoder1 = 15; //analog
int LBencoder2 = 16; //analog
int RFencoder1 = 17; //analog
int RFencoder2 = 18; //analog
int RBencoder1 = 19; //analog
int RBencoder2 = 20; //analog

volatile signed long LFCount = 0;

volatile signed long LBCount = 0;

volatile signed long RFCount = 0;

volatile signed long RBCount = 0;


void setup() {
  pinMode(LFmotorEn , OUTPUT);
  pinMode(LFmotorM1 , OUTPUT);
  pinMode(LFmotorM2 , OUTPUT);
  digitalWrite(LFmotorEn, LOW);
  pinMode(LBmotorEn , OUTPUT);
  pinMode(LBmotorM1 , OUTPUT);
  pinMode(LBmotorM2 , OUTPUT);
  digitalWrite(LBmotorEn, LOW);
  pinMode(RFmotorEn , OUTPUT);
  pinMode(RFmotorM1 , OUTPUT);
  pinMode(RFmotorM2 , OUTPUT);
  digitalWrite(RFmotorEn, LOW);
  pinMode(RBmotorEn , OUTPUT);
  pinMode(RBmotorM1 , OUTPUT);
  pinMode(RBmotorM2 , OUTPUT);
  digitalWrite(RBmotorEn, LOW);
  pinMode(LFencoder1, INPUT);
  pinMode(LFencoder2, INPUT);
  pinMode(LBencoder1, INPUT);
  pinMode(LBencoder2, INPUT);
  pinMode(RFencoder1, INPUT);
  pinMode(RFencoder2, INPUT);
  pinMode(RBencoder1, INPUT);
  pinMode(RBencoder2, INPUT);
}

void loop() {

}
void stay() {
  digitalWrite(LFmotorEn, HIGH);
  digitalWrite(LFmotorM1, LOW );
  digitalWrite(LFmotorM2, LOW );
  digitalWrite(LBmotorEn, HIGH);
  digitalWrite(LBmotorM1, LOW );
  digitalWrite(LBmotorM2, LOW );
  digitalWrite(RFmotorEn, HIGH);
  digitalWrite(RFmotorM1, LOW );
  digitalWrite(RFmotorM2, LOW );
  digitalWrite(RBmotorEn, HIGH);
  digitalWrite(RBmotorM1, LOW );
  digitalWrite(RBmotorM2, LOW );
}
void forward() {
  digitalWrite(LFmotorEn, HIGH);
  digitalWrite(LFmotorM1, HIGH);
  digitalWrite(LFmotorM2, LOW );
  digitalWrite(LBmotorEn, HIGH);
  digitalWrite(LBmotorM1, HIGH);
  digitalWrite(LBmotorM2, LOW );
  digitalWrite(RFmotorEn, HIGH);
  digitalWrite(RFmotorM1, LOW );
  digitalWrite(RFmotorM2, HIGH);
  digitalWrite(RBmotorEn, HIGH);
  digitalWrite(RBmotorM1, LOW );
  digitalWrite(RBmotorM2, HIGH);
}
void backward() {
  digitalWrite(LFmotorEn, HIGH);
  digitalWrite(LFmotorM2, HIGH);
  digitalWrite(LFmotorM1, LOW );
  digitalWrite(LBmotorEn, HIGH);
  digitalWrite(LBmotorM2, HIGH);
  digitalWrite(LBmotorM1, LOW );
  digitalWrite(RFmotorEn, HIGH);
  digitalWrite(RFmotorM2, LOW );
  digitalWrite(RFmotorM1, HIGH);
  digitalWrite(RBmotorEn, HIGH);
  digitalWrite(RBmotorM2, LOW );
  digitalWrite(RBmotorM1, HIGH);
}
void left() {
  digitalWrite(LFmotorEn, HIGH);
  digitalWrite(LFmotorM1, LOW );
  digitalWrite(LFmotorM2, HIGH);
  digitalWrite(LBmotorEn, HIGH);
  digitalWrite(LBmotorM1, LOW );
  digitalWrite(LBmotorM2, HIGH);
  digitalWrite(RFmotorEn, HIGH);
  digitalWrite(RFmotorM1, LOW );
  digitalWrite(RFmotorM2, HIGH);
  digitalWrite(RBmotorEn, HIGH);
  digitalWrite(RBmotorM1, LOW );
  digitalWrite(RBmotorM2, HIGH);
}
void right() {
  digitalWrite(LFmotorEn, HIGH);
  digitalWrite(LFmotorM1, HIGH);
  digitalWrite(LFmotorM2, LOW );
  digitalWrite(LBmotorEn, HIGH);
  digitalWrite(LBmotorM1, HIGH);
  digitalWrite(LBmotorM2, LOW );
  digitalWrite(RFmotorEn, HIGH);
  digitalWrite(RFmotorM1, HIGH);
  digitalWrite(RFmotorM2, LOW );
  digitalWrite(RBmotorEn, HIGH);
  digitalWrite(RBmotorM1, HIGH);
  digitalWrite(RBmotorM2, LOW );
}
void LFcounter() {
  if (digitalRead(LFencoder1) == HIGH) {
    if (digitalRead(LFencoder2) == LOW) {
      LFCount++;
    } else {
      LFCount--;
    }
  } else {
    if (digitalRead(LFencoder2) == LOW) {
      LFCount--;
    } else {
      LFCount++;
    }
  }
}
void LBcounter() {
  if (digitalRead(LBencoder1) == HIGH) {
    if (digitalRead(LBencoder2) == LOW) {
      LBCount++;
    } else {
      LBCount--;
    }
  } else {
    if (digitalRead(LBencoder2) == LOW) {
      LBCount--;
    } else {
      LBCount++;
    }
  }
}
void RFcounter() {
  if (digitalRead(RFencoder1) == HIGH) {
    if (digitalRead(RFencoder2) == LOW) {
      RFCount++;
    } else {
      RFCount--;
    }
  } else {
    if (digitalRead(RFencoder2) == LOW) {
      RFCount--;
    } else {
      RFCount++;
    }
  }
}
void RBCounter() {
  if (digitalRead(RBencoder1) == HIGH) {
    if (digitalRead(RBencoder2) == LOW) {
      RBCount++;
    } else {
      RBCount--;
    }
  } else {
    if (digitalRead(RBencoder2) == LOW) {
      RBCount--;
    } else {
      RBCount++;
    }
  }
}
