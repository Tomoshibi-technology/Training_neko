#include <Arduino.h>

int M0A_pin = 10;
int M0B_pin = 11;
int M1A_pin = 12;
int M1B_pin = 13;
int M2A_pin = 18;
int M2B_pin = 19;
int M3A_pin = 16;
int M3B_pin = 17;
int strtSW_pin = 14;

int strtSW = 0;
int go_dir = 0;
int go_sp = 0;

void move(int x, int y) {
  int MOVE0 = sin((x + 135.0) / 180.0 * 3.14) * y;
  int MOVE1 = sin((x + 45.0) / 180.0 * 3.14) * y;
  int MOVE2 = sin((x - 45.0) / 180.0 * 3.14) * y;
  int MOVE3 = sin((x - 135.0) / 180.0 * 3.14) * y;

  // int invMOVE0 = abs(MOVE0);
  // int invMOVE1 = abs(MOVE1);
  // int invMOVE2 = abs(MOVE2);
  // int invMOVE3 = abs(MOVE3);

  Serial.println(MOVE0);
  Serial.println(MOVE1);
  Serial.println(MOVE2);
  Serial.println(MOVE3);


  if(MOVE0 > 0) {
    analogWrite(M0A_pin, MOVE0);
    analogWrite(M0B_pin, 0);
  } else if (MOVE0 < 0) {
    analogWrite(M0A_pin, 0);
    analogWrite(M0B_pin, abs(MOVE0));
  } else {
    analogWrite(M0A_pin, 0);
    analogWrite(M0B_pin, 0);
  }

  if(MOVE1 > 0) {
    analogWrite(M1A_pin, MOVE1);
    analogWrite(M1B_pin, 0);
  } else if (MOVE1 < 0) {
    analogWrite(M1A_pin, 0);
    analogWrite(M1B_pin, abs(MOVE1));
  } else {
    analogWrite(M1A_pin, 0);
    analogWrite(M1B_pin, 0);
  }

  if(MOVE2 > 0) {
    analogWrite(M2A_pin, MOVE2);
    analogWrite(M2B_pin, 0);
  } else if (MOVE2 < 0) {
    analogWrite(M2A_pin, 0);
    analogWrite(M2B_pin, abs(MOVE2));
  } else {
    analogWrite(M2A_pin, 0);
    analogWrite(M2B_pin, 0);
  }

  if(MOVE3 > 0) {
    analogWrite(M3A_pin, MOVE3);
    analogWrite(M3B_pin, 0);
  } else if (MOVE3 < 0) {
    analogWrite(M3A_pin, 0);
    analogWrite(M3B_pin, abs(MOVE3));
  } else {
    analogWrite(M3A_pin, 0);
    analogWrite(M3B_pin, 0);
  }

}


void setup() {
  pinMode(21, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(M0A_pin, OUTPUT);
  pinMode(M0B_pin, OUTPUT);
  pinMode(M1A_pin, OUTPUT);
  pinMode(M1B_pin, OUTPUT);
  pinMode(M2A_pin, OUTPUT);
  pinMode(M2B_pin, OUTPUT);
  pinMode(M3A_pin, OUTPUT);
  pinMode(M3B_pin, OUTPUT);
  pinMode(strtSW_pin, INPUT);

  Serial.begin(115200);
}

void loop() {
  strtSW = digitalRead(strtSW_pin);
  if(strtSW == 0){
    go_dir = 0;
    go_sp = 100;
    move(go_dir, go_sp);
    digitalWrite(21, HIGH);
    digitalWrite(22, LOW);
    delay(200);
    digitalWrite(21, LOW);
    digitalWrite(22, HIGH);
    delay(200);


  }else{
    digitalWrite(21, HIGH);
    digitalWrite(22, LOW);
    delay(500);
    digitalWrite(21, LOW);
    digitalWrite(22, HIGH);
    delay(500);

    analogWrite(M0A_pin, 0);
    analogWrite(M0B_pin, 0);
    analogWrite(M1A_pin, 0);
    analogWrite(M1B_pin, 0);
    analogWrite(M2A_pin, 0);
    analogWrite(M2B_pin, 0);
    analogWrite(M3A_pin, 0);
    analogWrite(M3B_pin, 0);
  }
}
