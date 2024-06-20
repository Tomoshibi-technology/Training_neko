#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup(void){
  pinMode(4, INPUT_PULLUP); //プルアップ(念のため)
  pinMode(5, INPUT_PULLUP); //プルアップ(念のため)

  Serial.begin(115200);

  if (!bno.begin()){  //こいつら消すと動かんくなる
    while (1);        //こいつら消すと動かんくなる
  }                   //こいつら消すと動かんくなる

  delay(1000);
}

void get_bno055_data(void){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print(euler.x());
  // Serial.print(", ");
  // Serial.print(euler.y());
  // Serial.print(", ");
  // Serial.print(euler.z());

  Serial.println();
}

void loop(void){
  get_bno055_data();
  delay(200);
}