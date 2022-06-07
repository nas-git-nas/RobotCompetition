#include <Servo.h>

#define PICKUP 18
#define REST 80
#define DROP 120
#define SPEED 3
#define SLOW 1

#define pumpPin 25 // pin for the pump

Servo myservo;  // create servo object to control a servo

void setup() {
  myservo.attach(10);  // attaches the servo on pin 10 to the servo object
  pinMode(pumpPin, OUTPUT);
}

void loop() {
  // the robot sees the bottle
  bool condition = false;
  // the robot picks up the bottle
  digitalWrite(pumpPin, HIGH);
  condition = arm_pos(REST, PICKUP, SPEED, SLOW);
  delay(20);
  condition = false;
  // the robot drops the bottle
  condition = arm_pos(PICKUP, DROP, SPEED, SLOW);
  delay(20);
  digitalWrite(pumpPin, LOW);
  delay(20);
  condition = false;
  // the robot returns to the rest position
  condition = arm_pos(DROP, REST, SPEED, SLOW);
  delay(20);
  delay(5000);
}

bool arm_pos(int init_pos, int final_pos, int high_speed, int slow_speed) {
  if (init_pos < final_pos) {
    for(int pos = init_pos; pos >= final_pos-10; pos -= high_speed) {
      myservo.write(pos);
      delay(40);
    }
    for(int pos = final_pos-10; pos >= final_pos; pos -= slow_speed) {
      myservo.write(pos);
      delay(20);
    }
  }
  else {
    for(int pos = init_pos; pos <= final_pos-10; pos += high_speed) {
      myservo.write(pos);
      delay(40);
    }
    for(int pos = final_pos-10; pos <= final_pos; pos += slow_speed) {
      myservo.write(pos);
      delay(20);
    }
  }
  return true;
}
