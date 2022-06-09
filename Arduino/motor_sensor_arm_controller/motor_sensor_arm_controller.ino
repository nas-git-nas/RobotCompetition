/*
* --- MOTOR CONTROLLER
*/
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
#include <NewPing.h>
#include <Servo.h>
#include <DynamixelSerial.h>


/*
 * --- CONSTANTS
 */
#define M0S 2   // motor on the right
#define M0E A2
#define M0D A3

#define M1S 3   // motor on the right
#define M1E A4
#define M1D A5

#define M2S 4   // motor on the left
#define M2E A6
#define M2D A7

#define M3S 5   // motor on the left
#define M3E A8
#define M3D A9

#define MOTOR_VEL_MAX 3.49066 // rad/s corresponding to 2000 rounds/min (reduction of 60)
#define MOTOR_VEL_MIN 0.174533 // rad/s corresponding to 100 rounds/min
#define MOTOR_PWM_MAX 229.0 // corresponding to 90% of 255
#define MOTOR_PWM_MIN 26.0 // corresponding to 10% of 255
#define MOTOR_PWM_VEL_DELTA float((MOTOR_PWM_MAX-MOTOR_PWM_MIN)/(MOTOR_VEL_MAX-MOTOR_VEL_MIN))
#define MOTOR_PWM_ZERO_VEL float(MOTOR_PWM_MIN - (MOTOR_PWM_VEL_DELTA*MOTOR_VEL_MIN))

#define SONAR_NUM     7 // number of sensors.
#define MAX_DISTANCE 50 // maximum distance (in cm) to ping.
#define PING_INTERVAL 35 // milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

#define PICK 15 // position of arm when picks up the bottle
#define REST 80 // position of arm when at rest
#define DROP 125 // position of arm when drops the bottle
#define FAST 3 // high speed of arm
#define SLOW 1 // low speed of arm

#define PUMP_PIN 25 // pin for the pump


#define DYN_ID 3
#define DYN_BAUD 1000000
#define DYN_MAX_TORQUE 768
#define BASKET_SPEED 150
#define BASKET_LOW 218
#define BASKET_HIGH 385

/*
 * --- GLOBAL VARIABLES
 */
float vel[4] = {0,0,0,0};  // angular velocitiy in rad/s
bool new_motor_command = false; // command from ROS to activate motor
bool arm_pos_feedback = false; // feedback on the arm position for ROS
bool arm_pos_command = false; // command from ROS to activate arm
bool basket_command = false; // command from ROS to activate basket

unsigned long pingTimer[SONAR_NUM]; // holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // where the ping distances are stored.
uint8_t currentSensor = 0;          // keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // sensor object array.
  NewPing(30, 31, MAX_DISTANCE), // each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(32, 33, MAX_DISTANCE),
  NewPing(34, 35, MAX_DISTANCE),
  NewPing(36, 37, MAX_DISTANCE),
  NewPing(38, 39, MAX_DISTANCE),
  NewPing(40, 41, MAX_DISTANCE),
  NewPing(42, 43, MAX_DISTANCE)
};

Servo myservo; // create servo object to control a servo


/*
 * --- CALL BACK FCT.
 */
void raspberryCB( const std_msgs::Float32MultiArray& rasp2ard_msg){
  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led

  for(int i=0; i<4; i++) {
    vel[i] = rasp2ard_msg.data[i];
  }
  new_motor_command = true;

  if(rasp2ard_msg.data[4]==1) {
    arm_pos_command = true;
  }
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}


/*
 * ROS
 */
ros::NodeHandle  nh;
std_msgs::Int16MultiArray ard2rasp_msg;
ros::Publisher pub("ard2rasp", &ard2rasp_msg);
ros::Subscriber<std_msgs::Float32MultiArray> sub("motor_vel", &raspberryCB);


/*
 * --- WHEEL MOTOR FCT.
 */
void setupMotors(void)
{
    // define output pins
    pinMode(M0S, OUTPUT);
    pinMode(M0E, OUTPUT);
    pinMode(M0D, OUTPUT);
    pinMode(M1S, OUTPUT);
    pinMode(M1E, OUTPUT);
    pinMode(M1D, OUTPUT);
    pinMode(M2S, OUTPUT);
    pinMode(M2E, OUTPUT);
    pinMode(M2D, OUTPUT);
    pinMode(M3S, OUTPUT);
    pinMode(M3E, OUTPUT);
    pinMode(M3D, OUTPUT);
    
    // disable all motors
    digitalWrite(M0E, LOW);
    digitalWrite(M1E, LOW);
    digitalWrite(M2E, LOW);
    digitalWrite(M3E, LOW);

    // set direction frontwards
    digitalWrite(M0D, HIGH);
    digitalWrite(M1D, HIGH);
    digitalWrite(M2D, LOW);
    digitalWrite(M3D, LOW);
}

void setMotorCommand(float motor_vel, uint8_t motor_id)
{
  bool direction = motorCalcDirection(motor_vel, motor_id);
  uint8_t motor_pwm = motorCalcPWM(motor_vel);

  bool enable = true;
  if(motor_pwm==0) {
    enable = false;
  }

  switch(motor_id) {
    case 0:
      digitalWrite(M0E, enable);
      digitalWrite(M0D, direction);
      analogWrite(M0S, motor_pwm);
      break;
    case 1:
      digitalWrite(M1E, enable);
      digitalWrite(M1D, direction);
      analogWrite(M1S, motor_pwm);
      break;
    case 2:
      digitalWrite(M2E, enable);
      digitalWrite(M2D, direction);
      analogWrite(M2S, motor_pwm);
      break;
    case 3:
      digitalWrite(M3E, enable);
      digitalWrite(M3D, direction);
      analogWrite(M3S, motor_pwm);
      break;    
  }
}

bool motorCalcDirection(float motor_vel, uint8_t motor_id)
{
  switch(motor_id) {
    case 0:
    case 1: // motor_id = 0 or 1 (motor on the right side)
      if(motor_vel>0) { // move frontwards
        return false;
      } else { // move backwards
        return true;
      }
      break;
    case 2:
    case 3: // motor_id = 2 or 3 (motor on the right side)
      if(motor_vel>0) { // move frontwards
        return true;
      } else { // move backwards
        return false;
      }
      break;  
  }
  return true;
}

uint8_t motorCalcPWM(float motor_vel)
{
  if(motor_vel<0) {
    motor_vel = -motor_vel;
  }
  
  if(motor_vel==0) {
    return 0;
  } else if(motor_vel>=MOTOR_VEL_MAX) {
    return uint8_t(MOTOR_PWM_MAX);
  } else if(motor_vel<=MOTOR_VEL_MIN && motor_vel>0) {
    return uint8_t(MOTOR_PWM_MIN);
  } else {
    return uint8_t(MOTOR_PWM_ZERO_VEL + MOTOR_PWM_VEL_DELTA*motor_vel);
  }
  return 0;
}


/*
 * --- ULTRASONIC FCT.
 */
void oneSensorCycle() 
{ // sensor ping cycle complete, do something with the results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
  Serial.println();

  // send ultrasonic measurements to raspberry
  for(int i=0; i<SONAR_NUM; i++) {
    ard2rasp_msg.data[i] = cm[i];
  }
  pub.publish(&ard2rasp_msg);
}

void processUltrasonicSensors(void)
{
  // loop through all the sensors
  for (uint8_t i = 0; i < SONAR_NUM; i++) { 
    if (millis() >= pingTimer[i]) {         // is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) {
        oneSensorCycle(); // sensor ping cycle complete, do something with the results.
      }
      sonar[currentSensor].timer_stop();          // make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // sensor being accessed.
      cm[currentSensor] = 0;                      // make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
}


/*
 * --- ARM FCT.
 */
void setupArm(void)
{
  myservo.attach(10); // attaches the servo on pin 10 to the servo object
  pinMode(PUMP_PIN, OUTPUT);
}

bool arm_pos(int init_pos, int final_pos, int high_speed, int slow_speed) {
  if (init_pos < final_pos) {
    for(int pos = init_pos; pos <= final_pos-10; pos += high_speed) {
      myservo.write(pos);
      delay(40);
    }
    for(int pos = final_pos-10; pos <= final_pos; pos += slow_speed) {
      myservo.write(pos);
      delay(20);
    }
  }
  else {
    for(int pos = init_pos; pos >= final_pos+10; pos -= high_speed) {
      myservo.write(pos);
      delay(40);
    }
    for(int pos = final_pos+10; pos >= final_pos; pos -= slow_speed) {
      myservo.write(pos);
      delay(20);
    }
  }
  return true;
}

void processArmMovement(void)
{
  //digitalWrite(PUMP_PIN, HIGH); // the pump is ON
  delay(20);
  arm_pos_feedback = arm_pos(REST, PICK, FAST, SLOW); // arm picks up bottle
  delay(500);
  arm_pos_feedback = false;
  arm_pos_feedback = arm_pos(PICK, DROP, FAST, SLOW); // arm drops bottle
  delay(20);
  digitalWrite(PUMP_PIN, LOW); // the pump is OFF
  delay(300);
  arm_pos_feedback = false;
  arm_pos_feedback = arm_pos(DROP, REST, FAST, SLOW); // arm returns to rest position
  delay(20);
}

/*
 * --- Basket FCT.
 */

void initBasket(void)
{
  Dynamixel.begin(DYN_BAUD);
  Dynamixel.setMaxTorque(DYN_ID,DYN_MAX_TORQUE);
  Dynamixel.setEndless(DYN_ID,OFF);
}

void liftBasket(void)
{
  Dynamixel.moveSpeed(DYN_ID, BASKET_HIGH, BASKET_SPEED); 
  Dynamixel.action();
  delay(100);
  while (Dynamixel.moving(DYN_ID));
  delay(200);
}
void retractBasket(void)
{
  Dynamixel.moveSpeed(DYN_ID, BASKET_LOW, BASKET_SPEED);  // Move the Servo radomly from 200 to 800
  Dynamixel.action();
  delay(100);
  while (Dynamixel.moving(DYN_ID));
  delay(200);   
}

void emptyBasket(void)
{
  liftBasket();
  delay(2000);
  retractBasket();
  delay(100);
}


/*
 * --- MAIN
 */
void setup()
{
  setupMotors(); // init. motors
  //setupArm(); // init. arm
  initBasket(); // init. basket

  pinMode(LED_BUILTIN, OUTPUT);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  ard2rasp_msg.data = (int*)malloc(sizeof(float) * 7);
  ard2rasp_msg.data_length = 7;

  Serial.begin(57600);
  pingTimer[0] = millis() + 75;
  for(uint8_t i=1; i<SONAR_NUM; i++) {
    pingTimer[i] = pingTimer[i-1] + PING_INTERVAL;
  }
}

void loop()
{
  nh.spinOnce();
  
  if(new_motor_command) {
    for(int i=0; i<4; i++) {
      setMotorCommand(vel[i], i);
    }
    new_motor_command = false;
  }
  
  // process all ultrasonic sensors for which the waiting time is over
  processUltrasonicSensors();


  if(arm_pos_command) {
    processArmMovement();
    arm_pos_command = false;
  }

  if(basket_command){
    emptyBasket();
    basket_command = false;
  }
     
}
