/*
* --- MOTOR CONTROLLER
*/
#define M0S 2
#define M0E A2
#define M0D A3

#define M1S 3
#define M1E A4
#define M1D A5

#define M2S 4
#define M2E A6
#define M2D A7

#define M3S 5
#define M3E A8
#define M3D A9

#define MOTOR_VEL_MAX 3.49066 // rad/s corresponding to 2000 rounds/min (reduction of 60)
#define MOTOR_VEL_MIN 0.174533 // rad/s corresponding to 100 rounds/min
#define MOTOR_PWM_MAX 229.0 // corresponding to 90% of 255
#define MOTOR_PWM_MIN 26.0 // corresponding to 10% of 255
#define MOTOR_PWM_VEL_DELTA float((MOTOR_PWM_MAX-MOTOR_PWM_MIN)/(MOTOR_VEL_MAX-MOTOR_VEL_MIN))
#define MOTOR_PWM_ZERO_VEL float(MOTOR_PWM_MIN - (MOTOR_PWM_VEL_DELTA*MOTOR_VEL_MIN))

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
        return true;
      } else { // move backwards
        return false;
      }
      break;
    case 2:
    case 3: // motor_id = 2 or 3 (motor on the right side)
      if(motor_vel>0) { // move frontwards
        return false;
      } else { // move backwards
        return true;
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


void setup()
{
  setupMotors(); // init. motors
}

void loop()
{
  float vel[4] = {0,0,0,0};  // angular velocitiy in rad/s

  for(int i=0; i<4; i++) {
    setMotorCommand(vel[i], i);
  }
}
