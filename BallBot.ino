// libraries
#include <DRV8833.h>
#include <Servo.h>
// physical constraints
#define WHEEL_RADIUS 0.02
#define ENCODER_TICKS 450
// pins for inputs/outputs
#define RH_DRV_M1 6
#define RH_DRV_M2 9
#define LH_DRV_M1 11
#define LH_DRV_M2 10
#define RH_ENCODER_A 3
#define RH_ENCODER_B 12
#define LH_ENCODER_A 2
#define LH_ENCODER_B 4
#define SERVO_PIN 5
#define seconds() (millis()/1000.0)

DRV8833 drv = DRV8833();
Servo servo;

class PID {
    float E, e, e_old, e_dot;
    float kP, kI, kD;
  public:
    PID(float kPx, float kIx, float kDx) {
      kP = kPx;
      kI = kIx;
      kD = kDx;
      E = 0;
      e = 0;
      e_old = 0;
      e_dot = 0;
    }
    float input(float e_in) {
      e = e_in;
      E = E + e;
      e_dot = e - e_old;
      e_old = e;
      return kP * e + kI * E + kD * e_dot;
    }
    void restart() {
      E = 0;
      e = 0;
      e_old = 0;
      e_dot = 0;
    }
};

//global variables
PID leftPID = PID(24, 20, 0.1);
PID rightPID = PID(24, 20, 0.1);
float gVR, gVL; // goal velocities for PID
float s0 = 25;
float sR_ = 0, sL_ = 0; // motor baseline speed
float sR = 0, sL = 0; // motor current speed
float dsR = 0, dsL = 0; // adjustment to motor speed
boolean dirR = 1, dirL = 1; // direction of velocity, 0 = reverse, 1 = forward
float leftVel = 0; // right motor velocity
float rightVel = 0; // left motor velocity
byte clawAngle; // current claw position
byte newAngle = 2; // position for claw to be updated to
float brakeTime = 0.2;
volatile float leftCount = 0; // encoder counts
volatile float rightCount = 0; // encoder counts
// state control
enum State_enum {DRIVE, STOP, CLAW};
State_enum state;
// time (in seconds)
float t = seconds();
float t_new;
float dt;

// plotting

void setup() {
  // motor pins
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  drv.attachMotorA(RH_DRV_M1, RH_DRV_M2);
  drv.attachMotorB(LH_DRV_M1, LH_DRV_M2);
  // encoder interrupts
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);

  // servo setup, move claw to open position
  servo.write(newAngle); // update PWM state before movement
  servo.attach(SERVO_PIN);
  clawAngle = servo.read();
  delay(200);
  servo.detach();
  
    while (true) {
      if (Serial.peek() == 'B') { // begin arduino setup
        Serial.read();
        break;
      }
    }

  // motor speed setup, estimate baseline for movement
  gVR = 0.05;
  gVL = 0.05;
  int counter = 0;
  while (counter < 5) { // until both are set
    updateDynamics();
    sR_ = s0 + rightPID.input(gVR - rightVel); // PID correction
    sL_ = s0 + leftPID.input(gVL - leftVel); // PID correction
    drv.motorAForward((int) sR_);
    drv.motorBForward((int) sL_);
    if (abs(gVR - rightVel) < 0.02 && abs(gVL - leftVel) < 0.02) {
      counter++;
    }
    delay(200);
  }
  sR_--;
  sL_--;


  brake();

  
  Serial.begin(9600);
  
  Serial.println("READY"); // notify RPi to begin

  state = STOP;

}

void loop() {
  updateDynamics();
  action();

  if (Serial.available() > 0) { // gets next message
    char msg[Serial.available()];
    byte index = 0;
    while (Serial.available() > 0) {
      if (Serial.peek() == 'n') {
        Serial.read();
        break;
      }
      msg[index] = Serial.read(); // reading each char in
      index++;
    }
    //    Serial.println(msg);
    switch (msg[0]) { // updating states
      case 'D': { // D,0.3,0.3n --> drive both wheels at 0.3 m/s
          char* token = strtok(msg, ",");
          token = strtok(NULL, ",");
          gVR = atof(token);
          token = strtok(NULL, ",");
          gVL = atof(token);
          initDynamics(); // reset the PID when velocities are updated
          state = DRIVE;
          break;
        }
      case 'S': { // S,0.5n --> brake wheels for 0.5 s
          char* token = strtok(msg, ",");
          token = strtok(NULL, ",");
          brakeTime = atof(token);
          state = STOP;
          break;
        }
      case 'C': { // C,10n --> move claw to angle 10
          char* token = strtok(msg, ",");
          token = strtok(NULL, ",");
          newAngle = atoi(token);
          state = CLAW;
          break;
        }
      default: {// catch noise
          //          Serial.println("msg error:");
          //          Serial.println(msg);
        }
    }
  }
  delay(200);
}

void action() {
  switch (state) {
    case DRIVE:
      updateSpeeds();
      break;
    case STOP:
      brake();
      break;
    case CLAW:
      moveClaw();
      break;
  }
}

void initDynamics() {
  leftPID.restart();
  rightPID.restart();
  if (gVR < 0 && dirR == 1) {
    sR_ = -sR_;
    sR = sR_;
    dsR = 0;
    dirR = 0;
  } else if (gVR > 0 && dirR == 0) {
    sR_ = -sR_;
    sR = sR_;
    dsR = 0;
    dirR = 1;
  }
  if (gVL < 0 && dirL == 1) {
    sL_ = -sL_;
    sL = sL_;
    dsL = 0;
    dirL = 0;
  } else if (gVL > 0 && dirL == 0) {
    sL_ = -sL_;
    sL = sL_;
    dsL = 0;
    dirL = 1;
  }
  if (state != DRIVE ) { // reset speed to baseline if not in drive
    sR = sR_;
    sL = sL_;
  } else {
    sR = sR + dsR;
    sL = sL + dsL;
  }

}

void updateDynamics() {
  t_new = seconds();
  dt = t_new - t;
  leftVel = (2 * PI * WHEEL_RADIUS * leftCount) / (ENCODER_TICKS * dt);
  rightVel = (2 * PI * WHEEL_RADIUS * rightCount) / (ENCODER_TICKS * dt);
  t = t_new;

  leftCount = 0;
  rightCount = 0;

//  Serial.print(rightVel);
//  Serial.print(",");
//  Serial.println(leftVel);
}

void updateSpeeds() {
  dsR = rightPID.input(gVR - rightVel);
  dsL = leftPID.input(gVL - leftVel);
//  Serial.print("Right Speed: ");
//  Serial.println(sR + dsR);
//  Serial.print("Left Speed: ");
//  Serial.println(sL + dsL);
//  Serial.println();
  drv.motorAForward((int) sR + dsR);
  drv.motorBForward((int) sL + dsL);
}

void brake() {
  if (abs(leftVel) > 0.01 && abs(rightVel) > 0.01) {
    drv.motorAStop();
    drv.motorBStop();
    delay(brakeTime * 1000);
    drv.motorAForward(0);
    drv.motorBForward(0);
  }
}

void moveClaw() {
  servo.attach(SERVO_PIN);
  if (clawAngle < newAngle) {
    while (clawAngle < newAngle) {
      clawAngle += 1;
      servo.write(clawAngle);
      delay(50);
    }
  } else if (clawAngle > newAngle) {
    while (clawAngle > newAngle) {
      clawAngle -= 1;
      servo.write(clawAngle);
      delay(50);
    }
  }
  servo.detach();
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
