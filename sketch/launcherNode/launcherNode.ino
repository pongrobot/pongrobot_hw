#define USE_USBCON
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <Servo.h>
#include <AccelStepper.h>

// ---------------------------------------------------
// CONSTANTS

// Yaw Swivel states
#define SWIVEL_IDLE 0                   // The swivel is idle.
#define SWIVEL_CALIBRATE_BOUNDS_LOW 1   // Swivel is calibrating a lower bound.
#define SWIVEL_CALIBRATE_BOUNDS_HIGH 2  // Swivel is calibrating an upper bound. 
#define SWIVEL_MOVING 3                 // Swivel moving towards a position.

// Yaw Swivel constants
#define SWIVEL_STEP_ENABLE 7
#define SWIVEL_STEP_PIN 9
#define SWIVEL_DIRECTION_PIN 8
#define SWIVEL_MICROSTEP_PIN_1 4
#define SWIVEL_MICROSTEP_PIN_2 5
#define SWIVEL_MICROSTEP_PIN_3 6
#define SWIVEL_LIMIT_LOW_PIN 2
#define SWIVEL_LIMIT_HIGH_PIN 4

// Launcher states
#define LAUNCHER_IDLE 0               // The launcher is idle.
#define LAUNCHER_EXTENDING 1          // Launcher is extending, to release a ball.
#define LAUNCHER_RETRACTING 2         // Launcher is retracting/resetting.

// Launcher Constants
#define LAUNCHER_EXTENDED_POS 10      // "Extended" servo position
#define LAUNCHER_RETRACTED_POS 155    // "Retracted" servo position
#define LAUNCHER_SERVO_PIN 10         // What pin the launcher servo is on
#define LAUNCHER_EXTEND_DURATION 500  // Time it takes the launcher servo to extend
#define LAUNCHER_RETRACT_DURATION 500 // Time it takes the launcher servo to retract

// Ball Detector constants
#define BALL_DETECTOR_PIN 15

// ---------------------------------------------------
// GLOBAL VARIABLES

// Launcher variables & state 
int iLauncherState = LAUNCHER_IDLE;
unsigned long lLauncherLastAction = 0;
bool bWantsLaunch = false;
bool bHasCommand = false;
Servo launcherServo; 

// Ball Detector state
bool bHasBall = false;
bool bHasBallLatch = false;

// Swivel variables & state
int iSwivelState = SWIVEL_CALIBRATE_BOUNDS_LOW;
long lSwivelHighPos = 0;
long lSwivelNewTargetPos = 0;
long lSwivelTargetPos = 0;
long lSwivelLastYawMsg = 0;
long lSwivelRandomNoiseAmplitude = 0;
bool bSwivelClearedEndStop = false;
bool bYawIsReady = false;
bool bHasNewCommand = false;
AccelStepper stepper(AccelStepper::DRIVER, SWIVEL_STEP_PIN, SWIVEL_DIRECTION_PIN);

unsigned long lLastHeartbeatMsg = 0;

void(* resetFunc) (void) = 0;

// ---------------------------------------------------
// ROS SETUP

ros::NodeHandle nh;

void triggerCommandCallback(const std_msgs::Empty& launch_msg) {
  bWantsLaunch = true;
}

void yawCommandCallback(const std_msgs::Int8& yaw_msg) {
  // Yaw message will be a value from -90 to 90 inclusive
  // Convert this to a target position.

  // Compute a random noise term that increases as we target the same position multiple times
  // Each time we receive a command that duplicates a previous angle, add some uncertainty to the measurement
  if (lSwivelLastYawMsg == yaw_msg.data) {
    lSwivelRandomNoiseAmplitude += 1;
    if (lSwivelRandomNoiseAmplitude > 10) {
      lSwivelRandomNoiseAmplitude = 10;
    }
  } else {
    // Reset the uncertainty when we get a new position
    // So the launcher starts very accurately and slowly gets more random
    lSwivelRandomNoiseAmplitude = 0;
  }
  lSwivelLastYawMsg = yaw_msg.data;
  int randomNoise = random(-lSwivelRandomNoiseAmplitude/5, lSwivelRandomNoiseAmplitude/5);
  lSwivelNewTargetPos = constrain(map(-yaw_msg.data + randomNoise, -90, 90, 0, lSwivelHighPos),0, lSwivelHighPos);
  
  if (lSwivelNewTargetPos != lSwivelTargetPos || bHasCommand == false) {
    lSwivelTargetPos = lSwivelNewTargetPos;
    bYawIsReady = false;
    bHasCommand = true;
    bHasNewCommand = true;
  }
}

void resetCommandCallback(const std_msgs::Empty& reset_msg) {
    // Completely reset the system to get out of a bad state.
    resetFunc();
}

ros::Subscriber<std_msgs::Int8> yawSubscriber("yaw_cmd", &yawCommandCallback);
ros::Subscriber<std_msgs::Empty> triggerSubscriber("trigger", &triggerCommandCallback);
ros::Subscriber<std_msgs::Empty> resetSubscriber("reset", &resetCommandCallback);

std_msgs::Bool ready_msg;
ros::Publisher readyPublisher("yaw_ready", &ready_msg);

std_msgs::Bool ball_msg;
ros::Publisher ballPublisher("has_ball", &ball_msg);

// ---------------------------------------------------
// ROS SETUP

void sendBallMsg() {
  ball_msg.data = bHasBall;
  ballPublisher.publish(&ball_msg);
}

void sendReadyMsg() {
  ready_msg.data = bYawIsReady && bHasCommand;
  readyPublisher.publish(&ready_msg);
}

void setup() {
  // Initialize launcher servo
  launcherServo.attach(LAUNCHER_SERVO_PIN);
  launcherServo.write(LAUNCHER_RETRACTED_POS);

  // Initialize stepper motor
  pinMode(SWIVEL_STEP_ENABLE, OUTPUT);
  digitalWrite(SWIVEL_STEP_ENABLE, HIGH);
  
  // Set to use full microstepping
  pinMode(SWIVEL_MICROSTEP_PIN_1, OUTPUT);
  pinMode(SWIVEL_MICROSTEP_PIN_2, OUTPUT);
  pinMode(SWIVEL_MICROSTEP_PIN_3, OUTPUT);
  digitalWrite(SWIVEL_MICROSTEP_PIN_1, HIGH);
  digitalWrite(SWIVEL_MICROSTEP_PIN_2, HIGH);
  digitalWrite(SWIVEL_MICROSTEP_PIN_3, HIGH);
  
  // Stepper limit switches
  pinMode(SWIVEL_LIMIT_LOW_PIN, INPUT_PULLUP);
  pinMode(SWIVEL_LIMIT_HIGH_PIN, INPUT_PULLUP);

  // Ball detector
  pinMode(BALL_DETECTOR_PIN, INPUT_PULLUP);
  
  // Set max speed and acceleration for the stepper
  stepper.setSpeed(1500);
  stepper.setMaxSpeed(1500);
  stepper.setAcceleration(2000);

  // Start calibration mode
  iSwivelState = SWIVEL_CALIBRATE_BOUNDS_LOW;
  
  // Init ROS node and setup topics
  nh.initNode();
  //nh.advertise(swivelStatePublisher);
  //nh.advertise(launcherStatePublisher);
  nh.subscribe(yawSubscriber);
  nh.subscribe(triggerSubscriber);
  nh.subscribe(resetSubscriber);
  nh.advertise(readyPublisher);
  nh.advertise(ballPublisher);

}

// ---------------------------------------------------
// MAIN LOOP

void loop() {
  if (!nh.connected()) {
    //digitalWrite(SWIVEL_STEP_ENABLE, LOW);
    nh.spinOnce();

    if (nh.connected()) {
      //digitalWrite(SWIVEL_STEP_ENABLE, HIGH);
      //iSwivelState = SWIVEL_CALIBRATE_BOUNDS_LOW;
      bYawIsReady = false;
      sendReadyMsg();
      nh.spinOnce();
    }
    return;
  }

  // Check limit switches
  bool limitLow = !digitalRead(SWIVEL_LIMIT_LOW_PIN);
  bool limitHigh = !digitalRead(SWIVEL_LIMIT_HIGH_PIN);

  // Check ball detector
  // Latch so once it's on, needs to be reset manually
  bHasBall = !digitalRead(BALL_DETECTOR_PIN) || bHasBallLatch;

  switch (iSwivelState) {
    case SWIVEL_IDLE: {
      if (bHasNewCommand) {
        iSwivelState = SWIVEL_MOVING;
        bYawIsReady = false;
      } else {
        bYawIsReady = true;
      }

      // Send ball status while we are idle
      if (bHasBall) {
        bHasBallLatch = true;
      }
      sendBallMsg();
      sendReadyMsg();
      nh.spinOnce();
      
      break;
    }
    case SWIVEL_CALIBRATE_BOUNDS_LOW: {
      stepper.moveTo(-10000000);
      stepper.run();
      if (limitLow) {
        stepper.stop();
        iSwivelState = SWIVEL_CALIBRATE_BOUNDS_HIGH;
        stepper.setCurrentPosition(0);
        
        bYawIsReady = false;
        sendReadyMsg();
        nh.spinOnce();
      }
      break;
    }
    case SWIVEL_CALIBRATE_BOUNDS_HIGH: {
      stepper.moveTo(10000000);
      stepper.run();
      if (limitHigh) {
        stepper.stop();
        lSwivelHighPos = stepper.currentPosition();
        stepper.setCurrentPosition(lSwivelHighPos);
        stepper.run();
        
        // set target pos to middle of range
        lSwivelTargetPos = lSwivelHighPos / 2;
        bSwivelClearedEndStop = false;
        
        iSwivelState = SWIVEL_MOVING;
        bYawIsReady = false;
        sendReadyMsg();
        nh.spinOnce();
      }
      break;
    }
    case SWIVEL_MOVING: {
      stepper.moveTo(lSwivelTargetPos);
      stepper.run();
      if (!stepper.isRunning()) {
        iSwivelState = SWIVEL_IDLE;
        bHasNewCommand = false;
        bYawIsReady = true;
        sendReadyMsg();
        nh.spinOnce();
      }
      break;
    }
  }
  
  switch (iLauncherState) {
    case LAUNCHER_IDLE: {
      launcherServo.write(LAUNCHER_RETRACTED_POS);
      if (bWantsLaunch && iSwivelState == SWIVEL_IDLE) {
        launcherServo.write(LAUNCHER_EXTENDED_POS);
        iLauncherState = LAUNCHER_EXTENDING;
        lLauncherLastAction = millis();
      }
      break;
    }
    case LAUNCHER_EXTENDING: {
      // Wait while the launcher is extending, then start retraction.
      if (millis() - lLauncherLastAction > LAUNCHER_EXTEND_DURATION) {
        launcherServo.write(LAUNCHER_RETRACTED_POS);
        iLauncherState = LAUNCHER_RETRACTING;
        lLauncherLastAction = millis();
      }
      break;
    }
    case LAUNCHER_RETRACTING: {
      // Wait while the launcher is retracting, then return to idle.
      if (millis() - lLauncherLastAction > LAUNCHER_RETRACT_DURATION) {
        iLauncherState = LAUNCHER_IDLE;
        bWantsLaunch = false;

        // Reset "Has ball" status after we have launched the ball
        bHasBall = false;
        bHasBallLatch = false;
        sendBallMsg();
        nh.spinOnce();
      }
      break;
    }
  }
}
