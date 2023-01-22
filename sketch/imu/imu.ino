#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#define INTERRUPT_PIN 7

// ROS data
ros::NodeHandle  nh;
geometry_msgs::Quaternion orientation;
ros::Publisher orientation_pub("imu_orientation", &orientation);

// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// orientation/motion vars
Quaternion q;
VectorInt16 aa;
VectorInt16 gy;
VectorFloat gravity;
float ypr[3];

void setup()
{
  nh.initNode();
  nh.advertise(orientation_pub);
  imuSetup();
}

void loop()
{
  getImuData();
  orientation.x = q.x;
  orientation.y = q.y;
  orientation.z = q.z;
  orientation.w = q.w;
  orientation_pub.publish( &orientation );
  nh.spinOnce();
  delay(200);
}

// Setup I2C link with MPU6050 and initialize DMP
void imuSetup()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  pinMode(INTERRUPT_PIN, INPUT);
  
  mpu.initialize();
  devStatus = mpu.dmpInitialize();  

  // If DMP has intialized, continue setup
  if (devStatus == 0) 
  {
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    dmpReady = true;
  }
}

void getImuData()
{
  if (!dmpReady) return;  // if programming failed, don't try to do anything

  // Get the Latest packet 
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  { 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
}

void dmpDataReady() {
  mpuInterrupt = true;
}
