#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Pose.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#include <VL53L0X.h>
#include <Wire.h>
VL53L0X sensor;
uint16_t range;

// ROS data
ros::NodeHandle  nh;
geometry_msgs::Pose pose;
ros::Publisher pose_pub("imu_pose", &pose);

// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;

void setup()
{
  nh.initNode();
  nh.advertise(pose_pub);
  imuSetup();

  Wire.begin();
  sensor.setTimeout(500);
  while (!sensor.init()){}
  sensor.setSignalRateLimit(0.1); // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18); // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor.setMeasurementTimingBudget(200000); // increase timing budget to 200 ms
}

void loop()
{
  getImuData();
  pose.orientation.x = q.x;
  pose.orientation.y = q.y;
  pose.orientation.z = q.z;
  pose.orientation.w = q.w;

  range = sensor.readRangeSingleMillimeters();
  pose.position.x = 0;
  pose.position.y = 0;
  float rangeMm = range/1000.f;
  if ( rangeMm < 2.f )
    pose.position.z = pose.position.z * 0.6+ 0.4 * rangeMm;

  pose_pub.publish( &pose );
  nh.spinOnce();
  delay(100);
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
  
  mpu.initialize();
  devStatus = mpu.dmpInitialize();  

  // If DMP has initalized, continue setup
  if (devStatus == 0) 
  {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
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
  }
}
