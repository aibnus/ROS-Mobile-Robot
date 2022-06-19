/*
 * Mobile Robot w/ Teensy 3.2
 *  - ROS Control
 *  - Autonomous
 * 
 * MPU-6050 IMU, Motor + Encoder
 * Library to be needed: 
 *  - I2C_dev.h
 *  - MPU6050_6Axis_MotionApps612.h
 *  - TaskScheduler.h
 */


//===== IMU ====//
//==============//
#include "readAngle.h"
readAngle imu;



//===== MOTOR =====//
//=================//
//* Define motor driver pin on teensy3.2
#define PWMA 4
#define AIN2 5
#define AIN1 6
#define STBY 7
#define BIN1 8
#define BIN2 9
#define PWMB 10

#define encLA 3
#define encLB 2
#define encRA 12
#define encRB 11

#define readRA digitalReadFast(encRA) //faster than digitalRead()
#define readRB digitalReadFast(encRB)
#define readLA digitalReadFast(encLA)
#define readLB digitalReadFast(encLB)

const float wheelDia      = 70.0;           //Diameter of wheel 70 mm
const int   countsPerRev  = 1012;           //11 ppr * 46 gear ratio * (2 raising/falling edge)
const float wheelDistance = 135.0;          //Distance between wheel 135 mm
const float ticksPerMM    = (PI * wheelDia) /  countsPerRev;

volatile int32_t countL, countR;

#include "motorDriver.h"
MotorDriver mtr(countsPerRev, wheelDia/1000.0, 50);
pidControl  leftPID, rightPID;

int32_t posL_old, posR_old, posL_diff, posR_diff;
float posL_diff_mm, posR_diff_mm, pos_avg_diff, pos_total;



//===== ROS ====//
//==============//
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
// #include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

ros::NodeHandle                 nh;
// geometry_msgs::TransformStamped t;
nav_msgs::Odometry              odom_msg;
sensor_msgs::Imu                imu_msg;
std_msgs::Float32               battery_msg;
// tf::TransformBroadcaster        broadcaster;

char base_link[] = "base_link";
char odom[]      = "odom";

float throttle, steering;
float batt_voltage;

double x, y, theta;

bool moving;

void velCallback(const geometry_msgs::Twist &vel) {
  throttle = vel.linear.x;
  steering = vel.angular.z;

  if (throttle == 0 && steering == 0) {
    mtr.brake();
    mtr.PID_Clear(&leftPID);
    mtr.PID_Clear(&rightPID);
    moving = 0;
    return;
  }

  moving = 1;

  leftPID.TargetTicksPerFrame  = mtr.speedToTicks(throttle - steering * wheelDistance / 400.0);
  rightPID.TargetTicksPerFrame = mtr.speedToTicks(throttle + steering * wheelDistance / 400.0);
}

ros::Publisher                        odom_pub ("odom", &odom_msg);
ros::Publisher                        imu_pub  ("imu", &imu_msg);
ros::Publisher                        batt_pub ("batt_voltage", &battery_msg);
ros::Subscriber<geometry_msgs::Twist> vel_sub  ("cmd_vel", velCallback);



//===== SCHEDULER =====//
//=====================//
#include "TaskScheduler.h"
Scheduler runner;
//Create the main loop task that runs every 10 milliseconds
void updatePID();
void updateOdom();
Task tUpdatePID  (20, TASK_FOREVER, &updatePID);
Task tupdateOdom (100, TASK_FOREVER, &updateOdom);



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void encoderR() {
  if (readRA == readRB) countR++;
  else                  countR--;
}

void encoderL() {
  if (readLA != readLB) countL++;
  else                  countL--;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  imu.initMPU();
  mtr.init(PWMA, AIN2, AIN1, STBY, BIN1, BIN2, PWMB, -1);

  pinMode(encLA, INPUT);
  pinMode(encLB, INPUT);
  pinMode(encRA, INPUT);
  pinMode(encRB, INPUT);

  // enable interrupt for MPU-6050
  attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
  // enable interrupt for encoder
  attachInterrupt(encRA, encoderR, CHANGE);
  attachInterrupt(encLA, encoderL, CHANGE);

  //Scheduler
  runner.addTask(tUpdatePID);
  runner.addTask(tupdateOdom);
  tUpdatePID.enable();
  tupdateOdom.enable();

  //ROS
  nh.initNode();
  nh.subscribe(vel_sub);
  nh.advertise(odom_pub);
  nh.advertise(imu_pub);
  nh.advertise(batt_pub);

  // broadcaster.init(nh);

  //Load the battery voltage to the battery_voltage variable.
  //Teensy uses a default 10 bit analog to digital converter.
  //analogRead => 0 = 0V ..... 1023 = 3.3V
  //The voltage divider (1k & 10k) is 1:11.
  //analogRead => 0 = 0V ..... 1023 = 36.3V
  //1023 / 36.3 = 28.18.
  batt_voltage = (float)analogRead(A1) / 28.18;
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
  runner.execute();
  nh.spinOnce();
}

// ================================================================
// ===                  UPDATE PID CONTROLLER                   ===
// ================================================================
void updatePID() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    leftPID.Encoder  = countL;
    rightPID.Encoder = countR;
  }

  if (!moving) return;

  mtr.PID(&leftPID);
  mtr.PID(&rightPID);

  // Serial.printf("Enc: ");       Serial.print(leftPID.Encoder);
  // Serial.printf(" - ");         Serial.print(rightPID.Encoder);
  // Serial.printf("\tOutput: ");  Serial.print(leftPID.output);
  // Serial.printf(" - ");         Serial.print(rightPID.output);
  // Serial.println();

  mtr.setSpeed(leftPID.output, rightPID.output);
}

// ================================================================
// ===                     UPDATE ODOMETRY                      ===
// ================================================================
void updateOdom() {
  if (mpuInterrupt) imu.readData();

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //352.25 = 28.18 / 0.08.
  batt_voltage = batt_voltage * 0.92 + ((float)analogRead(A1) / 352.25);

  battery_msg.data = batt_voltage;
  batt_pub.publish(&battery_msg);

  
  //Calculate position from encoder data
  posL_diff = leftPID.Encoder - posL_old;
  posR_diff = rightPID.Encoder - posR_old;
  posL_old  = leftPID.Encoder;
  posR_old  = rightPID.Encoder;

  posL_diff_mm = posL_diff * ticksPerMM;
  posR_diff_mm = posR_diff * ticksPerMM;

  pos_avg_diff = (posL_diff_mm + posR_diff_mm) / 2;
  pos_total   += pos_avg_diff;

  theta += (posR_diff_mm - posL_diff_mm) / 360;

  if (theta > PI)  theta -= TWO_PI; //limit upper value
  if (theta < -PI) theta += TWO_PI; //limit lower value

  y += pos_avg_diff * sin(theta);   //y position from start
  x += pos_avg_diff * cos(theta);   //x position from start

  // // Broadcast odom->base_link transform with tf
  // t.header.frame_id = odom;
  // t.child_frame_id  = base_link;

  // t.transform.translation.x = x/1000; //convert to meters
  // t.transform.translation.y = y/1000;
  // t.transform.translation.z = 0;

  // t.transform.rotation  = tf::createQuaternionFromYaw(theta);
  // t.header.stamp        = nh.now();

  // broadcaster.sendTransform(t);

  //Publish odom message
  odom_msg.header.stamp           = nh.now();
  odom_msg.header.frame_id        = odom;
  odom_msg.pose.pose.position.x   = x/1000;
  odom_msg.pose.pose.position.y   = y/1000;
  odom_msg.pose.pose.position.z   = 0;
  odom_msg.pose.pose.orientation  = tf::createQuaternionFromYaw(theta);

  odom_msg.child_frame_id         = base_link;
  odom_msg.twist.twist.linear.x   = ((posL_diff_mm + posR_diff_mm) / 2) / 10;
  odom_msg.twist.twist.linear.y   = 0.0;
  odom_msg.twist.twist.angular.z  = ((posL_diff_mm - posR_diff_mm) / 360) / 100;

  odom_pub.publish(&odom_msg);

  //Publish IMU message
  imu_msg.header.stamp    = nh.now();
  imu_msg.header.frame_id = "imu";

  imu_msg.orientation.w   = imu.q.w;
  imu_msg.orientation.x   = imu.q.x;
  imu_msg.orientation.y   = imu.q.y;
  imu_msg.orientation.z   = imu.q.z;

  imu_msg.linear_acceleration.x = imu.aaReal.x;
  imu_msg.linear_acceleration.y = imu.aaReal.y;
  imu_msg.linear_acceleration.z = imu.aaReal.z;

  imu_msg.angular_velocity.x  = imu.gy.x;
  imu_msg.angular_velocity.y  = imu.gy.y;
  imu_msg.angular_velocity.z  = imu.gy.z;

  imu_pub.publish(&imu_msg);
}