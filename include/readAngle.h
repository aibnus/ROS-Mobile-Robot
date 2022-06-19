#ifndef _readAngle_H_
#define _readAngle_H_

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps612.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define INTERRUPT_PIN 13

class readAngle {
  public:
    void initMPU();
    void readData();
    
    // Variable
    // float yaw, roll, pitch;
    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    
  private:
    // MPU control/status vars
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    
    // orientation/motion vars
    VectorFloat gravity;    // [x, y, z]            gravity vector
    // float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
};

#endif