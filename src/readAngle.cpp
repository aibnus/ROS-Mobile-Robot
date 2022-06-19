#include "readAngle.h"

MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

void readAngle::initMPU() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity  
  mpu.setXAccelOffset(-1604);
  mpu.setYAccelOffset(-1699);
  mpu.setZAccelOffset(1205);
  mpu.setXGyroOffset(31);
  mpu.setYGyroOffset(33);
  mpu.setZGyroOffset(17);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(10);
    mpu.CalibrateGyro(10);
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
  }
}

void readAngle::readData() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGyro(&gy, fifoBuffer);

    mpu.dmpGetGravity(&gravity, &q);
    
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  }
}