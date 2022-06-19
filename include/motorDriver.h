#ifndef _MOTOR_DRIVER_H_
#define _MOTOR_DRIVER_H_

#include <Arduino.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count
  int PrevErr;                   // last error
  int Ierror;                    // integrated error
  int output;                    // last motor setting
}
pidControl;

class MotorDriver {
    public:
        MotorDriver(int cpr, float wheelDia, int PID_RATE);
        void init(int PWMApin, int AIn2Pin, int AIn1Pin, int STBYpin, int BIn1Pin, int BIn2Pin, int PWMBpin, int offset);

        int speedToTicks(float v);
        void setSpeed(int speedA, int speedB);
        void brake();
        void standby();

        void PID(pidControl *p);
        void PID_Tunings(float Kp, float Ki, float Kd);
        void PID_Clear(pidControl *p);

    private:
        int _Ain1Pin, _Ain2Pin, _pwmAPin, _Bin2Pin, _Bin1Pin, _pwmBPin, _stbyPin, _offset;
        int _pidRate;
        float _kp, _ki, _kd;
        
        int _cpr;
        float _wheelDia;

        void setSpeedA(int speed);
        void setSpeedB(int speed);
        
        #define MAXOUTPUT 255
};

#endif