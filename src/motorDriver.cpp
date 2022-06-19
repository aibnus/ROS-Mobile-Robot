#include "motorDriver.h"

MotorDriver::MotorDriver(int cpr, float wheelDia, int PID_RATE) {
    _cpr = cpr;
    _wheelDia = wheelDia;
    _pidRate = PID_RATE;

    _kp = 3;
    _ki = 0;
    _kd = 0.5;
}

void MotorDriver::init( int PWMApin, int AIn2Pin, int AIn1Pin, 
                        int STBYpin, 
                        int BIn1Pin, int BIn2Pin, int PWMBpin, 
                        int offset) {
    _Ain1Pin = AIn1Pin;
    _Ain2Pin = AIn2Pin;
    _pwmAPin = PWMApin;

    _Bin1Pin = BIn1Pin;
    _Bin2Pin = BIn2Pin;
    _pwmBPin = PWMBpin;

    _offset = offset;
    _stbyPin = STBYpin;

    pinMode(_Ain1Pin, OUTPUT);
    pinMode(_Ain2Pin, OUTPUT);
    pinMode(_pwmAPin, OUTPUT);
    pinMode(_stbyPin, OUTPUT);
    pinMode(_Bin1Pin, OUTPUT);
    pinMode(_Bin2Pin, OUTPUT);
    pinMode(_pwmBPin, OUTPUT);
}

int MotorDriver::speedToTicks(float v) {
    return int(v * _cpr / (_pidRate * PI * _wheelDia));
}

void MotorDriver::setSpeedA(int speed) {
    speed = speed * _offset;

    if (speed >= 0) {
        digitalWriteFast(_Ain1Pin, HIGH);
        digitalWriteFast(_Ain2Pin, LOW);
        analogWrite(_pwmAPin, speed);
    } else {
        digitalWriteFast(_Ain1Pin, LOW);
        digitalWriteFast(_Ain2Pin, HIGH);
        analogWrite(_pwmAPin, -speed);
    }
}

void MotorDriver::setSpeedB(int speed) {
    speed = speed * _offset;

    if (speed >= 0) {
        digitalWriteFast(_Bin1Pin, HIGH);
        digitalWriteFast(_Bin2Pin, LOW);
        analogWrite(_pwmBPin, speed);
    } else {
        digitalWriteFast(_Bin1Pin, LOW);
        digitalWriteFast(_Bin2Pin, HIGH);
        analogWrite(_pwmBPin, -speed);
    }
}

void MotorDriver::setSpeed(int speedA, int speedB) {
    digitalWriteFast(_stbyPin, HIGH);

    setSpeedA(speedA);
    setSpeedB(speedB);
}

void MotorDriver::brake() {
    digitalWriteFast(_Ain1Pin, HIGH);
    digitalWriteFast(_Ain2Pin, HIGH);
    analogWrite(_pwmAPin, 0);

    digitalWriteFast(_Bin1Pin, HIGH);
    digitalWriteFast(_Bin2Pin, HIGH);
    analogWrite(_pwmBPin, 0);
}

void MotorDriver::standby() {
    digitalWriteFast(_stbyPin, LOW);
}

void MotorDriver::PID(pidControl *p) {
    long pError, output;

    pError = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);

    output     = (_kp * pError + _kd * (pError - p->PrevErr) + _ki * p->Ierror);
    p->PrevErr = pError;
    p->PrevEnc = p->Encoder;

    output += p->output;

    if (output >= MAXOUTPUT)        output = MAXOUTPUT;
    else if (output <= -MAXOUTPUT)  output = -MAXOUTPUT;
    else                            p->Ierror += pError;

    p->output = output;
}

void MotorDriver::PID_Tunings(float Kp, float Ki, float Kd) {
    _kp = Kp; _ki = Ki; _kd = Kd;
}

void MotorDriver::PID_Clear(pidControl *p) {
    p->Ierror  = 0;
    p->PrevErr = 0;
    p->output  = 0;
}