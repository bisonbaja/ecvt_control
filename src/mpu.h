#ifndef MPU_H
#define MPU_H

#include "config.h"
#include "utils.h"
#include "MPU6050_6Axis_MotionApps612.h"

// Accelerometer Gyroscope
extern MPU6050 mpu;
extern int const INTERRUPT_PIN;

/*---MPU6050 Control/Status Variables---*/
extern bool DMPReady;
extern uint8_t MPUIntStatus;
extern uint8_t devStatus;
extern uint16_t packetSize;
extern uint8_t FIFOBuffer[64];

/*---Orientation/Motion Variables---*/ 
extern Quaternion q;
extern VectorInt16 aa;
extern VectorInt16 gy;
extern VectorInt16 aaReal;
extern VectorInt16 aaWorld;
extern VectorFloat gravity;
extern float euler[3];
extern float ypr[3];

/*------Interrupt detection routine------*/
extern volatile bool MPUInterrupt;
void DMPDataReady();

void MPUsetup();
void updateMPUWorld();

#endif // MPU_H