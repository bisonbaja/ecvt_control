#include "mpu.h"
#include "config.h"

extern MPU6050 mpu;

// Accelerometer Gyroscope
MPU6050 mpu;
int const INTERRUPT_PIN = 2;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;
uint8_t MPUIntStatus = 0;
uint8_t devStatus = 0;
uint16_t packetSize = 0;
uint8_t FIFOBuffer[64];

/*---Orientation/Motion Variables---*/ 
Quaternion q;
VectorInt16 aa;
VectorInt16 gy;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

volatile bool MPUInterrupt = false;

void DMPDataReady() {
    MPUInterrupt = true;
}

void MPUsetup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    if (!mpu.testConnection()) {
        Serial_println("MPU6050 Connection Failed!");
        fail();
    } else {
        Serial_println("MPU6050 Connection Successful!");
    }

    Serial_println("Initializing DMP...");
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial_println("These are the Active offsets: ");
        mpu.PrintActiveOffsets();
        Serial_println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial_print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial_print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial_println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
        MPUIntStatus = mpu.getIntStatus();

        Serial_println(F("DMP ready! Waiting for first interrupt..."));
        DMPReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial_print(F("DMP Initialization failed (code "));
        Serial_print(devStatus);
        Serial_println(F(")"));
    }
}

void updateMPUWorld() {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
}