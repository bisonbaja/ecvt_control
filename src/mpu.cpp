#include "mpu.h"
#include "config.h"
#include <Arduino.h>
#include "BluetoothSerial.h"
#include "MPU6050_6Axis_MotionApps612.h"

extern BluetoothSerial SerialBT;
extern MPU6050 mpu;

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
        SerialBT.println("MPU6050 Connection Failed!");
        fail();
    } else {
        SerialBT.println("MPU6050 Connection Successful!");
    }

    SerialBT.println("Initializing DMP...");
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
        SerialBT.println("These are the Active offsets: ");
        mpu.PrintActiveOffsets();
        SerialBT.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        SerialBT.print(F("Enabling interrupt detection (Arduino external interrupt "));
        SerialBT.print(digitalPinToInterrupt(INTERRUPT_PIN));
        SerialBT.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
        MPUIntStatus = mpu.getIntStatus();

        SerialBT.println(F("DMP ready! Waiting for first interrupt..."));
        DMPReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void updateMPUWorld() {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
}