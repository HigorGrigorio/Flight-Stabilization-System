/**
 * @file StabilizationController.h
 * @author Higor Grigorio (higorgrigorio@gmail.com)
 * @brief A stabilization controller to MPU6050 sensor monitoring
 * @version 0.1
 * @date 2022-03-18
 *
 * @copyright Copyright (c) 2022 Higor Grigorio
 *
 */

#ifndef STABILIZATION_CONTROLLER_h
#define STABILIZATION_CONTROLLER_h

#include <Adafruit_MPU6050.h>
#include <Adafruit_MPU6050.h>

#include <Wire.h>
#include <Servo.h>

class StabilizationController
{
public:
    // mpu6050 set/get
    void setMPU6050(Adafruit_MPU6050 *);
    Adafruit_MPU6050 *getMPU6050();

    // servo set/get
    void setServo(Servo *);
    Servo *getServo();

    // gyro set/get
    void setGyroSensor(Adafruit_Sensor *);
    Adafruit_Sensor *getGyroSensor();

    bool readRawGyro();
    void readNormalizedGyro();
    void calibrateSensor(uint8_t);

    //  static float getDpsPerSecond(Adafruit_MPU6050*);

private:
    // mpu6050 pointer
    Adafruit_MPU6050 *mpu;
    Adafruit_Sensor *gyro;

    // servo pointer
    Servo *servo;

    bool useCalibrate;

    sensors_event_t rg, dg, th, ng;
};

#endif // !STABILIZATION_CONTROLLER_h