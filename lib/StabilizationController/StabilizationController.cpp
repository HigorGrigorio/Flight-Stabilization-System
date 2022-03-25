#include <Arduino.h>
#include <StabilizationController.h>

void StabilizationController::setMPU6050(Adafruit_MPU6050 *pmpu)
{

    this->mpu = pmpu;

    if (pmpu)
    {
        setGyroSensor(mpu->getGyroSensor());
    }
}

Adafruit_MPU6050 *StabilizationController::getMPU6050()
{
    return this->mpu;
}

void StabilizationController::setServo(Servo *servo)
{
    this->servo = servo;
}

Servo *StabilizationController::getServo()
{
    return this->servo;
}

void StabilizationController::setGyroSensor(Adafruit_Sensor *gyroSensor)
{
    this->gyro = gyroSensor;
}

Adafruit_Sensor *StabilizationController::getGyroSensor()
{
    return this->gyro;
}

bool StabilizationController::readRawGyro()
{
    if (this->gyro)
    {
        gyro->getEvent(&rg);
        return true;
    }

    return false;
}

void StabilizationController::calibrateSensor(uint8_t samples)
{
    useCalibrate = true;

    // Reset values
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    float sigmaX = 0;
    float sigmaY = 0;
    float sigmaZ = 0;

    // Read n-samples
    for (uint8_t i = 0; i < samples; ++i)
    {
        readRawGyro();

        sumX += rg.gyro.x;
        sumY += rg.gyro.y;
        sumZ += rg.gyro.z;

        sigmaX += rg.gyro.x * rg.gyro.x;
        sigmaY += rg.gyro.y * rg.gyro.y;
        sigmaZ += rg.gyro.z * rg.gyro.z;

        delay(5);
    }

    // Calculate delta vectors
    dg.gyro.x = sumX / samples;
    dg.gyro.y = sumY / samples;
    dg.gyro.z = sumZ / samples;

    // Calculate threshold vectors
    th.gyro.x = sqrt((sigmaX / 50) - (dg.gyro.x * dg.gyro.x));
    th.gyro.y = sqrt((sigmaY / 50) - (dg.gyro.y * dg.gyro.y));
    th.gyro.z = sqrt((sigmaZ / 50) - (dg.gyro.z * dg.gyro.z));
}

void StabilizationController::readNormalizedGyro()
{
    // readRawGyro();

    // if (useCalibrate)
    // {
    //     ng.gyro.x = (rg.gyro.x - dg.gyro.x) * dpsPerDigit;
    //     ng.gyro.y = (rg.gyro.y - dg.gyro.y) * dpsPerDigit;
    //     ng.gyro.z = (rg.gyro.z - dg.gyro.z) * dpsPerDigit;
    // }
    // else
    // {
    //     ng.gyro.x = rg.gyro.x * dpsPerDigit;
    //     ng.gyro.y = rg.gyro.y * dpsPerDigit;
    //     ng.gyro.z = rg.gyro.z * dpsPerDigit;
    // }

    // return ng;
}