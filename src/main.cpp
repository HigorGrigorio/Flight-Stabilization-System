#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <Wire.h>
#include <Servo.h>

Servo servo;
Adafruit_MPU6050 mpu;
Adafruit_Sensor *acell;

void setup(void)
{
    Serial.begin(9600);

    while (!Serial)
        ;

    while (!mpu.begin())
    {
        Serial.println("failed to find MPU6050 chip! trying again!");
        delay(500);
    }

    Serial.println("MPU6050 founded!");

    acell = mpu.getAccelerometerSensor();

    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true); // Keep it latched.  Will turn off when reinitialized.
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);

    while (!servo.attached())
    {
        servo.attach(3);
    }

    Serial.println("servo attached!");

    servo.write(90);

    delay(2000);
}

void loop()
{
    sensors_event_t a;
    int value = 0;

    for (int i = 0; i < 10; i++)
    {
        acell->getEvent(&a);
        value += map(a.acceleration.y, -10, 10, 180, 0);
    }

    value /= 10;
    Serial.println(value);

    servo.write(value * .7);
}