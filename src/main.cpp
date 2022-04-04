/*
 * Copyright © 2022 Flight Stabilization System software.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PID_v1.h>

#include <Wire.h>
#include <Servo.h>

Servo servoWing, servoTail;
Adafruit_MPU6050 mpu;
Adafruit_Sensor *acell;

sensors_event_t a;

double
    wingInput,
    wingOutput,
    wingSetPoint,
    wingKp = .3,
    wingKi = .06,
    wingKd = 0,
    tailInput,
    tailOutput,
    tailSetPoint,
    tailKp = .3,
    tailKi = .06,
    tailKd = 0;

PID pidWing(&wingInput, &wingOutput, &wingSetPoint, wingKp, wingKi, wingKd, DIRECT),
    pidTail(&tailInput, &tailOutput, &tailSetPoint, tailKp, tailKi, tailKd, DIRECT);

void setup(void)
{
    Serial.begin(9600);

    while (!Serial)
        ;

    while (!mpu.begin())
    {

#ifdef DEBUG_h
        print("failed to find MPU6050 chip! trying again!");
#endif

        delay(500);
    }

#ifdef DEBUG_h
    print("MPU6050 founded!");
#endif

    acell = mpu.getAccelerometerSensor();

    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true); // Keep it latched.  Will turn off when reinitialized.
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);

    while (!servoWing.attached())
    {
        servoWing.attach(3);
    }

#ifdef DEBUG_h
    print("servoWing attached!");
#endif

    while (!servoTail.attached())
    {
        servoTail.attach(5);
    }

#ifdef DEBUG_h
    print("servoTail attached!");
#endif

    // wing pid config
    wingSetPoint = 0;
    pidWing.SetMode(AUTOMATIC);
    pidWing.SetOutputLimits(-10, 10);
    pidWing.SetSampleTime(10);

    // tail pid config
    tailSetPoint = 0;
    pidTail.SetMode(AUTOMATIC);
    pidTail.SetOutputLimits(-10, 10);
    pidWing.SetSampleTime(10);

}

void loop()
{
    int degreeWing = 0, degreeTail = 0;
    
    acell->getEvent(&a);

    wingInput = a.acceleration.y;
    tailInput = a.acceleration.x;

    if (pidWing.Compute())
    {
        degreeWing = map(wingOutput, -10, 10, 0, 180);
        servoWing.write(degreeWing);
    }

    if (pidTail.Compute())
    {
        degreeTail = map(tailOutput, -10, 10, 0, 180);
        servoTail.write(degreeTail);
    }

#ifdef DEBUG_h
    print("degreeWing: ");
    print(degreeWing);
    print("degreeTail: ");
    print(degreeTail);
#endif
}