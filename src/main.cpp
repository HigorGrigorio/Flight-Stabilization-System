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
#include <SoftwareSerial.h>

#include <stdlib.h>

#include <debug.h>

#include <EEPROM_Helper.h>

Servo servoWing, servoTail;
Adafruit_MPU6050 mpu;
Adafruit_Sensor *acell;
SoftwareSerial bluetooth(8, 9);

sensors_event_t a;

DataBase database;

double
    wingInput = 0,
    wingOutput = 0,
    wingSetPoint = database.readByte(WING_SET_POINT),
    wingKp = .7,
    wingKi = .00001,
    wingKd = 0,
    tailInput = 0,
    tailOutput = 0,
    tailSetPoint = database.readByte(TAIL_SET_POINT),
    tailKp = .7,
    tailKi = .00001,
    tailKd = 0;

PID pidWing(&wingInput, &wingOutput, &wingSetPoint, wingKp, wingKi, wingKd, P_ON_M, DIRECT),
    pidTail(&tailInput, &tailOutput, &tailSetPoint, tailKp, tailKi, tailKd, P_ON_M, DIRECT);

struct Args
{
    String var;
    String data;
};

enum pid_t : uint8_t
{
    WING = 0,
    TAIL
};

pid_t currentEdition;

Args *explode(String data, char delimiter);
void translate(Args *args);
void save();
void load();
void update();

void setup(void)
{
    Serial.begin(9600);

    while (!Serial)
        ;

    bluetooth.begin(9600);

    while (!bluetooth)
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
        servoWing.attach(4);
    }

#ifdef DEBUG_h
    print("servoWing attached!");
#endif

    while (!servoTail.attached())
    {
        servoTail.attach(7);
    }

#ifdef DEBUG_h
    print("servoTail attached!");
#endif

    load();
    update();

    // wing pid config
    wingSetPoint = 0;
    pidWing.SetMode(AUTOMATIC);
    pidWing.SetOutputLimits(-10, 10);
    pidWing.SetSampleTime(100);

    // tail pid config
    tailSetPoint = 0;
    pidTail.SetMode(AUTOMATIC);
    pidTail.SetOutputLimits(-10, 10);
    pidWing.SetSampleTime(100);
}

void loop()
{

    if (bluetooth.available())
    {
        String data = "";
        uint8_t i = 0;

        // read all parameters before updating the controller
        while (i++ <= 4)
        {
            // receive a complete message
            while (bluetooth.available())
            {
                char byte = bluetooth.read();

                if (byte == ';')
                    break;

                data += byte;

                // await the next character
                while (!bluetooth.available())
                    ;
            }
            
            Args *args = explode(data, '=');
            translate(args);
            data = "";
        }

        save();
        load();
        update();

        return;
    }

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
    // print("degreeWing: ");
    // print(wingInput);
    // print("degreeTail: ");
    // print(tailInput);
#endif
}

/**
 * @brief Translate the string data to args with base in the delimiter.
 * 
 * @param data 
 * @param delimiter 
 * @return Args* 
 */
Args *explode(String data, char delimiter)
{
    Args *args = new Args{"", ""};
    String aux = "";

    for (uint8_t i = 0; i < data.length(); i++)
    {
        if (data[i] == delimiter)
        {
            args->var = aux;
            aux = "";
            i++;
        }

        aux += data[i];
    }

    args->data = aux;

    return args;
}

/**
 * @brief update the local variables.
 * 
 * @param args 
 */
void translate(Args *args)
{
    if (args->var.indexOf("pid") != -1)
    {
        currentEdition = args->data.indexOf("wing") != -1 ? WING : TAIL;
    }
    else
    {
        double val = atof(args->data.c_str());

        if (val == -1)
            return;

        if (args->var.indexOf("kp") != -1)
        {
            if (currentEdition == WING)
                wingKp = val;
            else
                tailKp = val;
        }
        else if (args->var.indexOf("ki") != -1)
        {
            if (currentEdition == WING)
                wingKi = val;
            else
                tailKi = val;
        }
        else if (args->var.indexOf("kd") != -1)
        {
            if (currentEdition == WING)
                wingKd = val;
            else
                tailKd = val;
        }
        else if (args->var.indexOf("setpoint") != -1)
        {
            if (currentEdition == WING)
                wingSetPoint = val;
            else
                tailSetPoint = val;
        }
    }
}

/**
 * @brief Save all mutable variables in the EEPROM memory.
 * 
 */
void save()
{
    database.writeDoubleAt(WING_KP, wingKp);
    database.writeDoubleAt(WING_KI, wingKi);
    database.writeDoubleAt(WING_KD, wingKd);

    database.writeDoubleAt(TAIL_KP, tailKp);
    database.writeDoubleAt(TAIL_KI, tailKi);
    database.writeDoubleAt(TAIL_KD, tailKd);

    database.writeByte(WING_SET_POINT, wingSetPoint);
    database.writeByte(TAIL_SET_POINT, tailSetPoint);
}

/**
 * @brief Load all mutable variables from EEPROM memory.
 * 
 */
void load()
{
    wingKp = database.readDoubleAt(WING_KP);
    wingKi = database.readDoubleAt(WING_KI);
    wingKd = database.readDoubleAt(WING_KD);
    wingSetPoint = database.readByte(WING_SET_POINT);

    tailKp = database.readDoubleAt(TAIL_KP);
    tailKi = database.readDoubleAt(TAIL_KI);
    tailKd = database.readDoubleAt(TAIL_KD);
    tailSetPoint = database.readByte(TAIL_SET_POINT);

#ifdef DEBUG_h
    var(tailKp);
    var(tailKi);
    var(tailKd);
    var(tailSetPoint);
    var(wingKp);
    var(wingKi);
    var(wingKd);
    var(wingSetPoint);
    nl();
#endif

}

/**
 * @brief Update the PID variables of tail and wings.
 * 
 */
void update()
{
    pidWing.SetTunings(wingKp, tailKp, tailKd);
    pidTail.SetTunings(tailKp, tailKi, tailKd);
}