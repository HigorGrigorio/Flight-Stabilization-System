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

/**
 * @file EEPROM_Helper.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-25
 * 
 */

#include <EEPROM.h>
#include <inttypes.h>

/**
 * @brief the position of variables on the EEPROM
 *
 */
#define WING_SET_POINT 0
#define TAIL_SET_POINT 1

#define WING_KP 2
#define WING_KI 10
#define WING_KD 18

#define TAIL_KP 26
#define TAIL_KI 34
#define TAIL_KD 42

typedef unsigned char Byte;

/**
 * @brief Helper from convert a double value in byte vector.
 *
 */
union Double
{
    /**
     * @brief Constructor of double object.
     * 
     * @param val double value.
     */

    Double(double val) : value(val) {}

    // Default constructor
    Double() = default;

    /**
     * @brief Assignment operator.
     *
     * @param val double value.
     * @return double
     */
    auto operator=(double) -> double;

    /**
     * @brief Explicit convert to double.
     * 
     * @return double 
     */
    auto operator double() -> double { return value; }

    /**
     * @brief storage double.
     * 
     */
    double value;

    /**
     * @brief storage double in vector of 8 byte's.
     * 
     */
    Byte data[8];
};

/**
 * @brief The EEPROM_Helper class has the objective of auxiliar the save of data and load of data on EEPROM memory.
 * 
 */
class DataBase
{
public:
    /**
     * @brief Default constructor.
     * 
     */
    DataBase() = default;

    /**
     * @brief Destructor.
     * 
     */
    ~DataBase() {}

    /**
     * @brief Write a double value into position.
     * 
     */
    void writeDoubleAt(int, Double);

    /**
     * @brief Read a double value from position.
     * 
     * @return Double 
     */
    auto readDoubleAt(int) -> Double;

    /**
     * @brief Write 8 bit's value into position.
     * 
     */
    void writeByte(int, Byte);

    /**
     * @brief Read 8 bit's value from position.
     * 
     * @return Byte 
     */
    auto readByte(int) -> Byte;
};
