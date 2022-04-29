#include <EEPROM_Helper.h>
#include <Arduino.h>

double Double::operator=(double val)
{
    value = val;
    return value;
}

void DataBase::writeDoubleAt(int pos, Double db)
{
    for (uint8_t byte : db.data)
    {
        // Serial.println(String("writing pos: ") + pos + " -> " + byte);
        EEPROM.write(pos++, byte);
    }
}

Double DataBase::readDoubleAt(int pos)
{
    Double db;

    for (uint8_t i = 0; i < 8; i++)
    {
        db.data[i] = EEPROM.read(pos++);
        // Serial.println(String("reading pos: ") + pos + " -> " + db.data[i]);
    }

    return db;
}

void DataBase::writeByte(int pos, Byte val)
{
    return EEPROM.write(pos, val);
}

Byte DataBase::readByte(int pos)
{
    return EEPROM.read(pos);
}