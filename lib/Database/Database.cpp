#include <Database.h>
#include <Arduino.h>

auto Double::operator=(double val) -> double
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

auto DataBase::readDoubleAt(int pos) -> Double
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

auto DataBase::readByte(int pos) -> Byte
{
    return EEPROM.read(pos);
}