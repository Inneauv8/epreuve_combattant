#include <QTRSensors.h>
#include <Arduino.h>

namespace CapteurLigne 
{
    int initLine(const uint8_t pins[], uint8_t pinLedOn);

    int readLineValue(void);

    namespace {
        extern QTRSensors qtr;
        extern uint16_t sensorValues[8];
    }       
}
