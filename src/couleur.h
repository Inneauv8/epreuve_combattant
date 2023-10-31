#include <Arduino.h>
#include <Adafruit_TCS34725.h>

namespace Couleur
{
    char Get(void);

    namespace {
        extern Adafruit_TCS34725 tcs;
        extern uint16_t r,g,b,c;
    }       
}
