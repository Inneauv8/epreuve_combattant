/**************************************************************************************************
Nom du fichier : couleur.cpp
Auteur : Maxime Boucher
Date de création : 2023-10-31

Description : Librairie permettant d'utilisier la librairie du capteur de couleur

Notes :

Modifications :

***************************************************************************************************/

#include <Arduino.h>
#include "couleur.h"

namespace Couleur
{
    namespace
    {
        Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
        uint16_t r, g, b, c;
    }

    char Get(void)
    {
        tcs.getRawData(&r, &g, &b, &c);
        Serial.println(String(r) + '\t' + String(g) + '\t' + String(b));
        // if ( r > 240 && g < 150 && b < 150) return 'r';
        if (r > 200 && g > 200 && b > 100)
            return 'j';
        if (r < 120 && g < 150 && b < 160)
            return 'v';
        // if ( r < 110 && g > 140 && b > 190) return 'b';
        // if ( r > 5000 && g > 5000 && b > 5000) return 'w';
        // if ( r < 250 && g < 250 && b < 250) return 'n';
        return '\0';
    }
}
