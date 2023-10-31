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
    namespace {
        Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
        uint16_t r,g,b,c;
    }  

    char Get(void){
        tcs.getRawData(&r, &g, &b, &c);
        if ( r > 240 && g < 150 && b < 150) return 'r';
        if ( r > 450 && g > 350 && b < 300) return 'j';
        if ( r < 150 && g > 150 && b < 180) return 'v';
        if ( r < 110 && g > 140 && b > 190) return 'b';
        if ( r > 500 && g > 500 && b > 500) return 'w';
        return '\0';
    }
}
