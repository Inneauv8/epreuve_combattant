/**************************************************************************************************
Nom du fichier : capteurLigne.cpp
Auteur : Maxime Boucher
Date de création : 2023-10-17

Description : Librairie permettant d'utilisier la librairie du capteur de ligne pour le projet du combattant
              
Notes : 

Modifications : 

***************************************************************************************************/
// test pour commit
#include "capteurLigne.h"
#include <Arduino.h>

namespace CapteurLigne{

  namespace{
    QTRSensors qtr;
    uint16_t sensorValues[8];
  }
  /**
   * @brief Initialise le detecteur de ligne
   * @param pins Tableau des numéros des pin digitale de l'arduino utilisées par le capteur N de la ligne
   * @param pinLedOn Pin digitale qui définit si les diodes émetent ou non
   */
  int initLine(const uint8_t pins[], uint8_t pinLedOn){
      qtr.setTypeRC();
      qtr.setSensorPins(pins,8);
      qtr.setEmitterPin(pinLedOn);
      qtr.calibrate();

      qtr.calibrationOn.minimum[0] = 196;
      qtr.calibrationOn.minimum[1] = 248;
      qtr.calibrationOn.minimum[2] = 196;
      qtr.calibrationOn.minimum[3] = 344;
      qtr.calibrationOn.minimum[4] = 196;
      qtr.calibrationOn.minimum[5] = 204;
      qtr.calibrationOn.minimum[6] = 396;
      qtr.calibrationOn.minimum[7] = 396;  

      qtr.calibrationOn.maximum[0] = 2500;
      qtr.calibrationOn.maximum[1] = 2500;
      qtr.calibrationOn.maximum[2] = 2500;
      qtr.calibrationOn.maximum[3] = 2500;
      qtr.calibrationOn.maximum[4] = 2500;
      qtr.calibrationOn.maximum[5] = 2500;
      qtr.calibrationOn.maximum[6] = 2500;
      qtr.calibrationOn.maximum[7] = 2500; 

      qtr.calibrationOff.minimum[0] = 163;
      qtr.calibrationOff.minimum[1] = 360;
      qtr.calibrationOff.minimum[2] = 45440;
      qtr.calibrationOff.minimum[3] = 335;
      qtr.calibrationOff.minimum[4] = 4353;
      qtr.calibrationOff.minimum[5] = 773;
      qtr.calibrationOff.minimum[6] = 0;
      qtr.calibrationOff.minimum[7] = 14; 

      qtr.calibrationOff.maximum[0] = 163;
      qtr.calibrationOff.maximum[1] = 360;
      qtr.calibrationOff.maximum[2] = 45440;
      qtr.calibrationOff.maximum[3] = 335;
      qtr.calibrationOff.maximum[4] = 4353;
      qtr.calibrationOff.maximum[5] = 773;
      qtr.calibrationOff.maximum[6] = 0;
      qtr.calibrationOff.maximum[7] = 16; 

      qtr.calibrationOn.initialized = true;
      qtr.calibrationOff.initialized = true;

      qtr.calibrate();

      return 1;
  }

  /**
   * @brief Lit la valeur du capteur de ligne
   */
  int readLineValue(void) 
  {
    /*
    const uint8_t LINE_FOLLOWER_PINS[] = {2, 3, 14, 15, 8, 9, 10, 11};
    for (int i = 0; i < 8; i++) {
      if (i != 0) {
        Serial.print("\t");
      }
      Serial.print(digitalRead(LINE_FOLLOWER_PINS[i]));
    }

    Serial.println();
    */

    return qtr.readLineBlack(sensorValues);
  }

  bool isBlackLine(void)
  {
    static uint16_t mean = 0;
    uint16_t sensorValues[8];
    uint16_t total = 0;

    qtr.readCalibrated(sensorValues);

    for(int i = 0; i < 8; i++)
    {
      total += sensorValues[i];
    }
    
    mean = mean * 0.9 + total * 0.1;
    Serial.println(mean);
    return (mean > 6800);
  }
}
