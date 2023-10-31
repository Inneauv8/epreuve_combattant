#include "Arduino.h";

#ifndef SIFFLET_H
#define SIFFLET_H

namespace Sifflet {

    volatile bool active = false;

    float valeurFiltre = 0;

    #define TAILLE_MOYENNE 5
    #define DELAY_UPDATE 50

    #define PIN_SIGNAL A2
    #define PIN_AMBIANT A3

    namespace {
        float voltageAmbiant = 0;
        float voltageDetecteur = 0;
        float difference = 0;

        float moyenne[TAILLE_MOYENNE] = {0.0};

        long lastUpdate;

        void init() {
            pinMode(PIN_SIGNAL, INPUT);
            pinMode(PIN_AMBIANT, INPUT);
        }

        void updateValeurFiltre()
        {
            valeurFiltre = 0;
            for (int i = 0; i < TAILLE_MOYENNE; i++)
            {
                valeurFiltre += moyenne[i];
            }
            valeurFiltre /= TAILLE_MOYENNE;
        }

        void update()
        {
            updateValeurFiltre();

            if (millis() - lastUpdate > DELAY_UPDATE) {

                voltageAmbiant = ((float) analogRead(A3)) * 5.0 / 1023.0;
                voltageDetecteur = ((float) analogRead(A2)) * 5.0 / 1023.0;

                difference = voltageDetecteur - voltageAmbiant;
                difference = difference < 0 ? 0 : difference;

                for (int i = 1; i < TAILLE_MOYENNE; i++)
                {
                    moyenne[i - 1] = moyenne[i];
                }

                moyenne[TAILLE_MOYENNE - 1] = difference;

                lastUpdate = millis();
            }
        }

        bool update(float threshold) {
            update();
            return valeurFiltre > threshold;
        }
    }

    void trigger()
    {
        active = !active;
    }

     void printData()
        {
            difference = difference < 0 ? 0 : difference;

            Serial.print("Tension ambiante: ");
            Serial.print(voltageAmbiant);
            Serial.print("V");

            Serial.print("\t");

            Serial.print("Tension detecteur: ");
            Serial.print(voltageDetecteur);
            Serial.print("V");

            Serial.print("\t");

            Serial.print("Difference: ");
            Serial.print(difference);
            Serial.print("V");

            Serial.print("\t");

            Serial.print("valeur filtre: ");
            Serial.print(valeurFiltre);
            Serial.print("V");

            Serial.print("\t");

            Serial.print("Sifflet: ");
            Serial.println(active ? "On" : "Off");
        }
}

#endif