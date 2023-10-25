/**************************************************************************************************
Nom du fichier : PIDLigne.cpp
Auteur : Charles-William Lambert
Date de création : 2023-10-24

Description : Librairie PID suiveur de ligne
              
Notes : 

Modifications : - Creation - Charles-William Lambert - 2023-10-25
***************************************************************************************************/

// *************************************************************************************************
//  INCLUDES
// *************************************************************************************************	
#include "PIDLigne.h"
#include <Arduino.h>

/**
 * @brief Computes left and right motor speeds according to line sensor
 * @author Charles-William Lambert
*/
namespace PIDLigne {
	// *************************************************************************************************
	//  CONSTANTES
	// *************************************************************************************************
	
	
	// *************************************************************************************************
	//  STRUCTURES ET UNIONS
	// *************************************************************************************************
	
	
	// *************************************************************************************************
	// VARIABLES GLOBALES
	// *************************************************************************************************
	

	// *************************************************************************************************
	// FONCTIONS GLOBALES
	// *************************************************************************************************
    
    /**
     * @brief Initialise les paramètres du PID et du capteur de ligne.
     * 
     * @param _sensorDistance Distance entre le lapteur de ligne et le centre de rotation du robot.
     * @param _sensorWidth Largeur du capteur de ligne.
     * @param Kp Constante proportionnelle du PID.
     * @param Ki Constante intégrale du PID.
     * @param Kd Constante dérivée du PID.
     * @param pins Tableau des numéros des pin digitale de l'arduino utilisées par le capteur N de la ligne.
     * @param pinLedOn Pin digitale qui définit si les diodes émetent ou non.
     */
    void initPID(float _sensorDistance, float _sensorWidth, float Kp, float Ki, float Kd, const uint8_t pins[], uint8_t pinLedOn)
    {
        sensorDistance = _sensorDistance;
        sensorWidth = _sensorWidth;
        anglePID.Kp = Kp;
        anglePID.Ki = Ki;
        anglePID.Kd = Kd;
        anglePID.Sp = 0.0;

        CapteurLigne::initLine(pins, pinLedOn);
    }

    /**
     * @brief Calcule la vitesse angulaire cible pour la correction.
     * 
     * @param trueError Angle de déviation par rapport à la ligne suivie.
     * @return float La vitesse angulaire pour la correction des moteurs.
     */
    float computePID(float trueError)
    {
        anglePID.Pv = trueError;
        MOVE::calculPID(&anglePID);

        return anglePID.Out;
    }

    /**
     * @brief Calcule la vitesse de chaque roue pour corriger la trajectoire.
     * 
     * @param wheelBaseDiameter La distance entre les deux roues.
     * @param robotSpeed La vitesse cible du robot.
     * @return WheelVelocities La vitesse cible au sol de chaque roue.
     */
    WheelVelocities computeWheelSpeed(float wheelBaseDiameter, float robotSpeed)
    {
        float lineValue = CapteurLigne::readLineValue();
        float trueError = computeError(lineValue);
        float angularVelocity = computePID(trueError);
        float leftWheelSpeed = robotSpeed + (angularVelocity * wheelBaseDiameter) / 2.0;
        float rightWheelSpeed = robotSpeed - (angularVelocity * wheelBaseDiameter) / 2.0;

        return {leftWheelSpeed, rightWheelSpeed};
    }

	namespace {
		// *************************************************************************************************
		// VARIABLES LOCALES
		// *************************************************************************************************
		

		// *************************************************************************************************
		//  FONCTIONS LOCALES
		// *************************************************************************************************

        /**
         * @brief Convertie la valeur du capteur de ligne en une erreur angulaire.
         * 
         * @param sensorValue La valeur du capteur de ligne.
         * @return float L'erreur angulaire équivalente.
         */
        float computeError(int sensorValue)
        {
            float normalizedSensorValue = sensorValue / 3500.0 - 1.0;
            float sensorPhysicalPosition = normalizedSensorValue * (sensorWidth / 2);
            float trueError = atan2(sensorPhysicalPosition, sensorDistance);
            return trueError;
        }

		float sensorDistance = 0;
        float sensorWidth = 0;

        struct MOVE::valeursPID anglePID;
		
	}

}