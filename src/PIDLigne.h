/****************************************************************************************
Nom du fichier : PIDLigne.h
Auteur : Charles-William Lambert                  
Date de création : 24/10/23
  
****************************************************************************************/
#ifndef PIDLigne_H
#define PIDLigne_H

#include "Move.h"
#include "capteurLigne.h"

namespace PIDLigne
{
	
	// *************************************************************************************************
	//  CONSTANTES
	// *************************************************************************************************
    
		  
	// *************************************************************************************************
	//  STRUCTURES ET UNIONS
	// *************************************************************************************************
	struct WheelVelocities
    {
        float leftWheelSpeed;
        float rightWheelSpeed;
    };
		  
	// *************************************************************************************************
	// VARIABLES GLOBALES
	// *************************************************************************************************
	

	// *************************************************************************************************
	//  PROTOTYPE DE FONCTIONS
	// *************************************************************************************************
    void initPID(float sensorDistance, float sensorWidth, float Kp, float Ki, float Kd, const uint8_t pins[], uint8_t pinLedOn);
    float computePID(float trueError);
    WheelVelocities computeWheelSpeed(float wheelBaseDiameter, float robotSpeed);


	namespace {
		// *************************************************************************************************
		// VARIABLES LOCALES
		// *************************************************************************************************
        float computeError(int sensorValue);

		extern float sensorDistance;
        extern float sensorWidth;

		extern struct MOVE::valeursPID anglePID;
	}
}

#endif // PIDLigne_H