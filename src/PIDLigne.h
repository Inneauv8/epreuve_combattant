/****************************************************************************************
Nom du fichier : PIDLigne.h
Auteur : Charles-William Lambert                  
Date de création : 24/10/23
  
****************************************************************************************/
#ifndef PIDLigne_H
#define PIDLigne_H

namespace PIDLigne
{
	
	// *************************************************************************************************
	//  CONSTANTES
	// *************************************************************************************************
    
		  
	// *************************************************************************************************
	//  STRUCTURES ET UNIONS
	// *************************************************************************************************
	struct MotorVelocities
    {
        float leftMotorSpeed;
        float rightMotorSpeed;
    };
		  
	// *************************************************************************************************
	// VARIABLES GLOBALES
	// *************************************************************************************************
	

	// *************************************************************************************************
	//  PROTOTYPE DE FONCTIONS
	// *************************************************************************************************
    void initPID(float sensorDistance, float sensorWidth);
    float computePID(float trueError);
    MotorVelocities computeMotorSpeed(float wheelBaseDiameter, float robotSpeed);


	void main(void);
	int prototypeFonction(int ceciEstLeNomDeLaVariableEnParametre);

	namespace {
		// *************************************************************************************************
		// VARIABLES LOCALES
		// *************************************************************************************************
        float computeError(int sensorValue);

		extern float sensorDistance;
        extern float sensorWidth;
	}
}

#endif // PIDLigne_H