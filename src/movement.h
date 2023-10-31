/****************************************************************************************
Nom du fichier : movement.h
Auteur :                   
Date de création : 
  
****************************************************************************************/
#ifndef PROTOTYPE_H
#define PROTOTYPE_H

#include <LibRobus.h>
#include <Arduino.h>
#include "mathX.h"
#include "pid.h"
#include "float.h"

#define WHEEL_BASE_DIAMETER 7.5 //7.480315
#define WHEEL_DIAMETER 2.992126
#define INACTIVE NAN

namespace Movement
{

    extern float pulseToDist;

    struct WheelVelocities {
        float rightVelocity;
        float leftVelocity;
    };
	
    float computeOrientation();
    float computeDistance();

    bool distanceFlag(float distance, float *initialDistance);
    bool orientationFlag(float angle, float *initialOrientation);
    bool distanceFlag(float distance);
    bool orientationFlag(float angle);
    bool isInactive(float var);

    void rotate(float velocity, float radius);

    void move(float velocity, float angularVelocity);

    void moveUnited(float velocity, float radius, float orientation);

    bool rotate(float velocity, float radius, float angle);

    bool forward(float velocity, float distance);

    float computeLeftMotorSpeed();
    float computeRightMotorSpeed();

    void setPIDRight(float Kp, float Ki, float Kd);
    void setPIDLeft(float Kp, float Ki, float Kd);

    void setRightSpeed(float speed);
    void setLeftSpeed(float speed);

    void updatePIDs();


	namespace {
		// *************************************************************************************************
		// VARIABLES LOCALES
		// *************************************************************************************************
		/* VIDE */

        extern PID::valeursPID rightPID;
        extern PID::valeursPID leftPID;
	}
}

#endif // PROTOTYPE_H