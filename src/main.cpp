/**************************************************************************************************
Nom du fichier : main.cpp
Auteur : Maxime Boucher
Date de création : 2023-10-17

Description : Librairie permettant la mise en application du capteur de ligne
              
Notes : 

Modifications : 

***************************************************************************************************/

// *************************************************************************************************
//  INCLUDES
// *************************************************************************************************	

#include "capteurLigne.h"
#include "PIDLigne.h"
#include "Move.h"
#include <LibRobus.h>
#include <float.h>
#include <mathX.h>


// *************************************************************************************************
//  CONSTANTES
// *************************************************************************************************

#define ARM_SERVO SERVO_1

int movementIndex = 0;

// *************************************************************************************************
//  FONCTIONS LOCALES
// *************************************************************************************************


// *************************************************************************************************
//  STRUCTURES ET UNIONS
// *************************************************************************************************

enum STATE 
{
    Bleu = 0,
    Vert = 1,
    Jaune = 2,
    Rouge = 3,
};

// *************************************************************************************************
// VARIABLES GLOBALES
// *************************************************************************************************

long initialTime = 0;

MOVE::valeursPID rightPID = {};
MOVE::valeursPID leftPID = {};

void setupPID() {
    rightPID.Kp = 0.0625;
    rightPID.Ki = 0.0001;
    rightPID.Kd = 0.001;

    leftPID.Kp = 0.0625;
    leftPID.Ki = 0.0001;
    leftPID.Kd = 0.001;
}

void setupServo() {
    SERVO_Enable(ARM_SERVO);
}

void setup()
{   
    setupPID();
    setupServo();
    PIDLigne::initPID(2.7387791339, 2.625137795, 1.0, 0.0, 0.0, {}, 1);
    BoardInit();
    Serial.begin(9600);

    ENCODER_Reset(RIGHT);
    ENCODER_Reset(LEFT);
    //MOVE::updatePos();

    initialTime = millis();
}

float computeOrientation() {
    float deltaS = (ENCODER_Read(LEFT) - ENCODER_Read(RIGHT)) * MOVE::pulseToDist / 2.0;
    float theta = deltaS * 2 / (WHEEL_BASE_DIAMETER);

    return theta;
}

float computeDistance() {
    float distance = (ENCODER_Read(LEFT) + ENCODER_Read(RIGHT)) * MOVE::pulseToDist / 2.0;
    
    return distance;
}

float radToDeg(float rad) {
    return rad * 180 / M_PI;
}

void rotate(float velocity, float radius) {
    MOVE::WheelVelocities velocities = MOVE::moveByRadius(velocity, radius);

    rightPID.Sp = velocities.rightVelocity;
    leftPID.Sp = velocities.leftVelocity;
}

void move(float velocity, float angularVelocity) {
    float rightWheelVelocity = velocity - (angularVelocity * WHEEL_BASE_DIAMETER) / 2.0;
    float leftWheelVelocity = velocity + (angularVelocity * WHEEL_BASE_DIAMETER) / 2.0;

    rightPID.Sp = rightWheelVelocity;
    leftPID.Sp = leftWheelVelocity;
}

bool rotate(float velocity, float radius, float angle) {
    static float initialAngle = NAN;

    float orientation = computeOrientation();

    if (isnan(initialAngle)) {
        initialAngle = orientation;
    }

    bool angleReached = fabs(orientation - initialAngle) >= fabs(angle);

    rotate(angleReached ? 0 : velocity, radius * (angle > 0 ? 1 : -1));

    if (angleReached) {
        initialAngle = NAN;
    }



    return angleReached;
}

bool forward(float velocity, float distance) {
    static float initialDistance = NAN;

    float actualDistance = computeDistance();

    if (isnan(initialDistance)) {
        initialDistance = actualDistance;
    }

    bool distanceReached = fabs(actualDistance - initialDistance) >= fabs(distance);
    
    rotate(distanceReached ? 0 : velocity, INFINITY);

    if (distanceReached) {
        initialDistance = NAN;
    }

    return distanceReached;
}

void followWall(float id, float velocity, float radius = 13.3, float distance = 425.0) {
    float baseAngularVelocity = velocity / radius;

    float angularVelocity = sigmoid(ROBUS_ReadIR(id), distance, 1, 25.0, -2) * (id == RIGHT ? baseAngularVelocity : -baseAngularVelocity);

    move(velocity, angularVelocity);
}

bool activateServoForDistance(float id, float distance, float targetAngle, float resetAngle) {
    static float initialDistance = NAN;

    float actualDistance = computeDistance();

    if (isnan(initialDistance)) {
        initialDistance = actualDistance;
        SERVO_SetAngle(id, targetAngle);
    }

    bool distanceReached = fabs(actualDistance - initialDistance) >= fabs(distance);

    if (distanceReached) {
        SERVO_SetAngle(id, resetAngle);
        initialDistance = NAN;
    }

    return distanceReached;
}

void updatePIDs() {
    rightPID.Pv = MOVE::computeLeftMotorSpeed();
    MOTOR_SetSpeed(RIGHT, clamp(rightPID.update(), -1, 1));

    leftPID.Pv = MOVE::computeRightMotorSpeed();
    MOTOR_SetSpeed(LEFT, clamp(leftPID.update(), -1, 1));
}



const float velocity = 20;

enum ArmState {
    EXTENDED_RIGHT,
    NOT_EXTENDED,
    EXTENDED_LEFT,
};

ArmState armState = NOT_EXTENDED;

void loopLineFollower()
{
    PIDLigne::computeWheelSpeed(WHEEL_BASE_DIAMETER, 16.0);
}

void loop() {

    delay(5);
    
    /*
    if (movementIndex == 0) { // tournant à droite
        if (rotate(30, 6 + 12 * 0, M_PI / 2.0)) {
            movementIndex++;
        };
    } else if(movementIndex == 1) { //segment tapis
        if (forward(30, 24)) {
            movementIndex++;
        };
    } else if(movementIndex == 2) { // tournant à droite
        if (rotate(30, 18 + 12 * 2, M_PI / 2.0)) {
            movementIndex++;
        };
    } else if (movementIndex == 3) {
        if (forward(30, 96)) {
            movementIndex++;
        };
    }
    */

   move(16, 0);

    if (ROBUS_ReadIR(RIGHT) > 725) {
        armState = EXTENDED_RIGHT;
    }
        
    if (ROBUS_ReadIR(LEFT) > 725) {
        armState = EXTENDED_LEFT;
    }

    switch (armState)
    {
    case EXTENDED_RIGHT:
        if (activateServoForDistance(ARM_SERVO, 24, 0, 90)) {
            armState = NOT_EXTENDED;
        }
        break;
    
    case EXTENDED_LEFT:
        if (activateServoForDistance(ARM_SERVO, 24, 180, 90)) {
            armState = NOT_EXTENDED;
        }
        break;
    }
    

   //followWall(RIGHT, velocity);
   /*
    float angularVelocity = 0;
    float baseAngularVelocity = 1.2;

    angularVelocity = sigmoid(ROBUS_ReadIR(0), 425.0, 1, 25.0, -2) * baseAngularVelocity;


    rotate(velocity, 16 / angularVelocity);
    */

   //ROBUS_ReadIR
    

    //rightPID.Sp = 0;
    //leftPID.Sp = 0;

    //MOVE::WheelVelocities velocities = MOVE::moveByRadius(computeOrientation() < M_PI / 2 ? 16 : 0, 23.622);

    //Serial.println((MOVE::averageSpeedD()+MOVE::averageSpeedD())/2.0);
    //MOVE::updatePIDMain(5, MOVE::radiusToDV(9, -M_PI));
    //PIDLigne::computeWheelSpeed();
    updatePIDs();
}