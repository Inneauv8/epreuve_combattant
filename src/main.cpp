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
#include <LibRobus.h>
#include <float.h>
#include <mathX.h>
#include <Adafruit_TCS34725.h>
#include <movement.h>

using namespace Movement;


// *************************************************************************************************
//  CONSTANTES
// *************************************************************************************************

#define CLAW_SERVO SERVO_1
#define ARM_SERVO SERVO_2
const uint8_t LINE_FOLLOWER_PINS[] = {38, 52, 51, 43, 49, 48, 47, 46};

int movementIndex = -1;

// *************************************************************************************************
//  FONCTIONS LOCALES
// *************************************************************************************************


// *************************************************************************************************
//  STRUCTURES ET UNIONS
// *************************************************************************************************

enum ClawState {
    OPENED,
    CLOSED,
};

enum ArmState {
    EXTENDED_RIGHT,
    NOT_EXTENDED,
    EXTENDED_LEFT,
};

// *************************************************************************************************
// VARIABLES GLOBALES
// *************************************************************************************************

ArmState armState = NOT_EXTENDED;
ClawState clawState = OPENED;

void setupPID() {
    setPIDRight(0.0625, 0.0001, 0.001);
    setPIDLeft(0.0625, 0.0001, 0.001);
}

void setupServo() {
    SERVO_Enable(ARM_SERVO);
    SERVO_SetAngle(ARM_SERVO, 90);
    SERVO_Enable(CLAW_SERVO);
}

void setup()
{   
    setupPID();
    setupServo();
    BoardInit();
    //Good value : 8, 0.001, 0.15
    //PIDLigne::initPID(2.7387791339, 2.625137795, 6, 0, 0.1, LINE_FOLLOWER_PINS, 45);
    Serial.begin(9600);

    ENCODER_Reset(RIGHT);
    ENCODER_Reset(LEFT);
}

bool activateServoForDistance(float id, float distance, float targetAngle, float resetAngle) {
    static float initialDistance = INACTIVE;

    if (Movement::isInactive(initialDistance)) {
        SERVO_SetAngle(id, targetAngle);
    }

    bool distanceReached = Movement::distanceFlag(distance, &initialDistance);
    
    SERVO_SetAngle(id, distanceReached ? resetAngle : targetAngle);

    return distanceReached;
}

void setClaw(ClawState state) {
    clawState = state;
}

void setArm(ArmState state) {
    armState = state;
}

void loopLineFollower()
{
    PIDLigne::WheelVelocities wheelVelocities = PIDLigne::computeWheelSpeed(WHEEL_BASE_DIAMETER, 5.0);
    Movement::rightPID.Sp = wheelVelocities.rightWheelSpeed;
    Movement::leftPID.Sp = wheelVelocities.leftWheelSpeed;
}

void followWall(float id, float velocity, float radius = 13.3, float distance = 425.0) {
    float baseAngularVelocity = velocity / radius;

    float angularVelocity = sigmoid(ROBUS_ReadIR(id), distance, 1, 25.0, -2) * (id == RIGHT ? baseAngularVelocity : -baseAngularVelocity);

    Movement::move(velocity, angularVelocity);
}

void updateServos() {
    switch (armState)
    {
    case EXTENDED_RIGHT:
        if (activateServoForDistance(ARM_SERVO, 85, 180, 40)) {
           armState = NOT_EXTENDED;
        }
        break;
    
    case EXTENDED_LEFT:
        if (activateServoForDistance(ARM_SERVO, 85, 0, 40)) {
            armState = NOT_EXTENDED;
        }
        break;
    }
    
    SERVO_SetAngle(CLAW_SERVO, clawState == OPENED ? 75 : 110);
}

void loop() {

    delay(5);

    int closedTime = 19000;
    int openedTime = 1000;

    if (millis() % (closedTime + openedTime) < closedTime) {
        setClaw(CLOSED);
    } else {
        setClaw(OPENED);
    }
    
    
    if (movementIndex == -1) { // tournant à droite
        if (forward(10, 96 / 2.0)) {
            movementIndex++;
        }
    } else if (movementIndex == 0) { // tournant à droite
        if (rotate(20, 18 + 12, M_PI / 2.0)) {
            movementIndex++;
        }
    } else if(movementIndex == 1) { //segment tapis
        if (forward(20, 24)) {
            movementIndex++;
        }
    } else if(movementIndex == 2) { // tournant à droite
        if (rotate(20, 18 + 12, M_PI / 2.0)) {
            movementIndex++;
        }
    } else if (movementIndex == 3) {
        if (forward(20, 96)) {
            movementIndex = 0;
        }
    }
    

    if (ROBUS_ReadIR(RIGHT) > 500) {
        setArm(EXTENDED_RIGHT);
    }
        
        
    if (ROBUS_ReadIR(LEFT) > 500) {
        setArm(EXTENDED_LEFT);
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
    updateServos();
    Movement::updatePIDs();
}