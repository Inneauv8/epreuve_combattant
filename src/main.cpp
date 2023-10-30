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
#include <Adafruit_TCS34725.h>


// *************************************************************************************************
//  CONSTANTES
// *************************************************************************************************

#define CLAW_SERVO SERVO_1
#define ARM_SERVO SERVO_2
const uint8_t LINE_FOLLOWER_PINS[] = {38, 52, 51, 50, 49, 48, 47, 46};

int movementIndex = -1;

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

bool distanceFlag(float distance, float *initialDistance) {
    float actualDistance = computeDistance();

    if (isnan(*initialDistance)) {
        *initialDistance = actualDistance;
    }

    bool distanceReached = fabs(actualDistance - *initialDistance) >= fabs(distance);
    
    if (distanceReached) {
        *initialDistance = NAN;
    }

    return distanceReached;
}

bool orientationFlag(float angle, float *initialOrientation) {
    float actualOrientation = computeOrientation();

    if (isnan(*initialOrientation)) {
        *initialOrientation = actualOrientation;
    }

    bool angleReached = fabs(actualOrientation - *initialOrientation) >= fabs(angle);
    
    if (angleReached) {
        *initialOrientation = NAN;
    }

    return angleReached;
}

bool distanceFlag(float distance) {
    static float initialDistance = NAN;
    return distanceFlag(distance, &initialDistance);
}

bool orientationFlag(float angle) {
    static float initialOrientation = NAN;
    return orientationFlag(angle, &initialOrientation);
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

void moveUnited(float velocity, float radius, float orientation) {

    float baseAngularVelocity = isinf(radius) ? 0 : (velocity / radius);

    //10 / 33 = 0.303

    //Serial.println(radius);

    float targetAngle = smallestAngleDifference(computeOrientation(), orientation);
    float angularVelocity = sigmoid(targetAngle, 0, 1, 1, -2) * -baseAngularVelocity;

    move(velocity, angularVelocity);
}

bool rotate(float velocity, float radius, float angle) {
    static float initialOrientation = NAN;
    float actualOrientation = computeOrientation();

    if (isnan(initialOrientation)) {
        initialOrientation = actualOrientation;
    }

    float targetOrientation = wrap((initialOrientation + fabs(angle) * (radius > 0 ? 1 : -1)), -M_PI, M_PI);

    bool angleReached = fabs(actualOrientation - initialOrientation) >= fabs(angle);

    if (angleReached) {
        move(0, 0);
    } else {
        moveUnited(velocity, fabs(radius), targetOrientation);
    }
    
    if (angleReached) {
        initialOrientation = NAN;
    }

    return false;
}

bool forward(float velocity, float distance) {
    static float initialDistance = NAN;
    static float initialOrientation = NAN;

    //float actualDistance = computeDistance();
    //float actualOrientation = computeOrientation();

    if (isnan(initialDistance)) {
        initialOrientation = computeOrientation();
    }

    bool distanceReached = distanceFlag(distance, &initialDistance);

    if (distanceReached) {
        move(0, 0);
    } else {
        moveUnited(velocity, velocity / 5.0, initialOrientation);
    }

    //float correction = sigmoid(actualOrientation, initialOrientation, 1, 0.1, -2) * 5;
    
    //move(distanceReached ? 0 : velocity, 0);

    if (isnan(initialDistance)) {
       initialOrientation = NAN;
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

    bool distanceReached = distanceFlag(distance, &initialDistance);
    SERVO_SetAngle(id, distanceReached ? resetAngle : targetAngle);

    return distanceReached;
}

void setClaw(ClawState state) {
    clawState = state;
}

void updatePIDs() {
    rightPID.Pv = MOVE::computeLeftMotorSpeed();
    MOTOR_SetSpeed(RIGHT, clamp(rightPID.update(), -1, 1));

    leftPID.Pv = MOVE::computeRightMotorSpeed();
    MOTOR_SetSpeed(LEFT, clamp(leftPID.update(), -1, 1));
}

void loopLineFollower()
{
    PIDLigne::WheelVelocities wheelVelocities = PIDLigne::computeWheelSpeed(WHEEL_BASE_DIAMETER, 5.0);
    rightPID.Sp = wheelVelocities.rightWheelSpeed;
    leftPID.Sp = wheelVelocities.leftWheelSpeed;
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
        if (rotate(10, 18 + 12 + 3, M_PI / 2.0)) {
            movementIndex++;
        }
    } else if(movementIndex == 1) { //segment tapis
        if (forward(10, 24)) {
            movementIndex++;
        }
    } else if(movementIndex == 2) { // tournant à droite
        if (rotate(10, 18 + 12 + 3, M_PI / 2.0)) {
            movementIndex++;
        }
    } else if (movementIndex == 3) {
        if (forward(10, 96)) {
            movementIndex = 0;
        }
    }
    

    if (ROBUS_ReadIR(RIGHT) > 500) {
        armState = EXTENDED_RIGHT;
    }
        
        
    if (ROBUS_ReadIR(LEFT) > 500) {
        armState = EXTENDED_LEFT;
    }

    //loopLineFollower();

    //move(30, 0);

    //Serial.println(armState);

    //SERVO_SetAngle(ARM_SERVO, 0);

    
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