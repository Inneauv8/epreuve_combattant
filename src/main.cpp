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
#include <movement.h>
#include <couleur.h>
#include <sifflet.h>

using namespace Movement;


// *************************************************************************************************
//  CONSTANTES
// *************************************************************************************************

#define CLAW_SERVO SERVO_1
#define ARM_SERVO SERVO_2
const uint8_t LINE_FOLLOWER_PINS[] = {38, 52, 51, 42, 49, 48, 47, 46};

int movementIndex = -1;

// *************************************************************************************************
//  FONCTIONS LOCALES
// *************************************************************************************************

// *************************************************************************************************
//  STRUCTURES ET UNIONS
// *************************************************************************************************

enum ClawState
{
    OPENED,
    CLOSED,
};

enum ArmState
{
    EXTENDED_RIGHT,
    NOT_EXTENDED,
    EXTENDED_LEFT,
};

// *************************************************************************************************
// VARIABLES GLOBALES
// *************************************************************************************************

ArmState armState = NOT_EXTENDED;
ClawState clawState = OPENED;
byte state = 88; //Remettre à zéro pour le sifflet 

void setupPID() {
    setPIDRight(0.0625, 0.0001, 0.001);
    setPIDLeft(0.0625, 0.0001, 0.001);
}

void setupServo()
{
    SERVO_Enable(ARM_SERVO);
    SERVO_SetAngle(ARM_SERVO, 90);
    SERVO_Enable(CLAW_SERVO);
}

void setup()
{
    setupPID();
    setupServo();
    BoardInit();
    // Good value : 8, 0.001, 0.15
    //PIDLigne::initPID(2.7387791339, 2.625137795, 8, 0, 0.1, LINE_FOLLOWER_PINS, 45);
    Serial.begin(9600);


    CapteurLigne::initLine(LINE_FOLLOWER_PINS, 45);
    ENCODER_Reset(RIGHT);
    ENCODER_Reset(LEFT);
    //Sifflet::init();

    delay(1000);
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

void setClaw(ClawState state)
{
    clawState = state;
}

void setArm(ArmState state) {
    armState = state;
}

void loopLineFollower()
{
    PIDLigne::WheelVelocities wheelVelocities = PIDLigne::computeWheelSpeed(WHEEL_BASE_DIAMETER, 5.0);
    setRightSpeed(wheelVelocities.rightWheelSpeed);
    setLeftSpeed(wheelVelocities.leftWheelSpeed);
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

void updateEverything()
{
    switch (armState)
    {
    case EXTENDED_RIGHT:
        if (activateServoForDistance(ARM_SERVO, 85, 180, 40))
        {
            armState = NOT_EXTENDED;
        }
        break;

    case EXTENDED_LEFT:
        if (activateServoForDistance(ARM_SERVO, 85, 0, 40))
        {
            armState = NOT_EXTENDED;
        }
        break;
    }

    //SERVO_SetAngle(CLAW_SERVO, clawState == OPENED ? 75 : 110);
    updateServos();
    updatePIDs();
}

void temploop()
{
    
    delay(5);

    
    int closedTime = 19000;
    int openedTime = 1000;

    if (millis() % (closedTime + openedTime) < closedTime)
    {
        setClaw(CLOSED);
    }
    else
    {
        setClaw(OPENED);
    }

    
    if (movementIndex == -1)
    { // tournant à droite
        if (forward(10, 96 / 2.0))
        {
            movementIndex++;
        }
    } else if (movementIndex == 0) { // tournant à droite
        if (rotate(20, 18 + 12, M_PI / 2.0)) {
            movementIndex++;
        }
    } else if(movementIndex == 1) { //segment tapis
        if (forward(20, 28)) {
            movementIndex++;
        }
    } else if(movementIndex == 2) { // tournant à droite
        if (rotate(20, 18 + 12 + 2, M_PI / 2.0 + M_PI / 8)) {
            movementIndex++;
        }
    } else if(movementIndex == 2) { // tournant à droite
        if (forward(20, 96)) {
            movementIndex++;
        }
    } else if(movementIndex == 2) { // tournant à droite
        if (rotate(20, 18 + 12, M_PI / 2.0)) {
            movementIndex++;
        }
    } else if(movementIndex == 2) { // tournant à droite
        if (forward(20, 24)) {
            movementIndex++;
        }
    } else if(movementIndex == 2) { // tournant à droite
        if (rotate(20, 18 + 12, M_PI / 2.0)) {
            movementIndex++;
        }
    }else if (movementIndex == 3) {
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
    

    //loopLineFollower();
    //followWall(RIGHT, 10);

    updatePIDs();
}

void loop()
{
   // Serial.println(state);
    delay(5);
    switch (state)
    {
    case 0: // Attente du sifflet, détection de la couleur de départ

        if (!Sifflet::active)
        {
            if (Sifflet::update(1.0))
            {
                Sifflet::trigger();
            }
        }
        if (Sifflet::active)
        {
            if (Couleur::Get() == 'v')
            {
                state = 1;
            }
            else if (Couleur::Get() == 'j')
            {
                state = 2;
            }
        }
        break;

    case 1: // Suivi du vert, détection du cup
        static byte state2 = 0;
        switch (state2) {
            case 0:
                moveUnited(5,INFINITY,0);
                if (CapteurLigne::isBlackLine()) state2++;
            break;
            case 1:
                if (rotate(10,18,PI/2.0)) state2++;
            break;
            case 2:
                if (forward(10,24)) state2++;
            break;
            case 3:
                if (rotate(10,18,PI/2.0)) state2++;
            break;
            case 4:
                moveUnited(10,INFINITY,PI);
                if (ROBUS_ReadIR(RIGHT) > 500)
                        {
                            setArm(EXTENDED_RIGHT);
                        }
                        if(Couleur::Get() == 'w')
                        {
                            setArm(NOT_EXTENDED);
                            state = 3;
                        }
            break;
        }
    break;

    case 2: // Suivi du jaune, détection du cup
        static byte state3 = 0;
        switch (state3) {
            case 0:
                moveUnited(2,INFINITY,0);
                if (CapteurLigne::isBlackLine()) state3++;
            break;
            case 1:
                if (rotate(10,30,PI/2.0)) state3++;
            break;
            case 2:
                if (forward(10,24)) state3++;
            break;
            case 3:
                if (rotate(10,30,PI/2.0)) state3++;
            break;
            case 4:
                moveUnited(10,INFINITY,PI);
                if (ROBUS_ReadIR(RIGHT) > 500)
                        {
                            setArm(EXTENDED_RIGHT);
                        }
                        if(Couleur::Get() == 'w')
                        {
                            setArm(NOT_EXTENDED);
                            state = 3;
                        }
                if (Couleur::Get() != 'w') state++;
            break;
        }
    break;

    case 3: // Suivi de la ligne, détection de retour à la couleur

        loopLineFollower();

        if(Couleur::Get() != 'w')
        {
            setClaw(OPENED);
            rotate(20, 18 + 12, M_PI / 4.0);
            state = 4;
        }
        break;

    case 4: // On fait un tour et puis le shortcut
    case 5:
        
        followWall(RIGHT, 15);

        if(CapteurLigne::isBlackLine())
        {
            state++;
        }
        break;

    case 6: // Feni

        move(0, 0);

        if(ROBUS_IsBumper(3))
        {
            setClaw(CLOSED);
            state = 0;// On restart le parcours
        }
        break;
    case 88:
    break;
    case 89:
        Serial.println(CapteurLigne::isBlackLine());
    break;
    case 90:
        Serial.println(Couleur::Get());
    break;
    }

    updateEverything();
}