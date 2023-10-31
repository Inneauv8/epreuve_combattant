#include "movement.h"

namespace Movement {

    float pulseToDist = M_PI*WHEEL_DIAMETER/3200.0;
    float orientationOffset = 0;

    float computeOrientation() {
        float deltaS = (ENCODER_Read(LEFT) - ENCODER_Read(RIGHT)) * pulseToDist / 2.0;
        float theta = deltaS * 2 / (WHEEL_BASE_DIAMETER);

        return theta - orientationOffset;
    }

    float computeDistance() {
        float distance = (ENCODER_Read(LEFT) + ENCODER_Read(RIGHT)) * pulseToDist / 2.0;
        
        return distance;
    }

    bool distanceFlag(float distance, float *initialDistance) {
        float actualDistance = computeDistance();

        if (isnan(*initialDistance)) {
            *initialDistance = actualDistance;
        }

        bool distanceReached = fabs(actualDistance - *initialDistance) >= fabs(distance);
        
        if (distanceReached) {
            *initialDistance = INACTIVE;
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
            *initialOrientation = INACTIVE;
        }

        return angleReached;
    }


    bool distanceFlag(float distance) {
        static float initialDistance = INACTIVE;
        return distanceFlag(distance, &initialDistance);
    }

    bool orientationFlag(float angle) {
        static float initialOrientation = INACTIVE;
        return orientationFlag(angle, &initialOrientation);
    }

    bool isInactive(float var) {
        return isnan(var);
    }

    void rotate(float velocity, float radius) {
        //WheelVelocities velocities = moveByRadius(velocity, radius);

        move(velocity, isinf(radius) ? 0 : (velocity / radius));

        //rightPID.Sp = velocities.rightVelocity;
        //leftPID.Sp = velocities.leftVelocity;
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
        float angularVelocity = sigmoid(targetAngle, 0, 1, 0.1, -2) * -baseAngularVelocity;
        //Serial.println(angularVelocity);

        move(velocity, angularVelocity);
    }

    bool rotate(float velocity, float radius, float angle, boolean reset) {
        static float initialOrientation = NAN;
        float actualOrientation = computeOrientation();

        if (isnan(initialOrientation)) {
            initialOrientation = actualOrientation;
        }

        float targetOrientation = wrap((initialOrientation + fabs(angle * 2) * (radius > 0 ? 1 : -1)), -M_PI, M_PI);

        bool angleReached = fabs(actualOrientation - initialOrientation) >= fabs(angle) || reset;

        if (angleReached) {
            //move(0, 0);
        } else {
            rotate(velocity, radius);
            //moveUnited(velocity, radius, initialOrientation + angle);
        }

        //rotate(velocity, radius);
        
        if (angleReached) {
            initialOrientation = NAN;
        }

        return angleReached;
    }

    bool rotateAngularVelocity(float velocity, float angularVelocity, float angle, boolean reset) {
        static float initialOrientation = NAN;
        float actualOrientation = computeOrientation();

        if (isnan(initialOrientation)) {
            initialOrientation = actualOrientation;
        }

        float targetOrientation = wrap((initialOrientation + fabs(angle * 2) * (radius > 0 ? 1 : -1)), -M_PI, M_PI);

        bool angleReached = fabs(actualOrientation - initialOrientation) >= fabs(angle) || reset;

        if (angleReached) {
            //move(0, 0);
        } else {
            move(velocity, angularVelocity);
            //moveUnited(velocity, radius, initialOrientation + angle);
        }

        //rotate(velocity, radius);
        
        if (angleReached) {
            initialOrientation = NAN;
        }

        return angleReached;
    }

    bool forward(float velocity, float distance, boolean reset) {
        static float initialDistance = NAN;
        static float initialOrientation = NAN;

        float actualDistance = computeDistance();
        //float actualOrientation = computeOrientation();

        if (isnan(initialDistance)) {
            initialDistance = computeDistance();
            initialOrientation = computeOrientation();
        }

        bool distanceReached = fabs(actualDistance - initialDistance) >= fabs(distance) || reset;

        if (distanceReached) {
            //move(0, 0);
        } else {
            //moveUnited(velocity, velocity / 5.0, initialOrientation);
            moveUnited(velocity, 5.0 / 10.0, initialOrientation);
        }

        //float correction = sigmoid(actualOrientation, initialOrientation, 1, 0.1, -2) * 5;
        
        //move(distanceReached ? 0 : velocity, 0);

        if (distanceReached) {
            initialDistance = NAN;
            initialOrientation = NAN;
        }

        return distanceReached;
    }
    
    float computeLeftMotorSpeed()
    {
    
        static float past = 0.0;
        static float speedMotor = 0.0;
        static float oldPulse = 0.0;
        float present = micros();
        float pulse = ENCODER_Read(LEFT);
        speedMotor = 1000000.0 * pulseToDist * float(pulse-oldPulse) / float(present - past);
    
        past = present;
        oldPulse = pulse;
        
        return speedMotor;
    }

    float computeRightMotorSpeed()
    {
        static float past = 0.0;
        static float speedMotor = 0.0;
        static float oldPulse = 0.0;
        float present = micros();
        float pulse = ENCODER_Read(RIGHT);
        speedMotor = 1000000.0 * pulseToDist * float(pulse-oldPulse) / float(present - past);
        
        past = present;
        oldPulse = pulse;
        
        return speedMotor;
    }

    void setPIDRight(float Kp, float Ki, float Kd) {
        rightPID.Kp = Kp;
        rightPID.Ki = Ki;
        rightPID.Kd = Kd;
    }
    void setPIDLeft(float Kp, float Ki, float Kd) {
        leftPID.Kp = Kp;
        leftPID.Ki = Ki;
        leftPID.Kd = Kd;
    }

    void setRightSpeed(float speed) {
        rightPID.Sp = speed;
    }
    void setLeftSpeed(float speed) {
        leftPID.Sp = speed;
    }
    
    void updatePIDs() {
        rightPID.Pv = computeRightMotorSpeed();
        MOTOR_SetSpeed(RIGHT, clamp(rightPID.update(), -1, 1));

        leftPID.Pv = computeLeftMotorSpeed();
        MOTOR_SetSpeed(LEFT, clamp(leftPID.update(), -1, 1));
    }


    namespace {
        PID::valeursPID rightPID = {};
        PID::valeursPID leftPID = {};
    }
}