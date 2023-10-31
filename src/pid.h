#ifndef PID_H
#define PID_H

#include <Arduino.h>

namespace PID {
    struct valeursPID {
        valeursPID() : Kp(0.0), Ki(0.0), Kd(0.0), initialTime(0), Sp(0.0), Pv(0.0), integral(0.0), previous_error(0.0), Out(0.0) {}
        float Kp;           // Constante proportionnelle
        float Ki;           // Constante intégrale
        float Kd;           // Constante dérivée
        long initialTime;   // Temps initial
        float Sp;           // Set Point (Valeur voulue)
        float Pv;           // Process Value (Valeur réelle)
        float integral;     // Valeur intégrale
        float previous_error;  // Previous error value
        float Out;          // Valeur de sortie
        float update() {
            unsigned long currentTime = millis();  // Get current time in milliseconds

            // Calculate the time difference (dt) since the last update
            float dt = (currentTime - initialTime) / 1000.0; // Convert to seconds

            // Calculate the error
            float error = Sp - Pv;

            // Update the integral term
            integral += error * dt;

            // Calculate the derivative term
            float derivative = (error - previous_error) / dt;

            // Calculate the control output
            Out = Kp * error + Ki * integral + Kd * derivative;

            // Store the current time for the next update
            initialTime = currentTime;

            // Store the current error for the next iteration
            previous_error = error;

            return Out;
        }
    };
};

#endif // PID_H