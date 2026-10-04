#ifndef ROVER_HARDWARE_H
#define ROVER_HARDWARE_H

#include <Arduino.h>
#include "types.h"

// Inicialización de periféricos (LEDC PWM, GPIOs, Sensores)
void hardwareInit();

// Control de motores DC (-1.0 a 1.0)
void setMotors(float left, float right);

// Inversión dinámica de polaridad de motores
void setMotorInversions(bool invert_l, bool invert_r);
bool getMotorInvertL();
bool getMotorInvertR();

// Freno inmediato de ambos motores
void stopMotors();

// Lectura de distancia ultrasónica en centímetros (HC-SR04)
float readUltrasonicCm();

// Lectura de reflectancia analógica de los 4 sensores infrarrojos de piso
void readFloorSensors(int &fl, int &fr, int &rl, int &rr);

// Control de color RGB del LED NeoPixel
void setLedColor(uint8_t r, uint8_t g, uint8_t b);

// Asignación de color según el estado actual de la FSM
void setLedForState(RoverFsmState state);

#endif // ROVER_HARDWARE_H
