#ifndef ROVER_HARDWARE_H
#define ROVER_HARDWARE_H

#include <Arduino.h>
#include "types.h"

// Inicialización de actuadores, canales LEDC PWM, GPIOs y sensores
void hardwareInit();

// Control de velocidad de motores DC (-1.0 a 1.0)
void setMotors(float left, float right);

// Inversión dinámica de polaridad de motores (permite conmutar en caliente)
void setMotorInversions(bool invert_l, bool invert_r);
bool getMotorInvertL();
bool getMotorInvertR();
void setMotorTrim(float trim_l, float trim_r);
float getMotorTrimL();
float getMotorTrimR();

// Calibracion dinamica
// Freno inmediato de ambos motores (duty 0)
void stopMotors();

// Lectura de distancia ultrasónica en centímetros (HC-SR04)
float readUltrasonicCm();

// Lectura de reflectancia analógica de los 4 sensores infrarrojos de piso
void readFloorSensors(int &fl, int &fr, int &rl, int &rr);

// Control de color RGB del LED NeoPixel integrado (GPIO 2)
void setLedColor(uint8_t r, uint8_t g, uint8_t b);

// Asignación de color según el estado actual de la FSM
void setLedForState(RoverFsmState state);

// Lee el sensor de color TCS34725 y verifica si coincide con el color esperado
bool verifyCubeColor(CubeColor expected_color);

#endif // ROVER_HARDWARE_H
