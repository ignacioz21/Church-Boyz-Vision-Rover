#ifndef ROVER_HARDWARE_H
#define ROVER_HARDWARE_H

#include <Arduino.h>

void hardwareInit();

// Potencia de cada lado en [-1.0, 1.0]. Positivo = adelante (ya corrige polaridad).
void setMotors(float left, float right);
void stopMotors();

// Distancia al frente en cm (999 si no hay eco)
float readUltrasonicCm();

// Reflectancia de piso, ADC 0..4095
void readFloorSensors(int &fl, int &fr, int &rl, int &rr);

void setLedColor(uint8_t r, uint8_t g, uint8_t b);

// Giroscopio integrado (LSM6DS3TR-C, I2C). imuReady() = el sensor respondió al arrancar.
// gyroZDps() = velocidad de giro del robot en grados/s (0 si no hay sensor). Es
// inmediata: la cámara tarda ~0,4 s en mostrar lo mismo.
bool imuReady();
float gyroZDps();
// Llamar con el robot QUIETO: acerca de a poco el cero del giroscopio a lo que mide ahora
void gyroRezero();

// TODO: sensor de color (TCS34725) si se usa.

#endif // ROVER_HARDWARE_H
