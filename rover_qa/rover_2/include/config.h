#ifndef ROVER_CONFIG_H
#define ROVER_CONFIG_H

#include <Arduino.h>

// =============================================================================
// 1. IDENTIFICACIÓN OFICIAL DEL ROVER
// =============================================================================
#define ROVER_ID                11          // 10 u 11 (ID ArUco de este robot)
#define ROVER_PEER_ID           10          // ID ArUco del robot compañero

// =============================================================================
// 2. COMUNICACIÓN WI-FI Y SERVIDOR DE VISIÓN CENITAL
// =============================================================================
#define WIFI_SSID               "nodoarriba"
#define WIFI_PASSWORD           "21002296"

#define VISION_HOST             "192.168.86.149" // IP de la PC con la cámara
#define VISION_PORT             2026            // Puerto TCP oficial (CONTRATO.md)

#define TELEMETRY_TIMEOUT_MS    500             // Límite de latencia antes de activar failsafe
#define TCP_RETRY_INTERVAL_MS   1000            // Tiempo entre reintentos de reconexión

// =============================================================================
// 3. MAPA DE PINES (CRCibernetica IdeaBoard ESP32)
// =============================================================================
// Motores DC (Puentes H PWM)
#define PIN_M1A                 12          // Motor Izquierdo Canal A
#define PIN_M1B                 14          // Motor Izquierdo Canal B
#define PIN_M2A                 13          // Motor Derecho Canal A
#define PIN_M2B                 15          // Motor Derecho Canal B

// Polaridad física (Motores montados en sentidos opuestos en el chasis)
#define INVERT_MOTOR_L          false       // Motor Izquierdo
#define INVERT_MOTOR_R          true        // Motor Derecho (invertido como solicitaste)

// Calibración de Motores (Trim) para compensar asimetrías mecánicas
// Si el robot se desvía a la izquierda, reduce el TRIM derecho (ej. 0.95f) o viceversa.
#define MOTOR_TRIM_L            1.00f
#define MOTOR_TRIM_R            1.00f

// Configuración LEDC PWM en ESP32
#define PWM_CH_M1A              0
#define PWM_CH_M1B              1
#define PWM_CH_M2A              2
#define PWM_CH_M2B              3
#define PWM_FREQ_HZ             50          // Frecuencia estándar IdeaBoard (Hz)
#define PWM_RESOLUTION_BITS     10          // 10 bits = resolución 0 a 1023

// Sensor Ultrasónico HC-SR04
#define PIN_SONAR_TRIG          25
#define PIN_SONAR_ECHO          26
#define OBSTACLE_DIST_STOP_CM   12.0f

// Sensores Infrarrojos de Piso (4IR)
#define PIN_IR_FL               36          // Frontal Izquierdo
#define PIN_IR_FR               39          // Frontal Derecho
#define PIN_IR_RL               34          // Trasero Izquierdo
#define PIN_IR_RR               35          // Trasero Derecho

// LED RGB NeoPixel Integrado
#define PIN_NEOPIXEL            2           // GPIO 2 en IdeaBoard
#define NUM_PIXELS              1

// Botón Físico BOOT en la IdeaBoard
#define PIN_BOOT_BUTTON         0           // GPIO 0 con pullup interno

// =============================================================================
// 4. PARÁMETROS CINEMÁTICOS Y DE CONTROL (P-Controller)
// =============================================================================
#define SPEED_CRUISE            0.20f       // Velocidad nominal ultra-segura (~8 cm/s)
#define SPEED_SLOW              0.14f       // Velocidad de gateo/aproximación final
#define SPEED_TURN_MAX          0.32f       // Potencia máxima de pivote sobre su eje

#define KP_STEERING             0.004f      // Ganancia proporcional de guiado
#define ANGLE_DEADBAND_DEG      3.0f        // Zona muerta de alineación (grados)
#define ANGLE_PIVOT_THRESH_DEG  25.0f       // Umbral para girar en el lugar (grados)

// =============================================================================
// 5. DIMENSIONES ESTÁNDAR Y REGLAMENTO
// =============================================================================
#define CUBE_SIDE_CELLS         2.5f        // 50 mm = 2.5 celdas
#define DEPOT_RADIUS_CELLS      3.75f       // 75 mm de radio

#endif // ROVER_CONFIG_H
