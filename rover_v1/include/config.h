#ifndef ROVER_CONFIG_H
#define ROVER_CONFIG_H

#include <Arduino.h>

// =============================================================================
// 1. IDENTIFICACIÓN DEL ROVER
// =============================================================================
#define ROVER_ID                11          // 10 u 11 (ID ArUco del robot actual)
#define ROVER_PEER_ID           10          // ID ArUco del robot compañero

// =============================================================================
// 2. CONFIGURACIÓN DE RED WI-FI Y SERVIDOR DE VISIÓN
// =============================================================================
#define WIFI_SSID               "flia_martinez"
#define WIFI_PASSWORD           "CarterHugo2508*"

#define VISION_HOST             "192.168.88.13" // IP local de tu PC (wlp11s0)
#define VISION_PORT             2026            // Puerto oficial del reto (CONTRATO.md)

#define TELEMETRY_TIMEOUT_MS    500             // Watchdog de pérdida de visión (failsafe)
#define TCP_RETRY_INTERVAL_MS   1000            // Tiempo entre reintentos de reconexión

// =============================================================================
// 3. MAPA DE PINES DE LA HARDWARE (CRCibernetica IdeaBoard ESP32)
// =============================================================================
// Motores DC (Puentes H PWM)
#define PIN_M1A                 12          // Motor Izquierdo Canal A
#define PIN_M1B                 14          // Motor Izquierdo Canal B
#define PIN_M2A                 13          // Motor Derecho Canal A
#define PIN_M2B                 15          // Motor Derecho Canal B

#define INVERT_MOTOR_L          false       // Polaridad motor izquierdo
#define INVERT_MOTOR_R          true        // Polaridad motor derecho (invertido por montaje físico opuesto)

// Calibración de Motores (Trim) para compensar asimetrías mecánicas
// Si el robot se desvía a la izquierda, reduce el TRIM derecho (ej. 0.95f) o viceversa.
#define MOTOR_TRIM_L            1.00f
#define MOTOR_TRIM_R            1.00f

// Canales LEDC para PWM en ESP32
#define PWM_CH_M1A              0
#define PWM_CH_M1B              1
#define PWM_CH_M2A              2
#define PWM_CH_M2B              3
#define PWM_FREQ_HZ             50          // Frecuencia estándar IdeaBoard
#define PWM_RESOLUTION_BITS     10          // 10 bits = 0 a 1023

// Sensor Ultrasónico HC-SR04
#define PIN_SONAR_TRIG          25
#define PIN_SONAR_ECHO          26
#define OBSTACLE_DIST_STOP_CM   12.0f       // Distancia límite para considerar obstáculo frontal (cm)
#define UNSTUCK_DURATION_MS     500         // Tiempo de retroceso para desatascarse (ms)
#define STALL_TIME_MS           2500        // Tiempo sin avance para detectar bloqueo físico (ms)

// Sensores Infrarrojos de Piso (4IR)
#define PIN_IR_FL               36          // Adelante Izquierdo
#define PIN_IR_FR               39          // Adelante Derecho
#define PIN_IR_RL               34          // Atrás Izquierdo
#define PIN_IR_RR               35          // Atrás Derecho

// LED RGB NeoPixel Integrado
#define PIN_NEOPIXEL            2           // Pin oficial NeoPixel IdeaBoard (GPIO 2)
#define NUM_PIXELS              1

// =============================================================================
// 4. PARÁMETROS FÍSICOS Y CINEMÁTICOS DEL RETO
// =============================================================================
// Velocidades de motor [-1.0 a 1.0]
#define SPEED_CRUISE            0.40f       // Velocidad normal de aproximación
#define SPEED_PUSH              0.45f       // Velocidad constante durante el empuje
#define SPEED_TURN_MAX          0.50f       // Potencia de giro sobre su eje (50% para vencer fricción)
#define SPEED_RETREAT_L         -0.40f      // Reversa asimétrica izquierda (curva)
#define SPEED_RETREAT_R         -0.20f      // Reversa asimétrica derecha (curva)

// Ganancias de Control de Rumbo (P-Controller)
#define KP_STEERING             0.008f      // Ganancia proporcional de giro
#define ANGLE_DEADBAND_DEG      3.0f        // Zona muerta de alineación en grados
#define ANGLE_PIVOT_THRESH_DEG  30.0f       // Error angular para pivotar en el lugar

// Distancias y Tolerancias en Celdas (1 celda = 20 mm)
#define DIST_PREAPPROACH_CELLS  4.0f        // 8 cm detrás del cubo
#define TOL_DIST_ARRIVED_CELLS  1.5f        // Tolerancia para considerar llegada a pre-aproximación
#define TOL_ANGLE_ALIGNED_DEG   25.0f       // Tolerancia de ángulo alineado (25 deg para maniobrabilidad)

// Dimensiones de Cancha y Bordes
#define GRID_WIDTH_CELLS        50.0f
#define GRID_HEIGHT_CELLS       40.0f
#define EDGE_RISK_CELLS         2.5f        // 5 cm de borde de riesgo de caída
#define EDGE_SAFE_OFFSET_CELLS  4.0f        // Offset de punto intermedio de seguridad

// Dimensiones de Cubos y Acopio (según CONTRATO.md)
#define CUBE_SIDE_CELLS         2.5f        // 50 mm = 2.5 celdas
#define DEPOT_RADIUS_CELLS      3.75f       // 75 mm de radio (depósito de 150 mm)
#define DEPOT_DELIVERY_MARGIN   2.50f       // Radio máximo centro a centro para cubo adentro

// Temporizadores de Maniobras
#define RETREAT_DURATION_MS     1200        // Duración del Safe Retreat en milisegundos

#endif // ROVER_CONFIG_H
