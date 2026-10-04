#ifndef ROVER_CONFIG_H
#define ROVER_CONFIG_H

#include <Arduino.h>
#include "secrets.h"   // WIFI_SSID / WIFI_PASSWORD (no se versiona; ver secrets.example.h)

// =============================================================================
// 1. IDENTIDAD — lo único que cambia entre rover_10 y rover_11 (junto con §3)
// =============================================================================
#define ROVER_ID                10          // ID ArUco de ESTE robot
#define ROVER_PEER_ID           11          // ID ArUco del compañero

// =============================================================================
// 2. RED
// =============================================================================
#define VISION_HOST             "192.168.88.11" // PC que corre la visión
#define VISION_PORT             2026            // Puerto oficial (CONTRATO.md)

#define BRAIN_HOST              "192.168.88.11" // PC que corre el cerebro (monitor)
#define BRAIN_STATUS_PORT       8888            // Rover -> cerebro: estado
#define ROVER_CMD_PORT          8889            // Cerebro -> rover: comandos de prueba

#define TELEMETRY_TIMEOUT_MS    500     // Pose más vieja que esto => no se avanza
#define TELEMETRY_STALL_MS      1500    // Sin línea válida por este tiempo => reconectar
#define TELEMETRY_LINE_MAX      1200    // Largo máximo de una línea NDJSON (hoy ~690)
#define TCP_RETRY_INTERVAL_MS   1000

// =============================================================================
// 3. HARDWARE (CRCibernetica IdeaBoard ESP32)
// =============================================================================
#define PIN_M1A                 12      // Motor izquierdo
#define PIN_M1B                 14
#define PIN_M2A                 13      // Motor derecho
#define PIN_M2B                 15

#define INVERT_MOTOR_L          true    // Polaridad física de ESTE robot
#define INVERT_MOTOR_R          true

#define PWM_CH_M1A              0
#define PWM_CH_M1B              1
#define PWM_CH_M2A              2
#define PWM_CH_M2B              3
#define PWM_FREQ_HZ             50
#define PWM_RESOLUTION_BITS     10      // 0..1023

#define PIN_SONAR_TRIG          25
#define PIN_SONAR_ECHO          26

#define PIN_IR_FL               36
#define PIN_IR_FR               39
#define PIN_IR_RL               34
#define PIN_IR_RR               35

#define PIN_NEOPIXEL            2

// =============================================================================
// 4. CALIBRACIÓN DE ESTE ROBOT (R10: promedio de 5 mediciones, 2026-10-03) — medir con el comando de práctica "L" y copiar
//    aquí los valores que reporta (latencia, vgain, wgain). Unidades: celdas.
// =============================================================================
#define LOOP_LATENCY_MS         435.0f  // Orden de motor -> movimiento visto en telemetría
#define SPEED_GAIN              17.8f   // celdas/s por unidad de potencia (avance)
#define TURN_GAIN               335.0f  // grados/s por unidad de potencia (pivote)

// Geometría del robot (frente = pinzas). Estimada sobre foto con la cuadrícula;
// confirmar con regla. AXLE_OFFSET y CONTACT_OFFSET se miden desde el centro del
// MARCADOR; la huella (FP_*), desde el EJE DE LAS RUEDAS, que es el centro de giro.
#define AXLE_OFFSET             1.8f    // El eje de las ruedas está esto DETRÁS del marcador
#define CONTACT_OFFSET          4.6f    // Marcador -> centro del cubo cuando está dentro de las pinzas
#define FP_REAR                 1.9f    // Eje -> cola
#define FP_FRONT                4.9f    // Eje -> frente del chasis (donde apoya el cubo)
#define FP_HALF_WIDTH           3.25f   // Medio ancho con ruedas
#define FP_PRONG_TIP            7.8f    // Eje -> punta de las pinzas
#define FP_PRONG_SIDE           2.9f    // Línea central de cada pinza, a cada lado

#define POWER_MIN_MOVE          0.22f   // Mínimo que vence la fricción avanzando
#define POWER_MIN_PIVOT         0.30f   // Mínimo que vence la fricción pivotando
#define POWER_MAX_PIVOT         0.45f
#define POWER_CRUISE            0.50f
#define POWER_PUSH              0.35f

// =============================================================================
// 5. ESTRATEGIA (igual en ambos rovers)
// =============================================================================
// Fase en la que arranca la estrategia. Versión final: PHASE_READY (reglamento 9.3).
// Con el sistema de visión actual READY es una cuenta de preparación: para
// pruebas se puede usar PHASE_RUNNING.
#define START_PHASE             PHASE_READY

// Todas las distancias de la estrategia se miden desde el EJE DE LAS RUEDAS.
#define LINE_TOLERANCE          2.0f    // Cuánto puede sobresalir el robot de las líneas de la cancha
// Punto de preparación: desde ahí se avanza recto hacia el cubo. El eje debe quedar
// al menos a esta distancia, para que las puntas (FP_PRONG_TIP) no alcancen el
// cubo mientras se afina la puntería.
#define STAGE_DIST_MIN          11.0f
// Tramo final de una entrega (recto hacia el centro de la zona): corrección máxima
// de rumbo, como diferencia de potencia entre ruedas.
#define CARRY_STEER_MAX         0.10f
#define YIELD_DIST              17.0f   // Compañero más cerca que esto y en mi camino => cedo
#define YIELD_MAX_MS            5000    // Tras esperar esto, se rodea
#define RETREAT_DIST            7.0f    // Retroceso tras soltar el cubo
#define PUSH_TIMEOUT_MS         25000
#define MAX_ATTEMPTS            3       // Reintentos por cubo antes de pasar al siguiente

#endif // ROVER_CONFIG_H
