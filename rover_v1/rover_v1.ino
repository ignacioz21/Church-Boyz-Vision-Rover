/**
 * =============================================================================
 * ROVER V1 — VISION ROVER CHALLENGE
 * Firmware Autónomo en C/C++ para ESP32 (IdeaBoard)
 * 
 * Cumplimiento de Reglamento:
 * - Autonomía total a bordo del ESP32 (Reglas 4.3 y 6.3)
 * - Fases oficiales: IDLE -> READY -> RUNNING (FSM de ALGORITMO.md)
 * - Recepción TCP NDJSON en puerto 2026 (Core 0 FreeRTOS)
 * - Control de motores y navegación diferencial a 50 Hz (Core 1)
 * =============================================================================
 */

#include "include/config.h"
#include "include/types.h"
#include "include/hardware.h"
#include "include/telemetry.h"
#include "include/navigation.h"
#include "include/fsm.h"

void setup() {
    // 1. Inicializar puerto serie para telemetría y diagnóstico
    Serial.begin(115200);
    delay(500);

    Serial.println("\n==============================================");
    Serial.println("  ROVER V1 — CENFOBOT AUTONOMOUS SYSTEM       ");
    Serial.printf ("  ID Asignado: %d                             \n", ROVER_ID);
    Serial.println("==============================================");

    // 2. Inicializar actuadores (LEDC PWM) y sensores
    hardwareInit();

    // 3. Inicializar máquina de estados (FSM)
    fsmInit();

    // 4. Iniciar tarea de recepción Wi-Fi TCP en Core 0
    telemetryInit();

    Serial.println("[SETUP] Sistema listo y en espera de telemetría...");
}

void loop() {
    // Bucle de control en Core 1 ejecutándose a 50 Hz (cada 20 ms)
    fsmUpdate();
    delay(20);
}
