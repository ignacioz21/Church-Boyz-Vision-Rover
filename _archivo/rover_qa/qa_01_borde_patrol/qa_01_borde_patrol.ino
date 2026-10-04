/**
 * =============================================================================
 * ROVER QA 01 — PATRULLAJE DE BORDES CON HOLGURA
 * =========1====================================================================
 * Programa de Control de Calidad de Datos (QA) y Verificación en Campo:
 * - Valida la telemetría recibida a 20 Hz por TCP en puerto 2026.
 * - Detecta el borde perimetral en la dirección de avance con holgura de seguridad.
 * - Frena de forma determinista y ejecuta giros a la izquierda de 90° en su eje.
 * - Bordeo completo de los 4 lados de la cancha (1 vuelta).
 * - Monitorea en tiempo real reflectancia (4IR), ultrasonido y latencia de red.
 * 
 * Modos de disparo:
 * 1. Automático: Cambio oficial de visión a fase RUNNING.
 * 2. Manual: Presionar botón físico BOOT (GPIO 0) o enviar 's' por Serial USB.
 * =============================================================================
 */

#include "include/config.h"
#include "include/types.h"
#include "include/hardware.h"
#include "include/telemetry.h"
#include "include/navigation.h"
#include "include/qa_config.h"
#include "include/qa_fsm.h"

static uint32_t motor_diag_until_ms = 0;

static void printMenu() {
    Serial.println("\n=======================================================");
    Serial.println("  ROVER QA 01 — PATRULLAJE DE BORDES CON HOLGURA       ");
    Serial.printf ("  Robot ID: %d | Holgura: %.1f celdas (%.1f cm)        \n", 
                   telemetryGetRoverId(), EDGE_CLEARANCE_CELLS, EDGE_CLEARANCE_CELLS * 2.0f);
    Serial.printf ("  Polaridad Motores: INVERT_L = %s | INVERT_R = %s      \n",
                   getMotorInvertL() ? "TRUE" : "FALSE", getMotorInvertR() ? "TRUE" : "FALSE");
    Serial.println("-------------------------------------------------------");
    Serial.println("  [s] Iniciar patrullaje con VISION (o presionar BOOT)");
    Serial.println("  [t] Prueba de BANCO autonoma (avance + giro 90°)");
    Serial.println("  [f] Probar avance recto AMBOS motores por 1.2 seg");
    Serial.println("  [1] Probar SOLO Motor Izquierdo (M1) por 1.0 seg");
    Serial.println("  [2] Probar SOLO Motor Derecho (M2) por 1.0 seg");
    Serial.println("  [i] Invertir polaridad Motor Derecho (INVERT_R)");
    Serial.println("  [l] Invertir polaridad Motor Izquierdo (INVERT_L)");
    Serial.println("  [c] Alternar ID del Rover (10 <-> 11)");
    Serial.println("  [r] Reiniciar FSM / Detener motores");
    Serial.println("  [h] Mostrar este menu de ayuda");
    Serial.println("=======================================================\n");
}

void setup() {
    Serial.begin(115200);
    delay(500);

    // Configuración del botón físico BOOT (GPIO 0) con pullup interno
    pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);

    // 1. Inicializar actuadores y sensores
    hardwareInit();

    // 2. Inicializar máquina de estados de QA
    qaFsmInit();

    // 3. Iniciar tarea de recepción Wi-Fi TCP en Core 0
    telemetryInit();

    printMenu();
    Serial.println("[QA SETUP] Sistema listo. Esperando inicio...");
}

void loop() {
    // 1. Leer comandos desde el monitor serie USB
    if (Serial.available() > 0) {
        char cmd = (char)Serial.read();
        if (cmd == 's' || cmd == 'S') {
            qaFsmTriggerStart();
        } else if (cmd == 't' || cmd == 'T') {
            qaFsmTriggerBenchTest();
        } else if (cmd == 'r' || cmd == 'R') {
            Serial.println("[QA] Reiniciando prueba y deteniendo motores...");
            motor_diag_until_ms = 0;
            qaFsmInit();
        } else if (cmd == '1') {
            Serial.printf("[DIAG] Motor Izquierdo (M1) hacia adelante... (InvL=%d)\n", getMotorInvertL());
            setMotors(QA_SPEED_CRUISE, 0.0f);
            motor_diag_until_ms = millis() + 1000;
        } else if (cmd == '2') {
            Serial.printf("[DIAG] Motor Derecho (M2) hacia adelante... (InvR=%d)\n", getMotorInvertR());
            setMotors(0.0f, QA_SPEED_CRUISE);
            motor_diag_until_ms = millis() + 1000;
        } else if (cmd == 'f' || cmd == 'F') {
            Serial.printf("[DIAG] Ambos motores adelante (InvL=%d, InvR=%d) por 1.2s...\n", getMotorInvertL(), getMotorInvertR());
            setMotors(QA_SPEED_CRUISE, QA_SPEED_CRUISE);
            motor_diag_until_ms = millis() + 1200;
        } else if (cmd == 'i' || cmd == 'I') {
            bool new_r = !getMotorInvertR();
            setMotorInversions(getMotorInvertL(), new_r);
            Serial.printf("\n>>> [POLARIDAD] Motor DERECHO Invert_R = %s (InvL=%s)\n",
                          new_r ? "TRUE (Invertido)" : "FALSE (Normal)",
                          getMotorInvertL() ? "TRUE" : "FALSE");
            Serial.println(">>> Envia 'f' para verificar si ahora avanza recto!\n");
        } else if (cmd == 'l' || cmd == 'L') {
            bool new_l = !getMotorInvertL();
            setMotorInversions(new_l, getMotorInvertR());
            Serial.printf("\n>>> [POLARIDAD] Motor IZQUIERDO Invert_L = %s (InvR=%s)\n",
                          new_l ? "TRUE (Invertido)" : "FALSE (Normal)",
                          getMotorInvertR() ? "TRUE" : "FALSE");
            Serial.println(">>> Envia 'f' para verificar si ahora avanza recto!\n");
        } else if (cmd == 'c' || cmd == 'C') {
            int current = telemetryGetRoverId();
            int new_id = (current == 10) ? 11 : 10;
            int peer_id = (new_id == 10) ? 11 : 10;
            telemetrySetRoverId(new_id, peer_id);
            Serial.printf("\n>>> [ROVER ID] ID del rover cambiado a: %d (Companero: %d) <<<\n\n", new_id, peer_id);
        } else if (cmd == 'h' || cmd == 'H') {
            printMenu();
        }
    }

    // Manejo de apagado del test manual de diagnóstico
    if (motor_diag_until_ms > 0) {
        if (millis() >= motor_diag_until_ms) {
            stopMotors();
            motor_diag_until_ms = 0;
            Serial.println("[DIAG] Test motor finalizado (frenado).");
        }
        delay(20);
        return; // No dejar que la FSM interfiera durante el pulso de diagnóstico
    }

    // 2. Leer botón físico BOOT (pulsación corta: inicio con visión; pulsación larga >= 1.5s: prueba de banco)
    static uint32_t button_press_start_ms = 0;
    static bool button_was_pressed = false;

    if (digitalRead(PIN_BOOT_BUTTON) == LOW) {
        if (!button_was_pressed) {
            button_was_pressed = true;
            button_press_start_ms = millis();
        }
    } else {
        if (button_was_pressed) {
            uint32_t press_duration = millis() - button_press_start_ms;
            button_was_pressed = false;

            if (press_duration >= 1500) {
                // Pulsación larga: Prueba de banco sin cámara
                qaFsmTriggerBenchTest();
            } else if (press_duration >= 50) {
                // Pulsación corta: Prueba con visión
                qaFsmTriggerStart();
            }
        }
    }

    // 3. Ejecutar máquina de estados de QA a 50 Hz
    qaFsmUpdate();
    delay(20);
}
