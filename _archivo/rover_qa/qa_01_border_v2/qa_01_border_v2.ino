/**
 * =============================================================================
 * ROVER QA 01 V2 — ESQUELETO: CONFIG + TELEMETRÍA + HARDWARE
 * =============================================================================
 * Base mínima con:
 * - Conexión Wi-Fi y recepción de telemetría TCP (Core 0, 15-20 Hz).
 * - Control de motores, sensores IR, sonar y LED NeoPixel.
 * - Menú serial de diagnóstico para verificar hardware.
 * - Impresión continua de la posición y heading del rover.
 *
 * La lógica de navegación queda completamente a cargo del usuario.
 * =============================================================================
 */

#include "include/config.h"
#include "include/types.h"
#include "include/hardware.h"
#include "include/telemetry.h"
#include "include/qa_config.h"
#include "include/navigation.h"
#include <WiFi.h>
#include <WiFiUdp.h>

static uint32_t motor_diag_until_ms = 0;
static uint32_t last_print_ms = 0;

static void printMenu() {
    Serial.println("\n=======================================================");
    Serial.println("  ROVER QA 01 V2 — ESQUELETO (Config + Telemetría)     ");
    Serial.printf ("  Robot ID: %d | Holgura: %.1f celdas (%.1f cm)        \n", 
                   telemetryGetRoverId(), EDGE_CLEARANCE_CELLS, EDGE_CLEARANCE_CELLS * 2.0f);
    Serial.printf ("  Polaridad Motores: INVERT_L = %s | INVERT_R = %s      \n",
                   getMotorInvertL() ? "TRUE" : "FALSE", getMotorInvertR() ? "TRUE" : "FALSE");
    Serial.println("-------------------------------------------------------");
    Serial.println("  [f] Probar avance recto AMBOS motores por 1.2 seg");
    Serial.println("  [1] Probar SOLO Motor Izquierdo (M1) por 1.0 seg");
    Serial.println("  [2] Probar SOLO Motor Derecho (M2) por 1.0 seg");
    Serial.println("  [i] Invertir polaridad Motor Derecho (INVERT_R)");
    Serial.println("  [l] Invertir polaridad Motor Izquierdo (INVERT_L)");
    Serial.println("  [c] Alternar ID del Rover (10 <-> 11)");
    Serial.println("  [6] Mision MULTI-CUBO AUTONOMA (Busca 2 cubos por cercania)");
    Serial.println("  [7] Mision RETO GLOBAL (Coordinacion descentralizada)");
    Serial.println("  [r] Detener motores");
    Serial.println("  [h] Mostrar este menu de ayuda");
    Serial.println("=======================================================\n");
}

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);

    // 1. Inicializar actuadores y sensores
    hardwareInit();

    // 2. Iniciar tarea de recepción Wi-Fi TCP en Core 0
    telemetryInit();

    printMenu();
    Serial.println("[SETUP] Sistema listo. Telemetria activa. Esperando comandos...");
}

WiFiUDP udpCmd;
static bool udp_cmd_started = false;

void loop() {
    // Inicializar servidor de comandos UDP una vez que haya Wi-Fi
    if (WiFi.status() == WL_CONNECTED && !udp_cmd_started) {
        udpCmd.begin(8889);
        udp_cmd_started = true;
        Serial.println("[UDP] Servidor de comandos remoto listo en puerto 8889");
    }

    // =========================================================================
    // 1. RECEPCIÓN DE COMANDOS (Serial, UDP o Botón)
    // =========================================================================
    char cmd = 0;
    String str_cmd = "";
    
    if (Serial.available() > 0) {
        Serial.setTimeout(10);
        str_cmd = Serial.readStringUntil('\n');
        str_cmd.trim();
        if (str_cmd.length() == 1) {
            cmd = str_cmd.charAt(0);
        }
    } else if (udp_cmd_started && udpCmd.parsePacket()) {
        char packetBuffer[64];
        int len = udpCmd.read(packetBuffer, 63);
        if (len > 0) packetBuffer[len] = 0;
        str_cmd = String(packetBuffer);
        str_cmd.trim();
        if (str_cmd.length() == 1) {
            cmd = str_cmd.charAt(0);
        }
        udpCmd.flush(); // Limpiar el resto del paquete
    }

    // Interceptar comando extendido (ej. TRIM 0.95 1.0 o TRIM|0.95|1.0)
    if (str_cmd.startsWith("TRIM")) {
        // Parsear "TRIM 0.9 1.0" o "TRIM|0.9|1.0"
        str_cmd.replace('|', ' ');
        int space1 = str_cmd.indexOf(' ');
        int space2 = str_cmd.lastIndexOf(' ');
        if (space1 > 0 && space2 > space1) {
            float t_l = str_cmd.substring(space1 + 1, space2).toFloat();
            float t_r = str_cmd.substring(space2 + 1).toFloat();
            setMotorTrim(constrain(t_l, 0.0f, 2.0f), constrain(t_r, 0.0f, 2.0f));
            Serial.printf(">>> [CALIBRACION] Nuevos Trims: L=%.2f | R=%.2f\n", t_l, t_r);
        }
        cmd = 0; // Marcar como procesado
    }

    // Escuchar el botón físico BOOT
    static bool last_boot = HIGH;
    bool current_boot = digitalRead(PIN_BOOT_BUTTON);
    if (current_boot == LOW && last_boot == HIGH) {
        cmd = 'p'; // El botón dispara el patrullaje de bordes por cámara
        delay(50); // Anti-rebote
    }
    last_boot = current_boot;

    if (cmd != 0) {
        if (cmd == 'f' || cmd == 'F') {
            Serial.printf("[DIAG] Ambos motores adelante (InvL=%d, InvR=%d) por 1.2s...\n", getMotorInvertL(), getMotorInvertR());
            setMotors(QA_SPEED_CRUISE, QA_SPEED_CRUISE);
            motor_diag_until_ms = millis() + 1200;
        } else if (cmd == '1') {
            Serial.printf("[DIAG] Motor Izquierdo (M1) hacia adelante... (InvL=%d)\n", getMotorInvertL());
            setMotors(QA_SPEED_CRUISE, 0.0f);
            motor_diag_until_ms = millis() + 1000;
        } else if (cmd == '2') {
            Serial.printf("[DIAG] Motor Derecho (M2) hacia adelante... (InvR=%d)\n", getMotorInvertR());
            setMotors(0.0f, QA_SPEED_CRUISE);
            motor_diag_until_ms = millis() + 1000;
        } else if (cmd == 'i' || cmd == 'I') {
            bool new_r = !getMotorInvertR();
            setMotorInversions(getMotorInvertL(), new_r);
            Serial.printf("\n>>> [POLARIDAD] Motor DERECHO Invert_R = %s (InvL=%s)\n",
                          new_r ? "TRUE" : "FALSE", getMotorInvertL() ? "TRUE" : "FALSE");
        } else if (cmd == 'l' || cmd == 'L') {
            bool new_l = !getMotorInvertL();
            setMotorInversions(new_l, getMotorInvertR());
            Serial.printf("\n>>> [POLARIDAD] Motor IZQUIERDO Invert_L = %s (InvR=%s)\n",
                          new_l ? "TRUE" : "FALSE", getMotorInvertR() ? "TRUE" : "FALSE");
        } else if (cmd == 'c' || cmd == 'C') {
            int current = telemetryGetRoverId();
            int new_id = (current == 10) ? 11 : 10;
            int peer_id = (new_id == 10) ? 11 : 10;
            telemetrySetRoverId(new_id, peer_id);
            Serial.printf("\n>>> [ROVER ID] Cambiado a: %d (Companero: %d)\n\n", new_id, peer_id);
        } else if (cmd == 'r' || cmd == 'R') {
            stopMotors();
            motor_diag_until_ms = 0;
            Serial.println("[STOP] Motores detenidos.");
        } else if (cmd == 'm' || cmd == 'M') {
            TelemetrySnapshot snap;
            if (telemetryGetSnapshot(snap)) {
                printMap(snap);
            } else {
                Serial.println("[MAP] Sin datos de telemetria.");
            }
        } else if (cmd == 't' || cmd == 'T') {
            Serial.println("\n[MENU] >>> Iniciando TEST BORDER (Odometria IR)...");
            test_border();
        } else if (cmd == 'p' || cmd == 'P') {
            Serial.println("\n[MENU] >>> Iniciando PATRULLAJE DE BORDES (Telemetria)...");
            patrol_border();
        } else if (cmd == '3') {
            hunt_cube(COLOR_RED);
        } else if (cmd == '4') {
            hunt_cube(COLOR_GREEN);
        } else if (cmd == '7') {
            hunt_reto_mission();
        } else if (cmd == '6') {
            hunt_multiple_cubes(2);
        } else if (cmd == '5') {
            hunt_cube(COLOR_BLUE);
        } else if (cmd == 'h' || cmd == 'H') {
            printMenu();
        }
    }

    // Manejo del test manual de motores
    if (motor_diag_until_ms > 0) {
        if (millis() >= motor_diag_until_ms) {
            stopMotors();
            motor_diag_until_ms = 0;
            Serial.println("[DIAG] Test motor finalizado.");
        }
        delay(20);
        return;
    }

    // =========================================================================
    // 2. IMPRESIÓN DE TELEMETRÍA CADA 500 ms Y MAPA UDP CADA 1000 ms
    // =========================================================================
    if (millis() - last_print_ms > 500) {
        last_print_ms = millis();

        TelemetrySnapshot snap;
        bool ok = telemetryGetSnapshot(snap);

        if (ok && snap.my_rover.detected) {
            int fl, fr, rl, rr;
            readFloorSensors(fl, fr, rl, rr);

            Serial.printf("[TELEM] Pos:(%.1f, %.1f) Heading:%.1f° | Grid:%dx%d | Age:%ums | IR:(%d,%d,%d,%d)\n",
                snap.my_rover.x, snap.my_rover.y, snap.my_rover.heading,
                (int)snap.grid_cols, (int)snap.grid_rows, snap.my_rover.age_ms,
                fl, fr, rl, rr);
        } else {
            Serial.printf("[TELEM] WiFi:%s | TCP:%s | Rover %d: %s\n",
                (WiFi.status() == WL_CONNECTED) ? "OK" : "DESCONECTADO",
                (ok && snap.is_connected) ? "ENLACE_OK" : "SIN_CONEXION",
                telemetryGetRoverId(),
                (ok && snap.my_rover.detected) ? "DETECTADO" : "NO_VISTO");
        }

        // Actualización automática del Dashboard UDP cada 1000 ms (alternado)
        static bool alternate_map = false;
        if (alternate_map && ok) {
            printMap(snap);
        }
        alternate_map = !alternate_map;
    }

    // =========================================================================
    // 3. TU LÓGICA DE NAVEGACIÓN VA AQUÍ
    // =========================================================================
    // Tienes acceso a:
    //
    //   TelemetrySnapshot snap;
    //   telemetryGetSnapshot(snap);
    //     -> snap.my_rover.x, .y, .heading, .detected, .age_ms
    //     -> snap.peer_rover.x, .y, .heading, .detected
    //     -> snap.grid_cols, snap.grid_rows
    //     -> snap.phase (PHASE_SETUP, PHASE_RUNNING, PHASE_ENDED)
    //     -> snap.is_connected
    //
    //   setMotors(float left, float right);   // -1.0 a 1.0
    //   stopMotors();
    //   readFloorSensors(fl, fr, rl, rr);     // ADC 0-4095
    //   readUltrasonicCm();                   // distancia en cm
    //   setLedColor(r, g, b);                 // NeoPixel indicador
    //   isTelemetryFresh();                   // true si age < 500ms
    //


    delay(50); // ~20 Hz
}
