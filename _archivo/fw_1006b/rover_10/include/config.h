#ifndef ROVER_CONFIG_H
#define ROVER_CONFIG_H

#include <Arduino.h>
#include "secrets.h"   // WIFI_SSID / WIFI_PASSWORD (no se versiona; ver secrets.example.h)

// =============================================================================
// 1. IDENTIDAD — lo único que cambia entre rover_10 y rover_11 (junto con §3)
// =============================================================================
#define ROVER_ID                10          // ID ArUco de ESTE robot
#define ROVER_PEER_ID           11          // ID ArUco del compañero

// Versión del firmware: va en el reporte de estado y queda en la ficha de cada ronda
// grabada. Cambiarla cada vez que cambie el comportamiento, para poder separar las
// rondas hechas con una versión de las hechas con otra.
#define FW_VERSION              "1006b"

// =============================================================================
// 2. RED
// =============================================================================
#define VISION_HOST             "192.168.88.11" // PC que corre la visión
#define VISION_PORT             2026            // Puerto oficial (CONTRATO.md)

#define BRAIN_HOST              "192.168.88.11" // PC que corre el cerebro (monitor)
#define BRAIN_STATUS_PORT       8888            // Rover -> cerebro: estado
#define ROVER_CMD_PORT          8889            // Cerebro -> rover: comandos de prueba
#define ROVER_PEER_PORT         8887            // Rover <-> rover: estado para coordinarse (reglamento 7.2-7.5)

// Durante una ronda oficial (READY/RUNNING) el rover ignora todo comando externo.
// Esto solo decide si sigue REPORTANDO su estado al monitor de la PC (que no le
// responde). Poner en 0 si los jueces no permiten ni siquiera eso.
#define STATUS_DURING_ROUND     1

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

// Arranque suave (hardware.cpp): evita el pico de corriente que reinicia la placa
#define MOTOR_RAMP_MS           250     // Tiempo para subir de 0 a potencia 1.0
#define MOTOR_RAMP_START        0.18f   // La rampa parte de aquí (menos no mueve la rueda)
#define MOTOR_REVERSE_PAUSE_MS  80      // Pausa en cero antes de invertir un motor
#define BROWNOUT_DETECTOR       0       // 0 = un bajón breve de voltaje no reinicia la placa
#define MOTOR_SCALE             1.00f   // Propio de ESTE robot: baja toda la potencia (iguala al rover más lento)

#define PIN_SONAR_TRIG          25
#define PIN_SONAR_ECHO          26

#define PIN_IR_FL               36
#define PIN_IR_FR               39
#define PIN_IR_RL               34
#define PIN_IR_RR               35

#define PIN_NEOPIXEL            2

// 1 = usar el giroscopio. 0 = no tocar el bus I2C para nada (el rumbo se estima como
// antes, por la potencia mandada, y el ajuste fino va a toquecitos). Sirve para
// comprobar si los reinicios "colgado" vienen del giroscopio.
#define USE_GYRO                1
#define PIN_I2C_SDA             21      // Giroscopio integrado (LSM6DS3TR-C)
#define PIN_I2C_SCL             22

// =============================================================================
// 4. CALIBRACIÓN DE ESTE ROBOT (R10: promedio de 5 mediciones, 2026-10-04, baterías nuevas) — medir con el comando de práctica "L" y copiar
//    aquí los valores que reporta (latencia, vgain, wgain). Unidades: celdas.
// =============================================================================
#define LOOP_LATENCY_MS         413.0f  // Orden de motor -> movimiento visto en telemetría
#define SPEED_GAIN              19.4f   // celdas/s por unidad de potencia (avance)
#define TURN_GAIN               358.0f  // grados/s por unidad de potencia (pivote)

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
#define MAX_SPEED               10.0f   // celdas/s: tope de avance (más rápido, la cámara pierde el marcador)
#define PULSE_MIN_MS            90.0f   // Pulso más corto de ajuste fino de rumbo (se acorta solo si el robot se pasa)

// Giro trabado (algo en el piso frena al robot): se detecta con el giroscopio
// Regulación de la velocidad de giro con el giroscopio (motion.cpp, pivotRegulate)
#define PIVOT_RATE_CARRY        45.0f   // grados/s que se buscan al girar con un cubo en las pinzas
#define PIVOT_TRIM_GAIN         0.008f  // Qué tan rápido se corrige la potencia (por grado/s de error y por segundo)
#define PIVOT_TRIM_MAX          0.35f   // Tope de potencia extra
// Ajuste fino del rumbo con giroscopio (motion.cpp, motionTurnTo): giro continuo y suave
#define FINE_POWER              0.24f   // Potencia base de un retoque SIN cubo (menos que un giro normal)
#define FINE_POWER_CARRY        0.32f   // ...con un cubo en las pinzas
#define FINE_POWER_PER_DEG      0.004f  // Extra por cada grado que falta (ajuste más grande, más potencia)
#define FINE_KICK_STEP          0.004f  // Si no arranca, la potencia sube esto por ciclo (20 ms)
#define FINE_KICK_MAX           0.10f   // Tope de esa subida sin cubo
#define FINE_KICK_MAX_CARRY     0.28f   // ...y con cubo
#define FINE_COAST_S            0.10f   // Se corta cuando falta lo que el robot gira en este tiempo
#define FINE_SETTLE_EXTRA_MS    150     // Tras cada movimiento se espera quieto latencia + esto, hasta que la cámara lo muestre
#define FINE_MOVE_MAX_MS        1500    // Un movimiento que no termina en este tiempo se corta
#define FINE_HOLD_EXTRA         2.5f    // Ya apuntado, se tolera este desvío extra sin volver a corregir
#define GYRO_SIGN               1.0f    // +1 si el giroscopio da positivo al girar a la izquierda (comprobado en R10 y R11)
#define STALL_WINDOW_MS         900     // Ventana en la que se mide cuánto giró mientras los motores pivotan
#define STALL_WINDOW_CARRY_MS   2500    // ...con un cubo en las pinzas
#define STALL_MIN_DEG           3.0f    // Menos giro que esto en la ventana => está trabado
#define STALL_HALT_MS           1500    // Tiempo detenido avisando "OBSTACULO" antes de hacer nada
#define STALL_BUMP_MS           450     // Después: corrimiento recto (potencia normal) para salir de ese punto

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
#define NAV_ARRIVE_DIST         1.5f    // Con cubo: eje a menos de esto del destino de una ruta => llegó
// Tramo final de una entrega (recto hacia el centro de la zona): corrección máxima
// de rumbo, como diferencia de potencia entre ruedas.
#define CARRY_STEER_MAX         0.16f
#define TURN_WITH_CUBE_COST     0.20f   // Celdas de recorrido que "cuesta" cada grado a girar con el cubo (al elegir por dónde tomarlo)
// Dejar el cubo en el CENTRO de su zona. El margen es chico: el centro del cubo solo
// puede apartarse 1,63 celdas del centro de la zona en el sentido del fondo.
#define DROP_DEPTH_TOL 0.4f    // Se sigue empujando mientras le falte más que esto para el centro
#define DROP_MAX_NUDGES 8       // Empujones cortos, cada uno confirmado con la cámara
#define DROP_SIDE_TOL           1.5f    // Desviado de costado más que esto (y aún lejos): volver a apuntar
#define DROP_SAFE_MARGIN 0.8f    // Maniobrando: solo se suelta si al cubo le sobra esto hasta el límite
#define DROP_MIN_MARGIN         0.4f    // Entregado con menos margen que esto: retoque
#define DROP_RETOUCH 1       // 0 = sin retoque (si en la práctica cuesta más de lo que protege)
#define DIRECT_MAX_TURN         110.0f   // Con el cubo: si el destino queda a menos de este giro y el tramo está libre, ir derecho
#define AIM_WITH_CUBE_MS        12000   // Tiempo máximo para apuntar al destino con el cubo; si no lo logra, lo suelta y lo retoma
#define CAPTURE_SEEN_SLACK      2.6f    // El cubo se da por tomado cuando la cámara (con su atraso) lo ve a menos de esto del frente
#define SEAT_TOLERANCE          0.6f    // Cubo a más de esto del frente => no está asentado: avanzar antes de girar
#define SEAT_SLIP               1.0f    // Cubo adelantado más de esto (visto de forma sostenida) => parar y asentarlo
// Coordinación entre los dos rovers (coord.h)
#define PEER_PUBLISH_MS         100     // Cada cuánto le cuento al compañero qué estoy haciendo
#define PEER_TIMEOUT_MS         600     // Sin noticias por este tiempo => solo me guío por la cámara
#define PEER_PATIENCE_MS        12000   // Cuánto espero a que el compañero despeje antes de buscar otra salida
#define DEPART_GAP              15.0f   // Con el compañero a menos de esto, no salimos los dos a la vez
#define DEPART_WAIT_MS          6000    // Cuánto espera el de mayor ID a que el otro se aleje
#define YIELD_DIST              17.0f   // Compañero más cerca que esto y en mi camino => cedo
#define YIELD_MAX_MS            5000    // Tras esperar esto, se rodea
#define RETREAT_DIST            7.0f    // Retroceso tras soltar el cubo
#define PUSH_TIMEOUT_MS         25000
#define MAX_ATTEMPTS            5       // Intentos por cubo (cada uno prueba una forma distinta de tomarlo)
#define NO_PROGRESS_MS          20000   // Sin avanzar este tiempo => se descarta lo que estaba intentando
// Área de repelencia: las rutas prefieren pasar lejos de esto cuando hay lugar
#define REPEL_CUBE_RADIUS       11.0f   // Alrededor de un cubo
#define REPEL_PEER_RADIUS       15.0f   // Alrededor del compañero
#define REPEL_LINE_DIST         6.0f    // Franja junto a las líneas de la cancha

#endif // ROVER_CONFIG_H
