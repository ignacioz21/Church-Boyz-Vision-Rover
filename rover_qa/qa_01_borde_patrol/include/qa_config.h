#ifndef QA_CONFIG_H
#define QA_CONFIG_H

#include "config.h"

// =============================================================================
// PARÁMETROS DEL PROGRAMA QA DE PATRULLAJE DE BORDES
// =============================================================================

// Holgura de seguridad respecto a los bordes de la cancha (celdas)
// 1 celda = 20 mm. 6.0 celdas = 120 mm (12.0 cm)
// Con un chasis de ~14 cm de ancho (7 cm de radio), deja 5 cm libres a la orilla
#define EDGE_CLEARANCE_CELLS      6.0f

// Tolerancia de distancia para considerar alcanzada una esquina o el borde (celdas)
#define CORNER_ARRIVE_TOL_CELLS   2.8f

// Tolerancia angular para considerar completado el giro de 90 grados (grados)
#define TURN_TOL_DEG              3.0f

// Velocidad de crucero controlada para la prueba (potencia [-1.0 a 1.0])
#define QA_SPEED_CRUISE           0.26f

// Velocidad lenta de frenado y aproximación final al borde
#define QA_SPEED_SLOW             0.18f

// Potencia para giro sobre su propio eje (suficiente para superar rozamiento de llantas)
#define QA_SPEED_PIVOT            0.40f

// Umbral de sensores infrarrojos para detección de borde de mesa / abismo (0-4095)
#define IR_CLIFF_THRESHOLD        2000

// Tiempo de espera/pausa de estabilización en cada esquina (ms)
#define CORNER_PAUSE_MS           600

// Cantidad de bordes a patrullar (4 = 1 vuelta completa al perímetro)
#define MAX_EDGES_TO_PATROL       4

// Pin del botón físico BOOT en la IdeaBoard para disparo manual
#define PIN_BOOT_BUTTON           0

#endif // QA_CONFIG_H
