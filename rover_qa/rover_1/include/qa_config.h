#ifndef QA_CONFIG_H
#define QA_CONFIG_H

#include "config.h"

// =============================================================================
// PARÁMETROS DEL PROGRAMA QA DE PATRULLAJE DE BORDES (qa_01_border_v2)
// =============================================================================

// Holgura de seguridad respecto a los bordes de la cancha (celdas)
// 1 celda = 20 mm. 4.0 celdas = 80 mm (8.0 cm)
// Esto permite al rover acercarse lo suficiente para entrar a los depósitos R, G, B
#define EDGE_CLEARANCE_CELLS      4.0f

// Tolerancia de distancia para considerar alcanzada una esquina o el borde (celdas)
#define CORNER_ARRIVE_TOL_CELLS   3.5f

// Tolerancia angular para considerar completado el giro de 90 grados (grados)
#define TURN_TOL_DEG              4.5f

// Velocidad de crucero ultra-segura para la prueba (potencia [-1.0 a 1.0])
#define QA_SPEED_CRUISE           0.20f

// Velocidad lenta de gateo y frenado previo al borde
#define QA_SPEED_SLOW             0.14f

// Potencia para giro sobre su propio eje (suave, sin derrapes ni vibraciones)
#define QA_SPEED_PIVOT            0.32f

// Distancia al objetivo donde inicia la desaceleración suave (celdas = 20 cm)
#define QA_DECEL_DIST_CELLS       10.0f

// Umbral de sensores infrarrojos para detección de borde de mesa / abismo (0-4095)
#define IR_CLIFF_THRESHOLD        2000

// Tiempo de espera/pausa de estabilización en cada esquina (ms)
#define CORNER_PAUSE_MS           1000

// Timeout máximo de seguridad por tramo en movimiento (segundos)
#define QA_EDGE_TIMEOUT_SEC       15.0f

// Cantidad de bordes a patrullar (4 = 1 vuelta completa al perímetro)
#define MAX_EDGES_TO_PATROL       4

#endif // QA_CONFIG_H
