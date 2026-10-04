#ifndef ROVER_TYPES_H
#define ROVER_TYPES_H

#include <Arduino.h>

// Coordenadas en CELDAS de la cancha (CONTRATO.md v2). Origen = centro del
// marcador 0, col crece a la derecha, row crece hacia abajo.
// theta en grados, 0 = derecha, antihorario.

enum CompetitionPhase { PHASE_IDLE = 0, PHASE_READY, PHASE_RUNNING, PHASE_FINISHED };

// Índice fijo por color en los arreglos de cubos y depósitos
enum CubeColor { COLOR_UNKNOWN = -1, COLOR_RED = 0, COLOR_GREEN = 1, COLOR_BLUE = 2 };
#define NUM_COLORS 3

struct Point {
    float col = 0.0f;
    float row = 0.0f;
};

struct RoverPose {
    int id = 0;
    float col = 0.0f;
    float row = 0.0f;
    float theta = 0.0f;
    uint32_t age_ms = 0;
    bool detected = false;      // Vino en el último mensaje
};

struct Cube {
    float col = 0.0f;
    float row = 0.0f;
    uint32_t age_ms = 0;        // Crece si está tapado (sigue en la lista)
    bool detected = false;      // Vino en el último mensaje
};

// Estado del mundo según el último mensaje válido de la visión
struct TelemetrySnapshot {
    bool is_connected = false;          // Socket TCP activo
    bool is_valid = false;              // Ya se recibió al menos un mensaje válido
    uint32_t last_packet_time_ms = 0;   // millis() local de recepción

    uint32_t seq = 0;
    CompetitionPhase phase = PHASE_IDLE;
    uint32_t remaining_ms = 0;          // clock.remaining_ms (reloj oficial)

    float grid_cols = 43.0f;
    float grid_rows = 43.0f;
    float cell_mm = 20.0f;

    RoverPose me;
    RoverPose peer;
    Cube cubes[NUM_COLORS];
    Point depots[NUM_COLORS];           // Centro de cada zona
    Point start;
    float depot_length = 10.0f;         // Paralelo al borde
    float depot_depth = 7.5f;           // Hacia adentro
    float cube_side = 3.0f;
};

#endif // ROVER_TYPES_H
