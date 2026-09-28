#ifndef ROVER_TYPES_H
#define ROVER_TYPES_H

#include <Arduino.h>

// Vector bidimensional cartesiano en coordenadas de cancha (celdas continuas)
struct Vector2D {
    float x;
    float y;

    Vector2D() : x(0.0f), y(0.0f) {}
    Vector2D(float _x, float _y) : x(_x), y(_y) {}
};

// Fases oficiales publicadas por el sistema de visión (CONTRATO.md)
enum CompetitionPhase {
    PHASE_IDLE = 0,
    PHASE_READY,
    PHASE_RUNNING,
    PHASE_FINISHED
};

// Colores de cubos y depósitos
enum CubeColor {
    COLOR_UNKNOWN = -1,
    COLOR_RED = 0,
    COLOR_GREEN = 1,
    COLOR_BLUE = 2
};

// Pose de un robot detectado por la visión
struct RoverPose {
    int id;                 // 10 u 11
    float x;                // Columna (0.0 a grid_cols)
    float y;                // Fila (0.0 a grid_rows)
    float heading;          // Rumbo antihorario [0.0, 360.0) grados
    uint32_t age_ms;        // Antigüedad de detección en ms
    bool detected;          // True si es visto en el frame actual

    RoverPose() : id(0), x(0.0f), y(0.0f), heading(0.0f), age_ms(9999), detected(false) {}
};

// Pose de un cubo en la cancha
struct CubePose {
    CubeColor color;
    float x;
    float y;
    uint32_t age_ms;
    bool detected;

    CubePose() : color(COLOR_UNKNOWN), x(0.0f), y(0.0f), age_ms(9999), detected(false) {}
};

// Instantánea completa y atómica del estado del juego
struct TelemetrySnapshot {
    bool is_connected;              // ¿Socket TCP activo con la PC?
    bool is_valid;                  // ¿Se ha recibido al menos un paquete válido?
    uint32_t last_packet_time_ms;   // Timestamp local de recepción

    // Dimensiones dinámicas de la cancha (Regla 17.4 del reglamento)
    float grid_cols;                // Columnas de la cuadrícula (ej. 43.0)
    float grid_rows;                // Filas de la cuadrícula (ej. 43.0)
    float cell_mm;                  // Milímetros por celda (20.0 mm)

    CompetitionPhase phase;         // Fase actual (IDLE, READY, RUNNING, FINISHED)

    RoverPose my_rover;             // Pose de NUESTRO rover
    RoverPose peer_rover;           // Pose del rover compañero
    CubePose cubes[3];              // Cubos (Rojo, Verde, Azul)
    Vector2D depots[3];             // Centros de zonas de depósito (R, G, B)

    TelemetrySnapshot() : 
        is_connected(false), 
        is_valid(false), 
        last_packet_time_ms(0),
        grid_cols(43.0f),
        grid_rows(43.0f),
        cell_mm(20.0f),
        phase(PHASE_IDLE) {}
};

// Estados globales estándar del firmware
enum RoverFsmState {
    STATE_BOOT_INIT = 0,
    STATE_HEALTH_CHECK,
    STATE_IDLE,
    STATE_READY_PLAN,
    STATE_NAV_PREAPPROACH,
    STATE_PUSH_TO_DEPOT,
    STATE_SAFE_RETREAT,
    STATE_MISSION_COMPLETE,
    STATE_OBSTACLE_AVOID,
    STATE_FAILSAFE_HOLD
};

#endif // ROVER_TYPES_H
