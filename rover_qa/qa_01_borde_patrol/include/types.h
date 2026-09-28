#ifndef ROVER_TYPES_H
#define ROVER_TYPES_H

#include <Arduino.h>

// Colores de los cubos y depósitos
enum CubeColor {
    COLOR_RED = 0,
    COLOR_GREEN = 1,
    COLOR_BLUE = 2,
    COLOR_UNKNOWN = 3
};

// Estados oficiales de la competencia
enum CompetitionPhase {
    PHASE_UNKNOWN = 0,
    PHASE_IDLE,
    PHASE_READY,
    PHASE_RUNNING,
    PHASE_FINISHED
};

// Estados de la FSM del Rover
enum RoverFsmState {
    STATE_BOOT_INIT = 0,
    STATE_HEALTH_CHECK,
    STATE_IDLE,
    STATE_READY_PLAN,
    STATE_NAV_PREAPPROACH,
    STATE_PUSH_TO_DEPOT,
    STATE_SAFE_RETREAT,
    STATE_CHECK_NEXT_CUBE,
    STATE_MISSION_FINISHED,
    STATE_FAILSAFE_LAG,
    STATE_UNSTUCK_REVERSE
};

// Punto en 2D (en unidades de celdas)
struct Vector2D {
    float x;
    float y;

    Vector2D() : x(0.0f), y(0.0f) {}
    Vector2D(float _x, float _y) : x(_x), y(_y) {}
};

// Postura y orientación de un rover
struct RoverPose {
    int id;
    float x;            // Columna (col) en celdas
    float y;            // Fila (row) en celdas
    float heading;      // Orientación en grados (-180 a 180)
    uint32_t age_ms;    // Antigüedad del dato en ms
    bool detected;

    RoverPose() : id(0), x(0.0f), y(0.0f), heading(0.0f), age_ms(9999), detected(false) {}
};

// Información de un cubo
struct CubeData {
    CubeColor color;
    float x;            // Columna (col)
    float y;            // Fila (row)
    uint32_t age_ms;
    bool detected;
    bool in_depot;      // Determinado por cubo_en_su_zona

    CubeData() : color(COLOR_UNKNOWN), x(0.0f), y(0.0f), age_ms(9999), detected(false), in_depot(false) {}
};

// Información de una zona de acopio
struct DepotZone {
    CubeColor color;
    float x;            // Centro col
    float y;            // Centro row

    DepotZone() : color(COLOR_UNKNOWN), x(0.0f), y(0.0f) {}
    DepotZone(CubeColor c, float _x, float _y) : color(c), x(_x), y(_y) {}
};

// Snapshot global de telemetría recibido del servidor de visión
struct TelemetrySnapshot {
    RoverPose my_rover;
    RoverPose peer_rover;
    CubeData cubes[3];          // Red, Green, Blue
    DepotZone depots[3];        // Red, Green, Blue
    float grid_cols;            // Ancho dinámico de la cancha en celdas
    float grid_rows;            // Alto dinámico de la cancha en celdas
    float cell_mm;              // Tamaño de celda en mm (20.0)
    CompetitionPhase phase;
    uint32_t last_packet_time_ms;
    bool is_connected;
    bool is_valid;

    TelemetrySnapshot() : grid_cols(43.0f), grid_rows(43.0f), cell_mm(20.0f), phase(PHASE_UNKNOWN), last_packet_time_ms(0), is_connected(false), is_valid(false) {}
};

#endif // ROVER_TYPES_H
