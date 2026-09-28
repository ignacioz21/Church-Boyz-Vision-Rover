#include "../include/navigation.h"
#include "../include/config.h"
#include <math.h>

float euclideanDistance(Vector2D p1, Vector2D p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return sqrtf(dx * dx + dy * dy);
}

float normalizeAngleDeg(float angle_deg) {
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

float angleDifferenceDeg(float target_deg, float current_deg) {
    float diff_rad = (target_deg - current_deg) * (M_PI / 180.0f);
    float angle_rad = atan2f(sinf(diff_rad), cosf(diff_rad));
    return angle_rad * (180.0f / M_PI);
}

Vector2D calculatePreApproach(Vector2D cube_pos, Vector2D depot_pos, float offset_cells) {
    float dx = depot_pos.x - cube_pos.x;
    float dy = depot_pos.y - cube_pos.y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist < 0.001f) {
        // En caso excepcional de superposición, offset hacia la izquierda
        return Vector2D(cube_pos.x - offset_cells, cube_pos.y);
    }

    float ux = dx / dist;
    float uy = dy / dist;

    // El punto de pre-aproximación está DETRÁS del cubo respecto al depósito
    return Vector2D(cube_pos.x - offset_cells * ux, cube_pos.y - offset_cells * uy);
}

bool isNearBorder(Vector2D pos, float margin_cells) {
    return (pos.x < margin_cells || 
            pos.x > (GRID_WIDTH_CELLS - margin_cells) ||
            pos.y < margin_cells || 
            pos.y > (GRID_HEIGHT_CELLS - margin_cells));
}

Vector2D calculateSafeBorderWaypoint(Vector2D cube_pos) {
    Vector2D safe_pt = cube_pos;

    // Empujar el waypoint hacia el centro seguro de la cancha
    if (cube_pos.x < EDGE_RISK_CELLS) {
        safe_pt.x += EDGE_SAFE_OFFSET_CELLS;
    } else if (cube_pos.x > (GRID_WIDTH_CELLS - EDGE_RISK_CELLS)) {
        safe_pt.x -= EDGE_SAFE_OFFSET_CELLS;
    }

    if (cube_pos.y < EDGE_RISK_CELLS) {
        safe_pt.y += EDGE_SAFE_OFFSET_CELLS;
    } else if (cube_pos.y > (GRID_HEIGHT_CELLS - EDGE_RISK_CELLS)) {
        safe_pt.y -= EDGE_SAFE_OFFSET_CELLS;
    }

    return safe_pt;
}

bool isCubeInDepot(Vector2D cube_pos, Vector2D depot_pos, float max_dist_cells) {
    return euclideanDistance(cube_pos, depot_pos) <= max_dist_cells;
}

float vectorToAngleDeg(float dcol, float drow) {
    // En CONTRATO.md, theta es antihorario, 0 = derecha (+col), y row crece hacia ABAJO.
    // Por tanto, Y_cartesiano = -drow, X_cartesiano = dcol:
    float angle_rad = atan2f(-drow, dcol);
    float angle_deg = angle_rad * (180.0f / M_PI);
    if (angle_deg < 0.0f) angle_deg += 360.0f;
    return angle_deg;
}

void calculateSteeringMotors(float current_heading_deg, float target_heading_deg, float base_speed, float &out_left, float &out_right) {
    float error_deg = angleDifferenceDeg(target_heading_deg, current_heading_deg);

    // 1. Zona muerta de ángulo
    if (fabsf(error_deg) < ANGLE_DEADBAND_DEG) {
        out_left = base_speed;
        out_right = base_speed;
        return;
    }

    // 2. Giro sobre su propio eje si el error es considerable
    // En sistema antihorario [0, 360):
    // error > 0 => objetivo hacia la IZQUIERDA (antihorario) => izquierda retrocede, derecha avanza
    // error < 0 => objetivo hacia la DERECHA (horario) => izquierda avanza, derecha retrocede
    if (fabsf(error_deg) > ANGLE_PIVOT_THRESH_DEG) {
        if (error_deg > 0.0f) {
            out_left = -SPEED_TURN_MAX;
            out_right = SPEED_TURN_MAX;
        } else {
            out_left = SPEED_TURN_MAX;
            out_right = -SPEED_TURN_MAX;
        }
        return;
    }

    // 3. Avance proporcional con corrección diferencial
    float turn_correction = error_deg * KP_STEERING;
    out_left = constrain(base_speed - turn_correction, -1.0f, 1.0f);
    out_right = constrain(base_speed + turn_correction, -1.0f, 1.0f);
}
