#ifndef ROVER_NAVIGATION_H
#define ROVER_NAVIGATION_H

#include <Arduino.h>
#include "types.h"

// Distancia euclidiana entre dos puntos en celdas
float euclideanDistance(Vector2D p1, Vector2D p2);

// Normaliza un ángulo al rango [-180.0, 180.0] grados
float normalizeAngleDeg(float angle_deg);

// Diferencia angular mínima con signo (-180 a 180) de 'current' hacia 'target'
float angleDifferenceDeg(float target_deg, float current_deg);

// Convierte vector diferencial de posición (dcol, drow) a ángulo antihorario [0, 360) deg según CONTRATO.md
float vectorToAngleDeg(float dcol, float drow);

// Calcula el punto de pre-aproximación ubicado detrás del cubo respecto al depósito
Vector2D calculatePreApproach(Vector2D cube_pos, Vector2D depot_pos, float offset_cells);

// Comprueba si una posición está en zona de riesgo perimetral (< margin_cells)
bool isNearBorder(Vector2D pos, float margin_cells);

// Calcula un punto intermedio seguro alejado del borde para aproximación diagonal
Vector2D calculateSafeBorderWaypoint(Vector2D cube_pos);

// Evalúa si el cubo está válidamente entregado dentro del depósito según CONTRATO.md
bool isCubeInDepot(Vector2D cube_pos, Vector2D depot_pos, float max_dist_cells);

// Ley de control proporcional para calcular las velocidades de motor izquierdo y derecho
void calculateSteeringMotors(float current_heading_deg, float target_heading_deg, float base_speed, float &out_left, float &out_right);

#endif // ROVER_NAVIGATION_H
