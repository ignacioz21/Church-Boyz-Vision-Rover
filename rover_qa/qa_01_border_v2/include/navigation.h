#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "types.h"
#include "qa_config.h"

// =============================================================================
// NAVEGACIÓN — Funciones de cálculo geométrico y control de dirección
// =============================================================================

// Imprime un mapa ASCII 2D del campo
void printMap(const TelemetrySnapshot &snap);

// Función de prueba: navega hacia el borde contando pasos con el sensor IR
void test_border();

// Función de patrullaje: bordea la cancha usando telemetría, girando 90 grados al llegar a los límites
void patrol_border();

// Función de caza de cubos: navega al cubo, confirma con sensores, y empuja al depot
void hunt_cube(CubeColor target_color);

#endif // NAVIGATION_H
