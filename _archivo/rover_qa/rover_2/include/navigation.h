#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "types.h"
#include "qa_config.h"

// =============================================================================
// NAVEGACIÓN — Funciones de cálculo geométrico y control de dirección
// =============================================================================

// Imprime un mapa ASCII 2D del campo

// Función de prueba: navega hacia el borde contando pasos con el sensor IR
void test_border();

// Función de patrullaje: bordea la cancha usando telemetría, girando 90 grados al llegar a los límites
void patrol_border();

// Función de caza de cubos: navega al cubo, confirma con sensores, y empuja al depot
void go_home(float target_x, float target_y);
// Un paso (no bloqueante) del control "ir a (x, y)". Devuelve true al llegar.
bool goto_step(float target_x, float target_y);
bool hunt_cube(CubeColor target_color);
void hunt_multiple_cubes(int count);
void hunt_reto_mission();
void setPreApproachDistance(float blocks); 
float getPreApproachDistance();

#endif // NAVIGATION_H
