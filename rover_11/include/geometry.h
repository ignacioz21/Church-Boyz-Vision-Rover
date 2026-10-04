#ifndef ROVER_GEOMETRY_H
#define ROVER_GEOMETRY_H

#include "types.h"

// Misma geometría que cerebro/planner.py: si se cambia una, cambiar la otra.

float wrapDeg(float a);                         // -> (-180, 180]
float dist(Point a, Point b);
float headingTo(Point from, Point to);          // grados [0, 360), 0 = derecha, antihorario
Point advance(Point p, float theta_deg, float d);

Point clampToField(Point p, const TelemetrySnapshot &snap, float margin);
bool inField(Point p, const TelemetrySnapshot &snap);

// Cuánto le falta al cubo para estar COMPLETAMENTE dentro de su zona (celdas).
// 0 = entregado. Regla oficial: CONTRATO.md §3 "Cuándo un cubo está en su zona".
float cubeExcess(Point cube, CubeColor color, const TelemetrySnapshot &snap);

// Punto detrás del cubo sobre la recta zona->cubo, a 'd' del centro del cubo
Point approachPoint(Point cube, Point depot, float d);

// Si el tramo a->b pasa a menos de 'radius' del obstáculo, devuelve true y un
// punto de desvío por el lado más corto.
bool detourAround(Point a, Point b, Point obstacle, float radius, Point &via);

// Distancia de un punto al segmento a-b
float pointToSegment(Point p, Point a, Point b);

// El compañero como obstáculo: un círculo que contiene su cuerpo Y sus pinzas,
// centrado un poco por delante de su marcador.
Point peerCenter(const TelemetrySnapshot &snap);
#define PEER_BODY_RADIUS 6.3f

// --- Huella real del robot (cuerpo + pinzas) --------------------------------
// 'axle' es el eje de las ruedas y 'theta' el rumbo. Las medidas salen de config.h.

// ¿Toda la huella queda dentro de las líneas de la cancha (con LINE_TOLERANCE)?
// 'line_margin' = cuánto de esa tolerancia se deja sin usar, como holgura.
bool footprintInField(Point axle, float theta, const TelemetrySnapshot &snap, float line_margin = 0.0f);

// ¿La huella toca un objeto redondo de radio 'radius' centrado en 'obj'?
bool footprintHits(Point axle, float theta, Point obj, float radius);

// ¿En esa pose el robot está dentro de las líneas y no toca ningún cubo ni al
// compañero? 'skip' = cubo que se ignora (el que va en las pinzas, o ninguno).
// 'cube_margin' = holgura extra alrededor de cada cubo.
bool poseClear(Point axle, float theta, const TelemetrySnapshot &snap, CubeColor skip,
               float cube_margin = 0.4f, float line_margin = 0.0f);

// ¿Se puede pivotar de theta_from a theta_to en ese sentido (+1 antihorario,
// -1 horario) sin que la huella toque nada ni salga de las líneas?
// Se comprueba con holgura en las líneas ('line_margin'): un pivote real nunca sale
// exacto. PIVOT_TIGHT es lo mínimo, para cuando no hay forma de hacer sitio antes.
#define PIVOT_LINE_MARGIN 0.5f
#define PIVOT_TIGHT       0.1f
bool pivotClear(Point axle, float theta_from, float theta_to, int dir,
                const TelemetrySnapshot &snap, CubeColor skip, float line_margin = PIVOT_LINE_MARGIN);

#endif // ROVER_GEOMETRY_H
