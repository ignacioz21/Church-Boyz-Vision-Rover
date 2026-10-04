#ifndef ROVER_STRATEGY_H
#define ROVER_STRATEGY_H

#include "types.h"

// Lógica de competencia: corre A BORDO desde que arranca la ronda. Toma las
// tareas del plan (plan.h) y lleva cada cubo a su zona, recalculando la ruta
// en cada ciclo con la telemetría del momento.

void strategyReset();

// Se llama en cada vuelta de loop() con telemetría fresca. NO bloquea.
void strategyStep(const TelemetrySnapshot &snap);

// Para el monitor del cerebro
const char* strategyStateName();
CubeColor strategyTarget();     // Cubo en curso (COLOR_UNKNOWN si ninguno)
Point strategyGoal();           // Punto al que se dirige ahora

#endif // ROVER_STRATEGY_H
