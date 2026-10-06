#ifndef ROVER_PLAN_H
#define ROVER_PLAN_H

#include "types.h"

// El plan dice qué cubos lleva cada rover y en qué orden. Lo calcula la PC y lo
// carga ANTES de READY (reglamento 6.2.5, 8.6.6). Las rutas no viajan en el
// plan: cada rover las calcula a bordo con la telemetría del momento.
struct Plan {
    int id = 0;                         // 0 = sin plan de la PC
    CubeColor mine[NUM_COLORS];
    int n_mine = 0;
    CubeColor peer[NUM_COLORS];
    int n_peer = 0;
};

// Mensaje "P,<id>,10=<colores>,11=<colores>" con colores r/g/b (p. ej. "P,7,10=gb,11=r").
// Devuelve false si está mal formado (y no toca el plan vigente).
bool planLoad(const String &msg);

// Plan vigente. Si la PC no cargó ninguno, se arma uno por defecto con la
// telemetría: cada cubo al rover más cercano, máximo dos por rover.
const Plan& planGet(const TelemetrySnapshot &snap);

// ID del plan cargado por la PC (0 = ninguno). Es lo que se le confirma al cerebro.
int planLoadedId();

// Nueva ronda: el reparto por defecto se recalcula; el plan de la PC se conserva
void planNewRound();

#endif // ROVER_PLAN_H
