#ifndef ROVER_STRATEGY_H
#define ROVER_STRATEGY_H

#include "types.h"
#include "nav.h"

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
// Lo que piensa recorrer, en orden (para el monitor): ruta en curso, cubo y destino.
int strategyRoute(Point *out, int max);
// Planificar puede tardar un par de segundos. Esta función se llama cada tanto mientras
// tanto, para que el programa principal siga reportando (y no parezca apagado).
void strategySetKeepAlive(void (*fn)());

// --- Planificar por adelantado (fase CEREBRO) ----------------------------------------
// Cómo atender un cubo desde una pose dada: la captura, la ruta de ida, la entrega y la
// ruta con el cubo. Es el MISMO cálculo que hace el rover al elegir tarea, pero sin
// moverse: lo usa la PC antes de READY (con este código compilado como biblioteca).
struct Preplan {
    Point cube_at, from;
    Point stage;                // Captura: punto de preparación y rumbo (dir * 45°)
    int dir = 0;
    NavLeg go[NAV_MAX_LEGS];
    int n_go = 0;
    bool has_drop = false;      // false = entrega directa: girar y empujar
    Point drop_stage;
    int drop_dir = 0;
    NavLeg carry[NAV_MAX_LEGS];
    int n_carry = 0;
    Pose end;                   // Dónde queda el rover después de soltar y retroceder
};
// false si no hay forma de tomar ese cubo y llevarlo a su zona con el mundo de 'snap'.
// No llamar con la estrategia en marcha: usa (y pisa) el planificador de rutas.
bool strategyPreplan(const TelemetrySnapshot &snap, const Pose &pose, CubeColor c, Preplan &out);

// --- Diagnóstico (monitor del cerebro y simulador) ---------------------------------
// Cuántas veces pasó cada cosa en la ronda en curso.
struct StrategyStats {
    uint16_t deliveries;        // Entregas verificadas
    uint16_t parks;             // Cubos apartados para destrabar otra entrega
    uint16_t helps;             // Cubos del compañero que tomó porque él no los estaba atendiendo
    uint16_t clears;            // Veces que se corrió para no estorbar
    uint16_t retouches;         // Cubos que quedaron al borde de la zona y se empujaron de nuevo al centro
    uint16_t reaims;            // Capturas en las que hubo que retroceder y volver a apuntar
    uint16_t no_progress;       // Pasó demasiado tiempo sin avanzar y descartó lo que intentaba
    uint16_t nav_fail;          // Sin ruta al ir a tomar un cubo
    uint16_t aim_timeout;       // No logró apuntar al cubo (sin lado libre para girar)
    uint16_t capture_timeout;   // No llegó a meter el cubo en las pinzas
    uint16_t no_drop_route;     // Con el cubo tomado, no había ruta hasta su destino
    uint16_t carry_fail;        // Sin ruta a mitad del transporte
    uint16_t cube_lost;         // El cubo se salió de las pinzas
    uint16_t drop_blocked;      // Tramo final tapado o demorado
    uint16_t verify_fail;       // Lo dejó, pero no quedó dentro de la zona
};
const StrategyStats& strategyStats();

// Por qué no se está llevando ese cubo (texto corto), según la última vez que se eligió tarea
const char* strategyWhy(CubeColor c);
// ¿Está parado asentando el cubo contra el frente (antes de girar con él)?
bool strategySeating();

#endif // ROVER_STRATEGY_H
