#ifndef ROVER_NAV_H
#define ROVER_NAV_H

#include "types.h"
#include "motion.h"

// Navegación (con o sin cubo en las pinzas), planificada en posición Y rumbo.
//
// El robot es largo por delante (pinzas): no basta con saber por dónde pasa, hay
// que saber hacia dónde mira en cada punto y por qué lado gira. Por eso se arma
// una tabla "en este punto, con este rumbo, el robot cabe" (rejilla cada 2 celdas,
// 16 rumbos) y las rutas se componen solo de movimientos que la tabla da por
// libres: avanzar o retroceder recto a un punto vecino, o pivotar 45°.

enum NavStatus { NAV_RUNNING, NAV_ARRIVED, NAV_NO_PATH };

// Arma la tabla con lo que se ve ahora y calcula el costo de llegar desde 'from'
// a todos los puntos y rumbos. Tarda decenas de ms: llamar con el robot detenido.
// 'carried' = cubo que va en las pinzas (COLOR_UNKNOWN si ninguno): no cuenta como
// obstáculo, y con él no se retrocede (se quedaría atrás).
void navPrepare(const TelemetrySnapshot &snap, const Pose &from, CubeColor carried);

// Los rumbos de viaje son 8: dir * 45° (0 = derecha, antihorario).
#define NAV_DIRS 8

// Punto de la rejilla que queda 'steps' pasos DETRÁS de 'target' en la dirección
// 'dir' (o sea: desde ahí, avanzando con ese rumbo, se llega a 'target'). false si
// cae fuera de la cancha.
bool navStage(Point target, int dir, int steps, Point &stage);

// Costo (en celdas de recorrido equivalentes) de llegar al punto de rejilla más
// cercano a 'goal' quedando con rumbo dir * 45°, según el último navPrepare.
// Negativo si no se puede.
float navCost(Point goal, int dir);

// Un tramo recto de una ruta: hasta dónde, y si se recorre marcha atrás
struct NavLeg { Point to; bool reverse; };
#define NAV_MAX_LEGS 12

// Lleva al robot hasta ese punto para quedar con ese rumbo. Llamar en cada ciclo.
// Calcula la ruta entera una vez y la sigue tramo a tramo sin volver a calcular; solo
// replanifica si algo se cruzó, si se desvió o si el tramo siguiente ya no cabe.
NavStatus navGo(const Pose &pose, Point goal, int dir, const TelemetrySnapshot &snap, CubeColor carried);

// Carga una ruta ya hecha (la que calculó la PC antes de READY) hacia 'goal' con rumbo
// 'dir'. El siguiente navGo con esa misma meta la sigue en vez de calcular una; si el
// primer tramo no cabe desde donde está el robot, la descarta y planifica a bordo.
void navSetRoute(const NavLeg *legs, int n, Point goal, int dir);

// Copia la ruta calculada por el último navGo / navPlanRoute (todos sus tramos).
// Devuelve cuántos; 'complete' = llega hasta la meta (no quedó cortada por larga).
int navLegs(NavLeg *out, int max, bool *complete);

// Calcula la ruta de 'pose' a 'goal' sin mover el robot (para planificar por adelantado).
// false si no hay ruta. Si ya está en la meta devuelve true con cero tramos.
bool navPlanRoute(const Pose &pose, Point goal, int dir, const TelemetrySnapshot &snap, CubeColor carried);

// Tramo que el compañero está recorriendo ahora (de 'from' a 'to'). Mientras esté
// activo, las rutas lo evitan entero y no solo su posición actual. Lo usa el rover
// que NO tiene prioridad.
void navSetPeerLeg(bool active, Point from, Point to);

// Punto al que se dirige ahora (para el monitor y para ceder el paso)
Point navWaypoint();

// ¿Tiene una ruta en curso? (false mientras busca una y no la encuentra)
bool navHasRoute();

void navReset();

// Cuántas veces se calculó una ruta y cuántos ms se fueron en eso (desde el último reset
// de estas cuentas). Mientras calcula, el rover está quieto y sordo: es tiempo muerto.
void navPlanStats(uint32_t *count, uint32_t *ms);
void navPlanStatsReset();
// De esos cálculos, cuántos se hicieron EN VIAJE (parado a mitad de camino)
uint32_t navTravelPlans();

// Ruta en curso, resumida en sus esquinas (para el monitor). Devuelve cuántos puntos copió.
int navRoute(Point *out, int max);

#endif // ROVER_NAV_H
