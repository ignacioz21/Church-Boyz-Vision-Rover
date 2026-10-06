#ifndef ROVER_PLAN_H
#define ROVER_PLAN_H

#include "types.h"
#include "nav.h"

// El plan dice qué cubos lleva cada rover y en qué orden. Lo calcula la PC y lo
// carga ANTES de READY (reglamento 6.2.5, 8.6.6). Puede traer además, para cada cubo
// de este rover, la forma de tomarlo y las rutas ya calculadas (fase CEREBRO): el
// rover las sigue sin planificar, y si alguna ya no sirve (el cubo se movió, algo se
// cruzó) la descarta y planifica él a bordo como siempre.
struct Plan {
    int id = 0;                         // 0 = sin plan de la PC
    CubeColor mine[NUM_COLORS];
    int n_mine = 0;
    CubeColor peer[NUM_COLORS];
    int n_peer = 0;
};

// Cómo atender un cubo, precalculado: captura, ruta de ida, entrega y ruta con el cubo
struct TaskRoute {
    bool valid;
    Point cube_at;                      // Dónde estaba el cubo al planificar
    Point from;                         // Desde dónde se calculó la ruta de ida (eje del rover)
    Point stage;                        // Captura: punto de preparación...
    int dir;                            // ...y rumbo (dir * 45°)
    NavLeg go[NAV_MAX_LEGS];            // Tramos hasta el punto de preparación
    int n_go;
    bool has_drop;                      // false = entrega directa (girar y empujar), sin punto de preparación
    Point drop_stage;
    int drop_dir;
    NavLeg carry[NAV_MAX_LEGS];         // Tramos con el cubo hasta el punto de entrega
    int n_carry;
};

// Mensaje "P,<id>,10=<colores>,11=<colores>" con colores r/g/b (p. ej. "P,7,10=gb,11=r").
// Un cubo puede ir en las dos listas (el "comodín"): lo toma el rover que quede libre
// primero, y se avisan entre ellos (coord.h). Opcional ",x=1": los dos salen a la vez.
// Con rutas, siguen secciones separadas por '|' (números en décimas de celda):
//   |<color>,cx,cy,fx,fy,sx,sy,dir,n,(x,y,r)*n,drop,dx,dy,ddir,m,(x,y)*m   una por cubo
//   |K<suma>                                                             control: suma de los bytes anteriores
// Devuelve false si está mal formado o cortado (y no toca el plan vigente).
bool planLoad(const String &msg);

// Ruta precalculada para ese cubo, o nullptr si no hay (o ya se descartó)
const TaskRoute* planRoute(CubeColor c);
// La ruta de ese cubo ya no sirve: no volver a ofrecerla en esta ronda
void planDropRoute(CubeColor c);
// Cuántas rutas hay cargadas (se le confirma al cerebro)
int planRouteCount();
// La PC comprobó que las primeras rutas no se cruzan: los dos pueden salir a la vez
bool planDepartTogether();

// Plan vigente. Si la PC no cargó ninguno, se arma uno por defecto con la
// telemetría: cada cubo al rover más cercano, máximo dos por rover.
const Plan& planGet(const TelemetrySnapshot &snap);

// ID del plan cargado por la PC (0 = ninguno). Es lo que se le confirma al cerebro.
int planLoadedId();

// Nueva ronda: el reparto por defecto se recalcula; el plan de la PC se conserva
void planNewRound();

#endif // ROVER_PLAN_H
