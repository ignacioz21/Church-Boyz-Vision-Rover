#ifndef ROVER_COORD_H
#define ROVER_COORD_H

#include "types.h"
#include "motion.h"

// Coordinación entre los dos rovers: cada uno le cuenta al otro qué está haciendo,
// para que nadie tenga que adivinar. Lo que la cámara no muestra —qué cubo atiende,
// si lo lleva en las pinzas, adónde va— viaja en estos mensajes, junto con la pose
// que cada rover calcula para sí mismo (más al día que la de la cámara).
//
// Si los mensajes dejan de llegar, coordPeer().heard pasa a false y cada rover
// sigue solo con la cámara y el plan: nunca se queda esperando un mensaje.

enum Activity {
    ACT_IDLE = 0,       // Quieto, sin nada que pueda hacer ahora (espera)
    ACT_GOING,          // Yendo a tomar un cubo
    ACT_CAPTURING,      // Metiendo el cubo en las pinzas
    ACT_CARRYING,       // Llevando un cubo
    ACT_RELEASING,      // Soltando y retirándose
    ACT_CLEARING,       // Corriéndose para no estorbar
    ACT_DONE            // Terminó todo lo suyo
};

struct PeerState {
    bool heard = false;             // Hay un mensaje reciente del compañero
    Pose pose;                      // Su eje y rumbo, según él mismo
    Activity activity = ACT_IDLE;
    CubeColor cube = COLOR_UNKNOWN; // Cubo que atiende ahora
    bool carrying = false;          // Lo lleva en las pinzas
    bool blocked = false;           // No encuentra por dónde pasar (pide que le despejen)
    Point goal;                     // Adónde va con lo que está haciendo
    Point waypoint;                 // Punto al que se dirige en este momento
    uint32_t since_ms = 0;          // Desde cuándo está en esa actividad (reloj local)
};

void coordReset();

// Le cuenta al compañero mi estado (se limita sola a una vez cada PEER_PUBLISH_MS)
void coordPublish(const Pose &me, Activity activity, CubeColor cube, bool carrying, bool blocked,
                  Point goal, Point waypoint);

// Lo último que se sabe del compañero. Lee los mensajes pendientes.
const PeerState& coordPeer();

// ¿El compañero dice estar ocupándose de ese cubo?
bool coordPeerClaims(CubeColor c);

// ¿Quién pasa primero? El que lleva un cubo; si los dos o ninguno, el de menor ID.
bool coordIHavePriority(bool i_carry);

#endif // ROVER_COORD_H
