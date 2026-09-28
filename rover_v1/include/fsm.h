#ifndef ROVER_FSM_H
#define ROVER_FSM_H

#include <Arduino.h>
#include "types.h"

// Inicialización de la máquina de estados
void fsmInit();

// Actualización periódica de la FSM (ejecutada a 50 Hz en Core 1)
void fsmUpdate();

// Retorna el estado actual de la FSM
RoverFsmState fsmGetCurrentState();

// Retorna el nombre en texto del estado para logging por serial
const char* fsmGetStateName(RoverFsmState state);

#endif // ROVER_FSM_H
