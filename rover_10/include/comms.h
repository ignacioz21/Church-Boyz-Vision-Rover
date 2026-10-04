#ifndef ROVER_COMMS_H
#define ROVER_COMMS_H

#include <Arduino.h>

// Canal UDP con el cerebro (PC). Solo para MONITOREO y PRUEBAS en desarrollo:
// durante un intento ninguna PC puede mandar comandos (reglamento 6.3, 11.2.7).
//
//   Cerebro -> rover  (ROVER_CMD_PORT):    "10:r"  o  "*:r"  (a todos)
//   Rover  -> cerebro (BRAIN_STATUS_PORT): una línea JSON con el estado

// Devuelve true si llegó un comando para ESTE rover; 'cmd' queda sin el prefijo "ID:"
bool commsPoll(String &cmd);

// Envía una línea al cerebro (no bloquea; si no hay Wi-Fi, se descarta)
void commsSend(const char *line);

// --- Canal entre los dos rovers (ROVER_PEER_PORT, difusión en la red) ---------------
// Permitido durante el intento: los rovers pueden intercambiar estados, tareas y
// movimientos para coordinarse (reglamento 7.2 a 7.5). La PC no interviene.

// Difunde una línea para el compañero (no bloquea; sin Wi-Fi se descarta)
void commsPeerSend(const char *line);

// Última línea recibida del compañero desde la llamada anterior; false si no hay nueva
bool commsPeerPoll(String &line);

#endif // ROVER_COMMS_H
