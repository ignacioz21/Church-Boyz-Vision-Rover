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

#endif // ROVER_COMMS_H
