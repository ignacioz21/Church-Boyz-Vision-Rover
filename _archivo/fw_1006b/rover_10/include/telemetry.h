#ifndef ROVER_TELEMETRY_H
#define ROVER_TELEMETRY_H

#include <Arduino.h>
#include "types.h"

// Conecta Wi-Fi y lanza la tarea que lee la visión (Core 0)
void telemetryInit();

// Copia atómica del último estado. Devuelve snapshot.is_valid.
bool telemetryGetSnapshot(TelemetrySnapshot &out);

// true si hay enlace, el rover está visto y su pose tiene < TELEMETRY_TIMEOUT_MS
bool isTelemetryFresh();

#endif // ROVER_TELEMETRY_H
