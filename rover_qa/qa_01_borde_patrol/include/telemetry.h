#ifndef ROVER_TELEMETRY_H
#define ROVER_TELEMETRY_H

#include <Arduino.h>
#include "types.h"

// Inicializa Wi-Fi y lanza la tarea FreeRTOS en Core 0
void telemetryInit();

// Obtiene de forma thread-safe una copia del estado más reciente recibido
bool telemetryGetSnapshot(TelemetrySnapshot &out_snapshot);

// Retorna true si la telemetría es reciente (edad < 500 ms) y la conexión está activa
bool isTelemetryFresh();

// Configuración dinámica del ID del rover
void telemetrySetRoverId(int my_id, int peer_id);
int telemetryGetRoverId();
int telemetryGetPeerRoverId();

// Envía un mensaje formateado al Serial local y por TCP al Servidor de Visión
void telemetrySendLog(const char* format, ...);

#endif // ROVER_TELEMETRY_H
