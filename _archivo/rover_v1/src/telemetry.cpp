#include "../include/telemetry.h"
#include "../include/config.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include <stdarg.h>

static TelemetrySnapshot shared_snapshot;
static portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t logQueue = NULL;

// Tarea FreeRTOS fijada a Core 0 para no interferir con el control de motores en Core 1
static void telemetryTask(void *pvParameters) {
    WiFiClient client;
    String line_buffer = "";
    line_buffer.reserve(1200);

    Serial.println("[TELEMETRY] Tarea iniciada en Core 0");

    while (true) {
        // 1. Asegurar conexión Wi-Fi
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("[TELEMETRY] Conectando a Wi-Fi...");
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            int attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < 20) {
                vTaskDelay(pdMS_TO_TICKS(500));
                attempts++;
            }
            if (WiFi.status() == WL_CONNECTED) {
                Serial.printf("[TELEMETRY] Wi-Fi conectado. IP: %s\n", WiFi.localIP().toString().c_str());
            } else {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }

        // 2. Conectar por TCP al Servidor de Visión (puerto 2026)
        if (!client.connected()) {
            Serial.printf("[TELEMETRY] Conectando a %s:%d ...\n", VISION_HOST, VISION_PORT);
            IPAddress v_ip;
            bool ok = false;
            if (v_ip.fromString(VISION_HOST)) {
                ok = client.connect(v_ip, VISION_PORT);
            } else {
                ok = client.connect(VISION_HOST, VISION_PORT);
            }

            if (!ok) {
                portENTER_CRITICAL(&telemetryMux);
                shared_snapshot.is_connected = false;
                portEXIT_CRITICAL(&telemetryMux);

                vTaskDelay(pdMS_TO_TICKS(TCP_RETRY_INTERVAL_MS));
                continue;
            }
            client.setNoDelay(true);
            Serial.println("[TELEMETRY] Enlace TCP establecido con el servidor de visión");
            line_buffer = "";
        }

        // Transmitir logs pendientes hacia el servidor de visión por TCP
        if (client.connected() && logQueue != NULL) {
            char out_log[160];
            while (xQueueReceive(logQueue, out_log, 0) == pdTRUE) {
                client.println(out_log);
            }
        }

        // 3. Lectura continua de stream NDJSON
        while (client.connected() && client.available() > 0) {
            char c = (char)client.read();
            if (c == '\n') {
                if (line_buffer.length() > 10) {
                    // Parseo del frame JSON
#if defined(ARDUINOJSON_VERSION_MAJOR) && (ARDUINOJSON_VERSION_MAJOR >= 7)
                    JsonDocument doc;
                    DeserializationError err = deserializeJson(doc, line_buffer);
#else
                    StaticJsonDocument<2048> doc;
                    DeserializationError err = deserializeJson(doc, line_buffer);
#endif

                    if (!err) {
                        portENTER_CRITICAL(&telemetryMux);

                        // Grid dinámico (ancho, alto y tamaño de celda)
                        if (doc.containsKey("grid")) {
                            JsonObject g = doc["grid"];
                            shared_snapshot.grid_cols = g["cols"] | 43.0f;
                            shared_snapshot.grid_rows = g["rows"] | 43.0f;
                            shared_snapshot.cell_mm = g["cell_mm"] | 20.0f;
                        }

                        // Fase oficial
                        const char* phase_str = doc["phase"] | "IDLE";
                        if (strcmp(phase_str, "READY") == 0) {
                            shared_snapshot.phase = PHASE_READY;
                        } else if (strcmp(phase_str, "RUNNING") == 0) {
                            shared_snapshot.phase = PHASE_RUNNING;
                        } else if (strcmp(phase_str, "FINISHED") == 0) {
                            shared_snapshot.phase = PHASE_FINISHED;
                        } else {
                            shared_snapshot.phase = PHASE_IDLE;
                        }

                        // Rovers
                        shared_snapshot.my_rover.detected = false;
                        shared_snapshot.peer_rover.detected = false;

                        JsonArray rovers = doc["rovers"].as<JsonArray>();
                        for (JsonObject r : rovers) {
                            int id = r["id"] | 0;
                            if (id == ROVER_ID) {
                                float h = 0.0f;
                                if (r.containsKey("theta")) h = r["theta"];
                                else if (r.containsKey("heading")) h = r["heading"];

                                shared_snapshot.my_rover.id = id;
                                shared_snapshot.my_rover.x = r["col"] | 0.0f;
                                shared_snapshot.my_rover.y = r["row"] | 0.0f;
                                shared_snapshot.my_rover.heading = h;
                                shared_snapshot.my_rover.age_ms = r["age_ms"] | 0;
                                shared_snapshot.my_rover.detected = true;
                            } else if (id == ROVER_PEER_ID) {
                                float h = 0.0f;
                                if (r.containsKey("theta")) h = r["theta"];
                                else if (r.containsKey("heading")) h = r["heading"];

                                shared_snapshot.peer_rover.id = id;
                                shared_snapshot.peer_rover.x = r["col"] | 0.0f;
                                shared_snapshot.peer_rover.y = r["row"] | 0.0f;
                                shared_snapshot.peer_rover.heading = h;
                                shared_snapshot.peer_rover.age_ms = r["age_ms"] | 0;
                                shared_snapshot.peer_rover.detected = true;
                            }
                        }

                        // Cubos
                        JsonArray cubes = doc["cubes"].as<JsonArray>();
                        for (JsonObject cb : cubes) {
                            const char* col_name = cb["color"] | "";
                            CubeColor cc = COLOR_UNKNOWN;
                            if (strcmp(col_name, "red") == 0) cc = COLOR_RED;
                            else if (strcmp(col_name, "green") == 0) cc = COLOR_GREEN;
                            else if (strcmp(col_name, "blue") == 0) cc = COLOR_BLUE;

                            if (cc != COLOR_UNKNOWN) {
                                shared_snapshot.cubes[cc].color = cc;
                                shared_snapshot.cubes[cc].x = cb["col"] | 0.0f;
                                shared_snapshot.cubes[cc].y = cb["row"] | 0.0f;
                                shared_snapshot.cubes[cc].age_ms = cb["age_ms"] | 0;
                                shared_snapshot.cubes[cc].detected = true;
                            }
                        }

                        // Depósitos
                        JsonArray depots = doc["depots"].as<JsonArray>();
                        for (JsonObject dp : depots) {
                            const char* col_name = dp["color"] | "";
                            CubeColor cc = COLOR_UNKNOWN;
                            if (strcmp(col_name, "red") == 0) cc = COLOR_RED;
                            else if (strcmp(col_name, "green") == 0) cc = COLOR_GREEN;
                            else if (strcmp(col_name, "blue") == 0) cc = COLOR_BLUE;

                            if (cc != COLOR_UNKNOWN) {
                                shared_snapshot.depots[cc].color = cc;
                                shared_snapshot.depots[cc].x = dp["col"] | 0.0f;
                                shared_snapshot.depots[cc].y = dp["row"] | 0.0f;
                            }
                        }

                        shared_snapshot.last_packet_time_ms = millis();
                        shared_snapshot.is_connected = true;
                        shared_snapshot.is_valid = true;

                        portEXIT_CRITICAL(&telemetryMux);
                    } else {
                        Serial.printf("[TELEMETRY] Error JSON: %s (len: %d)\n", err.c_str(), line_buffer.length());
                    }
                }
                line_buffer = "";
            } else {
                if (line_buffer.length() < 1200) {
                    line_buffer += c;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Cede tiempo al sistema operativo
    }
}

void telemetryInit() {
    portENTER_CRITICAL(&telemetryMux);
    shared_snapshot.is_connected = false;
    shared_snapshot.is_valid = false;
    shared_snapshot.grid_cols = 43.0f;
    shared_snapshot.grid_rows = 43.0f;
    shared_snapshot.cell_mm = 20.0f;
    shared_snapshot.phase = PHASE_IDLE;
    portEXIT_CRITICAL(&telemetryMux);

    if (logQueue == NULL) {
        logQueue = xQueueCreate(16, 160);
    }

    // Crea la tarea de recepción en Core 0 con stack de 8192 bytes
    xTaskCreatePinnedToCore(
        telemetryTask,
        "TelemetryTask",
        8192,
        NULL,
        1,
        NULL,
        0 // Core 0
    );
}

bool telemetryGetSnapshot(TelemetrySnapshot &out_snapshot) {
    portENTER_CRITICAL(&telemetryMux);
    out_snapshot = shared_snapshot;
    portEXIT_CRITICAL(&telemetryMux);
    return out_snapshot.is_valid;
}

bool isTelemetryFresh() {
    portENTER_CRITICAL(&telemetryMux);
    uint32_t dt = millis() - shared_snapshot.last_packet_time_ms;
    uint32_t age = shared_snapshot.my_rover.age_ms;
    bool conn = shared_snapshot.is_connected;
    bool det = shared_snapshot.my_rover.detected;
    portEXIT_CRITICAL(&telemetryMux);

    return (conn && det && dt < TELEMETRY_TIMEOUT_MS && age < TELEMETRY_TIMEOUT_MS);
}

void telemetrySendLog(const char* format, ...) {
    char msg[160];
    va_list args;
    va_start(args, format);
    vsnprintf(msg, sizeof(msg), format, args);
    va_end(args);

    Serial.println(msg);

    if (logQueue != NULL) {
        xQueueSend(logQueue, msg, 0); // Envío no bloqueante
    }
}
