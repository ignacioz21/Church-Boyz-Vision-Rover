#include "../include/telemetry.h"
#include "../include/config.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include <stdarg.h>

static TelemetrySnapshot shared_snapshot;
static portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t logQueue = NULL;

static int current_rover_id = ROVER_ID;
static int current_peer_id = ROVER_PEER_ID;

void telemetrySetRoverId(int my_id, int peer_id) {
    portENTER_CRITICAL(&telemetryMux);
    current_rover_id = my_id;
    current_peer_id = peer_id;
    portEXIT_CRITICAL(&telemetryMux);
}

int telemetryGetRoverId() {
    return current_rover_id;
}

int telemetryGetPeerRoverId() {
    return current_peer_id;
}

static CubeColor parseColor(const char* name) {
    if (strcmp(name, "red") == 0) return COLOR_RED;
    if (strcmp(name, "green") == 0) return COLOR_GREEN;
    if (strcmp(name, "blue") == 0) return COLOR_BLUE;
    return COLOR_UNKNOWN;
}

static void parsePose(JsonObject r, int id, RoverPose &pose) {
    pose.id = id;
    pose.x = r["col"] | 0.0f;
    pose.y = r["row"] | 0.0f;
    pose.heading = r["theta"] | (r["heading"] | 0.0f);
    pose.age_ms = r["age_ms"] | 0;
    pose.detected = true;
}

// Parsea una línea NDJSON sobre 'work' (copia privada de la tarea). Se hace
// FUERA de la sección crítica; solo la copia final al estado compartido la usa.
static bool parseTelemetryLine(const char* line, size_t len, TelemetrySnapshot &work, int my_id, int peer_id) {
    JsonDocument doc;
    if (deserializeJson(doc, line, len)) return false;

    if (doc["grid"].is<JsonObject>()) {
        JsonObject g = doc["grid"];
        work.grid_cols = g["cols"] | 43.0f;
        work.grid_rows = g["rows"] | 43.0f;
        work.cell_mm = g["cell_mm"] | 20.0f;
    }

    const char* phase_str = doc["phase"] | "IDLE";
    if (strcmp(phase_str, "READY") == 0) work.phase = PHASE_READY;
    else if (strcmp(phase_str, "RUNNING") == 0) work.phase = PHASE_RUNNING;
    else if (strcmp(phase_str, "FINISHED") == 0) work.phase = PHASE_FINISHED;
    else work.phase = PHASE_IDLE;

    work.my_rover.detected = false;
    work.peer_rover.detected = false;
    for (JsonObject r : doc["rovers"].as<JsonArray>()) {
        int id = r["id"] | 0;
        if (id == my_id) parsePose(r, id, work.my_rover);
        else if (id == peer_id) parsePose(r, id, work.peer_rover);
    }

    // Un cubo que no viene en el mensaje deja de estar detectado (los ocultos
    // siguen llegando con su última posición y age_ms creciente).
    for (int i = 0; i < 3; i++) work.cubes[i].detected = false;
    for (JsonObject cb : doc["cubes"].as<JsonArray>()) {
        CubeColor cc = parseColor(cb["color"] | "");
        if (cc == COLOR_UNKNOWN) continue;
        work.cubes[cc].color = cc;
        work.cubes[cc].x = cb["col"] | 0.0f;
        work.cubes[cc].y = cb["row"] | 0.0f;
        work.cubes[cc].age_ms = cb["age_ms"] | 0;
        work.cubes[cc].detected = true;
    }

    for (JsonObject dp : doc["depots"].as<JsonArray>()) {
        CubeColor cc = parseColor(dp["color"] | "");
        if (cc == COLOR_UNKNOWN) continue;
        work.depots[cc].x = dp["col"] | 0.0f;
        work.depots[cc].y = dp["row"] | 0.0f;
    }
    return true;
}

static void setConnected(bool connected) {
    portENTER_CRITICAL(&telemetryMux);
    shared_snapshot.is_connected = connected;
    portEXIT_CRITICAL(&telemetryMux);
}

// Tarea FreeRTOS fijada a Core 0 para que la red no interrumpa el lazo de control en Core 1
static void telemetryTask(void *pvParameters) {
    WiFiClient client;
    static char chunk[512];
    static char line_buf[TELEMETRY_LINE_MAX];
    static char latest[TELEMETRY_LINE_MAX];
    static TelemetrySnapshot work;
    size_t line_len = 0;
    uint32_t last_line_ms = 0;

    Serial.println("[TELEMETRY] Tarea iniciada en Core 0");

    while (true) {
        // 1. Asegurar conexión Wi-Fi
        if (WiFi.status() != WL_CONNECTED) {
            setConnected(false);
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
            setConnected(false);
            client.stop();
            Serial.printf("[TELEMETRY] Conectando a %s:%d ...\n", VISION_HOST, VISION_PORT);
            IPAddress v_ip;
            bool ok = v_ip.fromString(VISION_HOST) ? client.connect(v_ip, VISION_PORT)
                                                   : client.connect(VISION_HOST, VISION_PORT);
            if (!ok) {
                vTaskDelay(pdMS_TO_TICKS(TCP_RETRY_INTERVAL_MS));
                continue;
            }
            client.setNoDelay(true);
            Serial.println("[TELEMETRY] Enlace TCP establecido con el servidor de visión");
            line_len = 0;
            last_line_ms = millis();
        }

        // 3. Vaciar todo lo recibido en bloques, quedándose solo con la ÚLTIMA
        //    línea completa: la cola atrasada se descarta (CONTRATO.md).
        size_t latest_len = 0;
        int avail;
        while ((avail = client.available()) > 0) {
            int n = client.read((uint8_t*)chunk, min(avail, (int)sizeof(chunk)));
            if (n <= 0) break;
            for (int i = 0; i < n; i++) {
                char c = chunk[i];
                if (c == '\n') {
                    if (line_len > 10) {
                        memcpy(latest, line_buf, line_len);
                        latest_len = line_len;
                    }
                    line_len = 0;
                } else if (line_len < sizeof(line_buf)) {
                    line_buf[line_len++] = c;
                } else {
                    line_len = 0; // Línea demasiado larga: se descarta
                }
            }
        }

        if (latest_len > 0) {
            int my_id, peer_id;
            portENTER_CRITICAL(&telemetryMux);
            my_id = current_rover_id;
            peer_id = current_peer_id;
            portEXIT_CRITICAL(&telemetryMux);

            if (parseTelemetryLine(latest, latest_len, work, my_id, peer_id)) {
                last_line_ms = millis();
                work.is_connected = true;
                work.is_valid = true;
                work.last_packet_time_ms = last_line_ms;
                portENTER_CRITICAL(&telemetryMux);
                shared_snapshot = work;
                portEXIT_CRITICAL(&telemetryMux);
            }
        }

        // 4. Enlace "vivo" pero sin datos: reconectar. La visión manda el estado
        //    actual al aceptar, así que no hay nada que recuperar.
        if (millis() - last_line_ms > TELEMETRY_STALL_MS) {
            Serial.printf("[TELEMETRY] %u ms sin telemetria valida. Reconectando...\n",
                          (unsigned)(millis() - last_line_ms));
            client.stop();
            setConnected(false);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
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

    // Tarea FreeRTOS en Core 0 con stack de 8192 bytes
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
        xQueueSend(logQueue, msg, 0); // No bloqueante
    }
}
