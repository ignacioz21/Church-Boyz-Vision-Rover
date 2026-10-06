#include "../include/telemetry.h"
#include "../include/config.h"
#include <WiFi.h>
#include <ArduinoJson.h>

static TelemetrySnapshot shared_snapshot;
static portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;

static CubeColor parseColor(const char* name) {
    if (strcmp(name, "red") == 0) return COLOR_RED;
    if (strcmp(name, "green") == 0) return COLOR_GREEN;
    if (strcmp(name, "blue") == 0) return COLOR_BLUE;
    return COLOR_UNKNOWN;
}

static void parsePose(JsonObject r, RoverPose &pose) {
    pose.id = r["id"] | 0;
    pose.col = r["col"] | 0.0f;
    pose.row = r["row"] | 0.0f;
    pose.theta = r["theta"] | 0.0f;
    pose.age_ms = r["age_ms"] | 0;
    pose.detected = true;
}

// Parsea una línea NDJSON sobre 'work' (copia privada de la tarea), FUERA de
// la sección crítica. Mensajes con versión desconocida se descartan.
static bool parseLine(const char* line, size_t len, TelemetrySnapshot &work) {
    JsonDocument doc;
    if (deserializeJson(doc, line, len)) return false;
    if ((doc["v"] | 0) != 2) return false;

    work.seq = doc["seq"] | 0;

    const char* phase = doc["phase"] | "IDLE";
    if (strcmp(phase, "READY") == 0) work.phase = PHASE_READY;
    else if (strcmp(phase, "RUNNING") == 0) work.phase = PHASE_RUNNING;
    else if (strcmp(phase, "FINISHED") == 0) work.phase = PHASE_FINISHED;
    else work.phase = PHASE_IDLE;
    work.remaining_ms = doc["clock"]["remaining_ms"] | 0;

    work.grid_cols = doc["grid"]["cols"] | 43.0f;
    work.grid_rows = doc["grid"]["rows"] | 43.0f;
    work.cell_mm = doc["grid"]["cell_mm"] | 20.0f;

    work.me.detected = false;
    work.peer.detected = false;
    for (JsonObject r : doc["rovers"].as<JsonArray>()) {
        int id = r["id"] | 0;
        if (id == ROVER_ID) parsePose(r, work.me);
        else if (id == ROVER_PEER_ID) parsePose(r, work.peer);
    }

    for (int i = 0; i < NUM_COLORS; i++) work.cubes[i].detected = false;
    for (JsonObject c : doc["cubes"].as<JsonArray>()) {
        CubeColor cc = parseColor(c["color"] | "");
        if (cc == COLOR_UNKNOWN) continue;
        work.cubes[cc].col = c["col"] | 0.0f;
        work.cubes[cc].row = c["row"] | 0.0f;
        work.cubes[cc].age_ms = c["age_ms"] | 0;
        work.cubes[cc].detected = true;
    }

    for (JsonObject d : doc["depots"].as<JsonArray>()) {
        CubeColor cc = parseColor(d["color"] | "");
        if (cc == COLOR_UNKNOWN) continue;
        work.depots[cc].col = d["col"] | 0.0f;
        work.depots[cc].row = d["row"] | 0.0f;
    }

    work.start.col = doc["start"]["col"] | 0.0f;
    work.start.row = doc["start"]["row"] | 0.0f;
    work.depot_length = doc["depot_size"]["length"] | 10.0f;
    work.depot_depth = doc["depot_size"]["depth"] | 7.5f;
    work.cube_side = doc["cube_side"] | 3.0f;
    return true;
}

static void setConnected(bool connected) {
    portENTER_CRITICAL(&telemetryMux);
    shared_snapshot.is_connected = connected;
    portEXIT_CRITICAL(&telemetryMux);
}

static void telemetryTask(void *) {
    WiFiClient client;
    static char chunk[512];
    static char line_buf[TELEMETRY_LINE_MAX];
    static char latest[TELEMETRY_LINE_MAX];
    static TelemetrySnapshot work;
    size_t line_len = 0;
    uint32_t last_line_ms = 0;

    while (true) {
        // 1. Wi-Fi
        if (WiFi.status() != WL_CONNECTED) {
            setConnected(false);
            Serial.printf("[TELEMETRY] Conectando a Wi-Fi '%s'...\n", WIFI_SSID);
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) vTaskDelay(pdMS_TO_TICKS(500));
            if (WiFi.status() != WL_CONNECTED) continue;
            Serial.printf("[TELEMETRY] Wi-Fi OK. IP: %s\n", WiFi.localIP().toString().c_str());
        }

        // 2. TCP a la visión
        if (!client.connected()) {
            setConnected(false);
            client.stop();
            if (!client.connect(VISION_HOST, VISION_PORT)) {
                Serial.printf("[TELEMETRY] Sin conexion a %s:%d\n", VISION_HOST, VISION_PORT);
                vTaskDelay(pdMS_TO_TICKS(TCP_RETRY_INTERVAL_MS));
                continue;
            }
            client.setNoDelay(true);
            Serial.println("[TELEMETRY] Conectado a la vision");
            line_len = 0;
            last_line_ms = millis();
        }

        // 3. Vaciar lo recibido en bloques y quedarse solo con la ÚLTIMA línea
        //    completa: nunca se decide con una cola de estados viejos.
        size_t latest_len = 0;
        int avail;
        while ((avail = client.available()) > 0) {
            int n = client.read((uint8_t*)chunk, min(avail, (int)sizeof(chunk)));
            if (n <= 0) break;
            for (int i = 0; i < n; i++) {
                char c = chunk[i];
                if (c == '\n') {
                    if (line_len > 0) {
                        memcpy(latest, line_buf, line_len);
                        latest_len = line_len;
                    }
                    line_len = 0;
                } else if (line_len < sizeof(line_buf)) {
                    line_buf[line_len++] = c;
                } else {
                    line_len = 0;  // Línea demasiado larga: se descarta
                }
            }
        }

        if (latest_len > 0 && parseLine(latest, latest_len, work)) {
            last_line_ms = millis();
            work.is_connected = true;
            work.is_valid = true;
            work.last_packet_time_ms = last_line_ms;
            portENTER_CRITICAL(&telemetryMux);
            shared_snapshot = work;
            portEXIT_CRITICAL(&telemetryMux);
        }

        // 4. Enlace abierto pero sin datos: reconectar (la visión manda el
        //    estado actual al aceptar, no hay nada que recuperar)
        if (millis() - last_line_ms > TELEMETRY_STALL_MS) {
            Serial.println("[TELEMETRY] Sin datos de la vision. Reconectando...");
            client.stop();
            setConnected(false);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void telemetryInit() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // Sin ahorro de energía: menos latencia en la recepción
    xTaskCreatePinnedToCore(telemetryTask, "telemetry", 8192, NULL, 1, NULL, 0);
}

bool telemetryGetSnapshot(TelemetrySnapshot &out) {
    portENTER_CRITICAL(&telemetryMux);
    out = shared_snapshot;
    portEXIT_CRITICAL(&telemetryMux);
    return out.is_valid;
}

bool isTelemetryFresh() {
    TelemetrySnapshot s;
    telemetryGetSnapshot(s);
    return s.is_connected && s.me.detected &&
           (millis() - s.last_packet_time_ms) < TELEMETRY_TIMEOUT_MS &&
           s.me.age_ms < TELEMETRY_TIMEOUT_MS;
}
