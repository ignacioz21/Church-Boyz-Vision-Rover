#include "../include/plan.h"
#include "../include/config.h"
#include "../include/geometry.h"

static Plan loaded;         // El de la PC (id > 0)
static Plan fallback;       // El calculado a bordo (id = 0)
static bool fallback_ready = false;

static CubeColor colorFromChar(char c) {
    switch (c) {
        case 'r': return COLOR_RED;
        case 'g': return COLOR_GREEN;
        case 'b': return COLOR_BLUE;
        default:  return COLOR_UNKNOWN;
    }
}

// Lee "<id>=<colores>" de 'msg' y llena la lista. La clave ausente deja la lista vacía.
static bool parseTasks(const String &msg, int rover_id, CubeColor *out, int &n) {
    n = 0;
    String key = "," + String(rover_id) + "=";
    int at = msg.indexOf(key);
    if (at < 0) return true;
    for (int i = at + key.length(); i < (int)msg.length() && msg.charAt(i) != ','; i++) {
        CubeColor c = colorFromChar(msg.charAt(i));
        if (c == COLOR_UNKNOWN || n >= NUM_COLORS) return false;
        out[n++] = c;
    }
    return true;
}

bool planLoad(const String &msg) {
    int a = msg.indexOf(',');
    int b = msg.indexOf(',', a + 1);
    if (a < 0 || b < 0) return false;
    Plan p;
    p.id = msg.substring(a + 1, b).toInt();
    if (p.id <= 0) return false;
    if (!parseTasks(msg, ROVER_ID, p.mine, p.n_mine)) return false;
    if (!parseTasks(msg, ROVER_PEER_ID, p.peer, p.n_peer)) return false;
    loaded = p;
    return true;
}

int planLoadedId() {
    return loaded.id;
}

void planNewRound() {
    fallback_ready = false;
}

// Los dos rovers corren esto con (casi) la misma telemetría y deben llegar al
// mismo reparto: por eso todo se expresa por ID de rover, no por "yo/el otro".
static void buildFallback(const TelemetrySnapshot &snap) {
    const RoverPose &low = (ROVER_ID < ROVER_PEER_ID) ? snap.me : snap.peer;
    const RoverPose &high = (ROVER_ID < ROVER_PEER_ID) ? snap.peer : snap.me;
    Point p_low = { low.col, low.row }, p_high = { high.col, high.row };

    CubeColor low_tasks[NUM_COLORS], high_tasks[NUM_COLORS];
    int n_low = 0, n_high = 0;
    for (int c = 0; c < NUM_COLORS; c++) {
        Point cube = { snap.cubes[c].col, snap.cubes[c].row };
        if (!snap.cubes[c].detected || !inField(cube, snap)) continue;
        // Sin compañero a la vista, todo es para el que sí está
        bool to_low = !high.detected ||
                      (low.detected && dist(p_low, cube) <= dist(p_high, cube));
        if (to_low && n_low >= 2 && high.detected) to_low = false;
        if (!to_low && n_high >= 2 && low.detected) to_low = true;
        if (to_low) low_tasks[n_low++] = (CubeColor)c;
        else high_tasks[n_high++] = (CubeColor)c;
    }

    bool i_am_low = ROVER_ID < ROVER_PEER_ID;
    fallback = Plan();
    fallback.n_mine = i_am_low ? n_low : n_high;
    fallback.n_peer = i_am_low ? n_high : n_low;
    for (int i = 0; i < fallback.n_mine; i++) fallback.mine[i] = i_am_low ? low_tasks[i] : high_tasks[i];
    for (int i = 0; i < fallback.n_peer; i++) fallback.peer[i] = i_am_low ? high_tasks[i] : low_tasks[i];
}

const Plan& planGet(const TelemetrySnapshot &snap) {
    if (loaded.id > 0) return loaded;
    // El reparto por defecto se fija una sola vez: recalcularlo en cada ciclo
    // haría que los rovers se intercambien cubos al moverse.
    if (!fallback_ready) {
        buildFallback(snap);
        fallback_ready = true;
    }
    return fallback;
}
