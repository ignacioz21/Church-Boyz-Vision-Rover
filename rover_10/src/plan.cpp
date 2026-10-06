#include "../include/plan.h"
#include "../include/config.h"
#include "../include/geometry.h"

static Plan loaded;         // El de la PC (id > 0)
static Plan fallback;       // El calculado a bordo (id = 0)
static bool fallback_ready = false;
static TaskRoute routes[NUM_COLORS];     // Rutas de la PC, por color de cubo
static bool depart_together = false;

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

// Lee el siguiente entero de una lista separada por comas; 'pos' avanza. false si no hay.
static bool nextInt(const String &s, int &pos, int &out) {
    if (pos >= (int)s.length()) return false;
    int end = s.indexOf(',', pos);
    if (end < 0) end = s.length();
    if (end == pos) return false;
    out = s.substring(pos, end).toInt();
    pos = end + 1;
    return true;
}

static bool nextPoint(const String &s, int &pos, Point &out) {
    int x, y;
    if (!nextInt(s, pos, x) || !nextInt(s, pos, y)) return false;
    out.col = x / 10.0f;
    out.row = y / 10.0f;
    return true;
}

// Una sección "<color>,cx,cy,fx,fy,sx,sy,dir,n,(x,y,r)*n,drop,dx,dy,ddir,m,(x,y)*m"
static bool parseRoute(const String &sec, CubeColor &cube, TaskRoute &r) {
    if (sec.length() < 3 || sec.charAt(1) != ',') return false;
    cube = colorFromChar(sec.charAt(0));
    if (cube == COLOR_UNKNOWN) return false;
    int pos = 2, v;
    if (!nextPoint(sec, pos, r.cube_at) || !nextPoint(sec, pos, r.from) || !nextPoint(sec, pos, r.stage)) return false;
    if (!nextInt(sec, pos, r.dir) || r.dir < 0 || r.dir >= NAV_DIRS) return false;
    if (!nextInt(sec, pos, r.n_go) || r.n_go < 0 || r.n_go > NAV_MAX_LEGS) return false;
    for (int i = 0; i < r.n_go; i++) {
        if (!nextPoint(sec, pos, r.go[i].to) || !nextInt(sec, pos, v)) return false;
        r.go[i].reverse = v != 0;
    }
    if (!nextInt(sec, pos, v)) return false;
    r.has_drop = v != 0;
    if (!nextPoint(sec, pos, r.drop_stage)) return false;
    if (!nextInt(sec, pos, r.drop_dir) || r.drop_dir < 0 || r.drop_dir >= NAV_DIRS) return false;
    if (!nextInt(sec, pos, r.n_carry) || r.n_carry < 0 || r.n_carry > NAV_MAX_LEGS) return false;
    for (int i = 0; i < r.n_carry; i++) {
        if (!nextPoint(sec, pos, r.carry[i].to)) return false;
        r.carry[i].reverse = false;         // Con el cubo en las pinzas no se retrocede
    }
    r.valid = true;
    return true;
}

bool planLoad(const String &msg) {
    // Con secciones: la última es la suma de control de todo lo anterior
    int bar = msg.indexOf('|');
    String head = bar < 0 ? msg : msg.substring(0, bar);
    static TaskRoute parsed[NUM_COLORS];
    for (int c = 0; c < NUM_COLORS; c++) parsed[c].valid = false;
    if (bar >= 0) {
        int k = msg.lastIndexOf("|K");
        if (k < 0) return false;
        uint32_t sum = 0;
        for (int i = 0; i < k; i++) sum += (uint8_t)msg.charAt(i);
        if ((int)(sum & 0xFFFF) != msg.substring(k + 2).toInt()) return false;
        int at = bar + 1;
        while (at < k) {
            int end = msg.indexOf('|', at);
            if (end < 0 || end > k) end = k;
            CubeColor cube;
            static TaskRoute r;
            if (!parseRoute(msg.substring(at, end), cube, r)) return false;
            parsed[cube] = r;
            at = end + 1;
        }
    }

    int a = head.indexOf(',');
    int b = head.indexOf(',', a + 1);
    if (a < 0 || b < 0) return false;
    Plan p;
    p.id = head.substring(a + 1, b).toInt();
    if (p.id <= 0) return false;
    if (!parseTasks(head, ROVER_ID, p.mine, p.n_mine)) return false;
    if (!parseTasks(head, ROVER_PEER_ID, p.peer, p.n_peer)) return false;
    loaded = p;
    depart_together = head.indexOf(",x=1") > 0;
    for (int c = 0; c < NUM_COLORS; c++) routes[c] = parsed[c];
    return true;
}

const TaskRoute* planRoute(CubeColor c) {
    if (c < 0 || c >= NUM_COLORS || loaded.id <= 0 || !routes[c].valid) return nullptr;
    return &routes[c];
}

void planDropRoute(CubeColor c) {
    if (c >= 0 && c < NUM_COLORS) routes[c].valid = false;
}

int planRouteCount() {
    int n = 0;
    for (int c = 0; c < NUM_COLORS; c++) n += loaded.id > 0 && routes[c].valid;
    return n;
}

bool planDepartTogether() { return loaded.id > 0 && depart_together; }

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
