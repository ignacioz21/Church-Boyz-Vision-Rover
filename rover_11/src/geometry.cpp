#include "../include/geometry.h"
#include "../include/config.h"
#include <math.h>

static const float DEG = 180.0f / M_PI;

float wrapDeg(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

float dist(Point a, Point b) {
    return hypotf(b.col - a.col, b.row - a.row);
}

float headingTo(Point from, Point to) {
    // row crece hacia abajo y theta es antihorario: de ahí el signo de row
    float h = atan2f(-(to.row - from.row), to.col - from.col) * DEG;
    return h < 0 ? h + 360.0f : h;
}

Point advance(Point p, float theta_deg, float d) {
    Point out;
    out.col = p.col + d * cosf(theta_deg / DEG);
    out.row = p.row - d * sinf(theta_deg / DEG);
    return out;
}

Point clampToField(Point p, const TelemetrySnapshot &snap, float margin) {
    p.col = constrain(p.col, margin, snap.grid_cols - margin);
    p.row = constrain(p.row, margin, snap.grid_rows - margin);
    return p;
}

bool inField(Point p, const TelemetrySnapshot &snap) {
    return p.col >= 0 && p.col <= snap.grid_cols && p.row >= 0 && p.row <= snap.grid_rows;
}

float cubeExcess(Point cube, CubeColor color, const TelemetrySnapshot &snap) {
    Point depot = snap.depots[color];

    // La zona apoya su lado largo en el borde más cercano a su centro
    float to_horizontal = min(depot.row, snap.grid_rows - depot.row);   // arriba / abajo
    float to_vertical = min(depot.col, snap.grid_cols - depot.col);     // izquierda / derecha
    bool on_top_or_bottom = to_horizontal <= to_vertical;
    float semi_col = (on_top_or_bottom ? snap.depot_length : snap.depot_depth) / 2.0f;
    float semi_row = (on_top_or_bottom ? snap.depot_depth : snap.depot_length) / 2.0f;

    // Media diagonal: mete el cubo entero con cualquier rotación
    float margin = snap.cube_side * M_SQRT2 / 2.0f;
    float ex_col = max(0.0f, fabsf(cube.col - depot.col) - (semi_col - margin));
    float ex_row = max(0.0f, fabsf(cube.row - depot.row) - (semi_row - margin));
    return hypotf(ex_col, ex_row);
}

Point approachPoint(Point cube, Point depot, float d) {
    float len = dist(depot, cube);
    Point out = cube;
    if (len < 0.01f) return out;
    out.col += (cube.col - depot.col) / len * d;
    out.row += (cube.row - depot.row) / len * d;
    return out;
}

bool detourAround(Point a, Point b, Point obstacle, float radius, Point &via) {
    float seg = dist(a, b);
    if (seg < 0.01f) return false;
    float ux = (b.col - a.col) / seg, uy = (b.row - a.row) / seg;

    // Proyección del obstáculo sobre el tramo
    float t = (obstacle.col - a.col) * ux + (obstacle.row - a.row) * uy;
    if (t <= 0.0f || t >= seg) return false;            // No está entre a y b
    Point closest;
    closest.col = a.col + ux * t;
    closest.row = a.row + uy * t;
    float d = dist(closest, obstacle);
    if (d >= radius) return false;

    // Desvío: desde el obstáculo hacia el lado por donde ya pasaba el tramo
    float nx = closest.col - obstacle.col, ny = closest.row - obstacle.row;
    if (d < 0.05f) { nx = -uy; ny = ux; d = 1.0f; }     // Tramo justo por el centro
    via.col = obstacle.col + nx / d * radius * 1.25f;
    via.row = obstacle.row + ny / d * radius * 1.25f;
    return true;
}

float pointToSegment(Point p, Point a, Point b) {
    float seg2 = (b.col - a.col) * (b.col - a.col) + (b.row - a.row) * (b.row - a.row);
    float t = seg2 < 1e-6f ? 0.0f
            : ((p.col - a.col) * (b.col - a.col) + (p.row - a.row) * (b.row - a.row)) / seg2;
    t = constrain(t, 0.0f, 1.0f);
    Point c = { a.col + (b.col - a.col) * t, a.row + (b.row - a.row) * t };
    return dist(p, c);
}

Point peerCenter(const TelemetrySnapshot &snap) {
    // De la cola a la punta de las pinzas, visto desde el marcador
    float front = FP_PRONG_TIP - AXLE_OFFSET, rear = FP_REAR + AXLE_OFFSET;
    Point marker = { snap.peer.col, snap.peer.row };
    return advance(marker, snap.peer.theta, (front - rear) / 2.0f);
}

// --- Huella real del robot ----------------------------------------------------

// Punto del robot (x hacia el frente, y hacia un costado, desde el eje) -> cancha.
// La huella es simétrica, así que no importa cuál costado.
static Point bodyPoint(Point axle, float theta, float x, float y) {
    float c = cosf(theta / DEG), s = sinf(theta / DEG);
    Point p = { axle.col + x * c + y * s, axle.row - x * s + y * c };
    return p;
}

// Dónde está el robot AHORA (lo fija la estrategia en cada ciclo)
static Point self_axle;
static float self_theta = 0.0f;
static bool self_known = false;

void geometrySetSelf(Point axle, float theta) {
    self_axle = axle;
    self_theta = theta;
    self_known = true;
}

static const float FOOT_CORNERS[][2] = {
    { -FP_REAR, FP_HALF_WIDTH }, { -FP_REAR, -FP_HALF_WIDTH },
    { FP_FRONT, FP_HALF_WIDTH }, { FP_FRONT, -FP_HALF_WIDTH },
    { FP_PRONG_TIP, FP_PRONG_SIDE }, { FP_PRONG_TIP, -FP_PRONG_SIDE },
};

// Cuánto sobresale la huella de las líneas (0 si está toda adentro)
static float lineExcess(Point axle, float theta, const TelemetrySnapshot &snap) {
    float worst = 0.0f;
    for (const auto &k : FOOT_CORNERS) {
        Point p = bodyPoint(axle, theta, k[0], k[1]);
        worst = fmaxf(worst, fmaxf(fmaxf(-p.col, p.col - snap.grid_cols), fmaxf(-p.row, p.row - snap.grid_rows)));
    }
    return worst;
}

bool footprintInField(Point axle, float theta, const TelemetrySnapshot &snap, float line_margin) {
    float tol = LINE_TOLERANCE - line_margin;
    // Si YA estoy más afuera que lo permitido (me colocaron muy atrás en la salida),
    // exigir la tolerancia me dejaría sin ningún movimiento válido. Vale lo que hay
    // ahora: se puede ir a cualquier lado que no me saque más.
    if (self_known) {
        float now = lineExcess(self_axle, self_theta, snap);
        if (now > tol) tol = now + 0.05f;
    }
    // Basta con las esquinas: cola, frente del chasis y puntas de las pinzas
    static const float CORNERS[][2] = {
        { -FP_REAR, FP_HALF_WIDTH }, { -FP_REAR, -FP_HALF_WIDTH },
        { FP_FRONT, FP_HALF_WIDTH }, { FP_FRONT, -FP_HALF_WIDTH },
        { FP_PRONG_TIP, FP_PRONG_SIDE }, { FP_PRONG_TIP, -FP_PRONG_SIDE },
    };
    for (const auto &k : CORNERS) {
        Point p = bodyPoint(axle, theta, k[0], k[1]);
        if (p.col < -tol || p.col > snap.grid_cols + tol || p.row < -tol || p.row > snap.grid_rows + tol) return false;
    }
    return true;
}

float footprintDistance(Point axle, float theta, Point obj) {
    // El objeto en el marco del robot
    float c = cosf(theta / DEG), s = sinf(theta / DEG);
    float dx = obj.col - axle.col, dy = obj.row - axle.row;
    float x = dx * c - dy * s;          // Hacia el frente
    float y = dx * s + dy * c;          // Hacia un costado

    // Distancia al rectángulo del cuerpo
    float bx = constrain(x, -FP_REAR, FP_FRONT), by = constrain(y, -FP_HALF_WIDTH, FP_HALF_WIDTH);
    float body = hypotf(x - bx, y - by);
    // Distancia a la pinza de ese lado (un segmento)
    float px = constrain(x, FP_FRONT, FP_PRONG_TIP), py = y > 0 ? FP_PRONG_SIDE : -FP_PRONG_SIDE;
    return fminf(body, hypotf(x - px, y - py));
}

bool footprintHits(Point axle, float theta, Point obj, float radius) {
    return footprintDistance(axle, theta, obj) < radius;
}

// --- Compañero quieto: su forma real -----------------------------------------------
static bool peer_still = false;
void geometrySetPeerStill(bool still) { peer_still = still; }
bool geometryPeerStill() { return peer_still; }

// Puntos del contorno del robot (x al frente, y a un costado, desde el eje)
static const float OUTLINE[12][2] = {
    { -FP_REAR, FP_HALF_WIDTH }, { -FP_REAR, -FP_HALF_WIDTH }, { -FP_REAR, 0.0f },
    { FP_FRONT, FP_HALF_WIDTH }, { FP_FRONT, -FP_HALF_WIDTH }, { FP_FRONT, 0.0f },
    { (FP_FRONT - FP_REAR) / 2.0f, FP_HALF_WIDTH }, { (FP_FRONT - FP_REAR) / 2.0f, -FP_HALF_WIDTH },
    { FP_PRONG_TIP, FP_PRONG_SIDE }, { FP_PRONG_TIP, -FP_PRONG_SIDE },
    { (FP_FRONT + FP_PRONG_TIP) / 2.0f, FP_PRONG_SIDE }, { (FP_FRONT + FP_PRONG_TIP) / 2.0f, -FP_PRONG_SIDE },
};

// Distancia de un punto (en el marco del robot) a la huella
static inline float localDistance(float x, float y) {
    float bx = constrain(x, -FP_REAR, FP_FRONT), by = constrain(y, -FP_HALF_WIDTH, FP_HALF_WIDTH);
    float px = constrain(x, FP_FRONT, FP_PRONG_TIP), py = y > 0 ? FP_PRONG_SIDE : -FP_PRONG_SIDE;
    return fminf(hypotf(x - bx, y - by), hypotf(x - px, y - py));
}

void peerShape(const TelemetrySnapshot &snap, PeerShape &out) {
    Point marker = { snap.peer.col, snap.peer.row };
    out.axle = advance(marker, snap.peer.theta, -AXLE_OFFSET);
    out.c = cosf(snap.peer.theta / DEG);
    out.s = sinf(snap.peer.theta / DEG);
    for (int i = 0; i < 12; i++) {
        out.pts[i].col = out.axle.col + OUTLINE[i][0] * out.c + OUTLINE[i][1] * out.s;
        out.pts[i].row = out.axle.row - OUTLINE[i][0] * out.s + OUTLINE[i][1] * out.c;
    }
}

bool peerShapeHits(const PeerShape &peer, Point axle, float c, float s, float margin) {
    // Lejos: ni mirar (cada robot cabe en un círculo de radio ~8.4 desde su eje)
    if (hypotf(peer.axle.col - axle.col, peer.axle.row - axle.row) > 17.0f + margin) return false;
    for (int i = 0; i < 12; i++) {
        // Su contorno contra mi huella
        float dx = peer.pts[i].col - axle.col, dy = peer.pts[i].row - axle.row;
        if (localDistance(dx * c - dy * s, dx * s + dy * c) < margin) return true;
        // Mi contorno contra su huella
        float mc = axle.col + OUTLINE[i][0] * c + OUTLINE[i][1] * s, mr = axle.row - OUTLINE[i][0] * s + OUTLINE[i][1] * c;
        dx = mc - peer.axle.col; dy = mr - peer.axle.row;
        if (localDistance(dx * peer.c - dy * peer.s, dx * peer.s + dy * peer.c) < margin) return true;
    }
    return false;
}

// Distancia que hay que guardar con el compañero. Lo normal es PEER_BODY_RADIUS; pero
// si YA estoy más cerca que eso (la salida, con los dos rovers lado a lado), exigirla
// me dejaría sin ningún movimiento permitido. Entonces vale lo que hay ahora: se puede
// ir a cualquier lado que no me acerque más.
static float peerKeepOut(const TelemetrySnapshot &snap) {
    if (!self_known) return PEER_BODY_RADIUS;
    float now = footprintDistance(self_axle, self_theta, peerCenter(snap));
    return now < PEER_BODY_RADIUS ? fmaxf(now - 0.2f, PEER_TOUCH_RADIUS) : PEER_BODY_RADIUS;
}

bool poseClear(Point axle, float theta, const TelemetrySnapshot &snap, CubeColor skip,
               float cube_margin, float line_margin) {
    if (!footprintInField(axle, theta, snap, line_margin)) return false;
    // Cubo: se toma como un círculo que lo contiene con cualquier rotación, más holgura
    float cube_radius = snap.cube_side * M_SQRT2 / 2.0f + cube_margin;
    for (int c = 0; c < NUM_COLORS; c++) {
        Point cube = { snap.cubes[c].col, snap.cubes[c].row };
        if (c == skip || !snap.cubes[c].detected || !inField(cube, snap)) continue;
        if (footprintHits(axle, theta, cube, cube_radius)) return false;
    }
    if (snap.peer.detected) {
        if (peer_still) {
            PeerShape shape;
            peerShape(snap, shape);
            if (peerShapeHits(shape, axle, cosf(theta / DEG), sinf(theta / DEG), PEER_STILL_MARGIN)) return false;
        } else if (footprintHits(axle, theta, peerCenter(snap), peerKeepOut(snap))) {
            return false;
        }
    }
    return true;
}

bool pivotClear(Point axle, float theta_from, float theta_to, int dir,
                const TelemetrySnapshot &snap, CubeColor skip, float line_margin) {
    // Ángulo a recorrer en ese sentido, en [0, 360)
    float sweep = fmodf((theta_to - theta_from) * dir + 720.0f, 360.0f);
    // Se empieza un paso más allá: la pose actual es la que es, lo que se decide es el giro
    for (float a = 10.0f; a < sweep; a += 10.0f) {
        if (!poseClear(axle, theta_from + dir * a, snap, skip, 0.4f, line_margin)) return false;
    }
    return poseClear(axle, theta_to, snap, skip, 0.4f, line_margin);
}
