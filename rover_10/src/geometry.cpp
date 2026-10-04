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

// --- Huella real del robot ----------------------------------------------------

// Punto del robot (x hacia el frente, y hacia un costado, desde el eje) -> cancha.
// La huella es simétrica, así que no importa cuál costado.
static Point bodyPoint(Point axle, float theta, float x, float y) {
    float c = cosf(theta / DEG), s = sinf(theta / DEG);
    Point p = { axle.col + x * c + y * s, axle.row - x * s + y * c };
    return p;
}

bool footprintInField(Point axle, float theta, const TelemetrySnapshot &snap, float line_margin) {
    const float tol = LINE_TOLERANCE - line_margin;
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

bool footprintHits(Point axle, float theta, Point obj, float radius) {
    // El objeto en el marco del robot
    float c = cosf(theta / DEG), s = sinf(theta / DEG);
    float dx = obj.col - axle.col, dy = obj.row - axle.row;
    float x = dx * c - dy * s;          // Hacia el frente
    float y = dx * s + dy * c;          // Hacia un costado

    // Distancia al rectángulo del cuerpo
    float bx = constrain(x, -FP_REAR, FP_FRONT), by = constrain(y, -FP_HALF_WIDTH, FP_HALF_WIDTH);
    if (hypotf(x - bx, y - by) < radius) return true;
    // Distancia a la pinza de ese lado (un segmento)
    float px = constrain(x, FP_FRONT, FP_PRONG_TIP), py = y > 0 ? FP_PRONG_SIDE : -FP_PRONG_SIDE;
    return hypotf(x - px, y - py) < radius;
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
        // El compañero como un círculo que contiene su cuerpo (sin contar sus pinzas)
        Point peer = { snap.peer.col, snap.peer.row };
        if (footprintHits(axle, theta, peer, 5.0f)) return false;
    }
    return true;
}

static const float PIVOT_LINE_MARGIN = 0.15f;

bool pivotClear(Point axle, float theta_from, float theta_to, int dir,
                const TelemetrySnapshot &snap, CubeColor skip) {
    // Ángulo a recorrer en ese sentido, en [0, 360)
    float sweep = fmodf((theta_to - theta_from) * dir + 720.0f, 360.0f);
    // Se empieza un paso más allá: la pose actual es la que es, lo que se decide es el giro
    for (float a = 10.0f; a < sweep; a += 10.0f) {
        if (!poseClear(axle, theta_from + dir * a, snap, skip, 0.4f, PIVOT_LINE_MARGIN)) return false;
    }
    return poseClear(axle, theta_to, snap, skip, 0.4f, PIVOT_LINE_MARGIN);
}
