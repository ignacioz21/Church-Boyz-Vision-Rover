#ifndef ROVER_MOTION_H
#define ROVER_MOTION_H

#include "types.h"

// Pose estimada AHORA: la última de la cámara adelantada con las órdenes de
// motor dadas desde que se capturó (compensa la latencia del lazo).
//
// OJO: 'p' es el EJE DE LAS RUEDAS (el punto sobre el que gira el robot), no el
// marcador. Así un pivote no mueve el punto que se controla. El marcador está
// AXLE_OFFSET por delante.
struct Pose {
    Point p;
    float theta = 0.0f;
};
Pose motionPredict(const TelemetrySnapshot &snap);

// Giro trabado: el robot manda a pivotar y el giroscopio dice que no gira (un desnivel
// del piso, algo que lo frena). motionObstacle() queda en true un par de segundos
// después de cada detección; motionObstacleCount() es el total desde que encendió.
bool motionObstacle();
uint16_t motionObstacleCount();
float motionPivotTrim();            // Potencia extra de giro que está aplicando la regulación
float motionFineTrim();
Point motionObstaclePoint();        // Dónde fue el último (eje del robot)

// Toda orden de motor de la estrategia pasa por aquí: queda registrada para la predicción
void motionDrive(float left, float right);

// Potencia de crucero y tope de velocidad (celdas/s). Arrancan en los valores de
// config.h (los probados); se pueden cambiar antes de la ronda con el comando "V" para
// elegirlos en el lugar. Un reinicio vuelve a los de config.h.
void motionSetSpeed(float cruise, float max_speed);
float motionCruise();
float motionMaxSpeed();

// Para el monitor. Cómo está girando ahora: 0 no gira · 1 giro normal · 2 ajuste fino
// (giroscopio) · 3 por pulsos · 4 haciendo sitio en recta (ningún lado libre) · 5 quieto
// sin lado libre · 6 detenido por giro trabado · 7 corrimiento tras trabarse
int motionTurnMode();
void motionLastCommand(float *left, float *right);
void motionStop();

// Primitivas NO bloqueantes: llamar en cada ciclo; devuelven true al terminar (y frenan).
// Usan la huella real del robot: 'snap' para saber qué hay alrededor y 'carried'
// para ignorar el cubo que va en las pinzas (COLOR_UNKNOWN si no lleva ninguno).

// Pivota hacia el rumbo por el lado en que las pinzas no pasan sobre un cubo ni
// sobre la línea. Si ningún lado es seguro, hace sitio en línea recta. Con un cubo
// en las pinzas gira despacio, para que la pinza lo arrastre sin soltarlo.
bool motionTurnTo(const Pose &pose, float target_heading, float tol_deg,
                  const TelemetrySnapshot &snap, CubeColor carried);

// Va al punto corrigiendo el rumbo en arco; solo pivota si el desvío es grande.
bool motionGoTo(const Pose &pose, Point target, float tol, float max_power,
                const TelemetrySnapshot &snap, CubeColor carried);

// Órdenes de motor para el tramo final de una entrega: avanza recto hacia 'to'
// con el cubo contra el frente, corrigiendo apenas el rumbo.
// 'cube_ahead' = distancia eje -> centro del cubo.
void carryCommand(const Pose &pose, Point to, float cube_ahead, float &left, float &right);

// Calibración (BLOQUEA ~4 s y mueve el robot: ~20 cm adelante y un giro).
// Mide latencia y ganancias, las deja activas y las imprime para copiar a config.h.
void motionCalibrate();

struct MotionCal {
    float latency_ms;
    float speed_gain;   // celdas/s por unidad de potencia
    float turn_gain;    // grados/s por unidad de potencia
};
const MotionCal& motionCal();

#endif // ROVER_MOTION_H
