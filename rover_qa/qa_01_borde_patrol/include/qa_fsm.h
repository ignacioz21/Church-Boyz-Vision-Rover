#ifndef QA_FSM_H
#define QA_FSM_H

#include "types.h"

// Estados de la FSM de prueba QA de bordes
enum QaState {
    QA_STATE_INIT = 0,
    QA_STATE_WAIT_START,
    QA_STATE_APPROACH_FIRST_EDGE,
    QA_STATE_CORNER_STOP,
    QA_STATE_PIVOT_LEFT,
    QA_STATE_PATROL_EDGE,
    QA_STATE_REPORT_FINISHED,
    QA_STATE_FAILSAFE,
    QA_STATE_BENCH_TEST
};

// Inicializa la FSM de QA
void qaFsmInit();

// Actualización cíclica ejecutada a 50 Hz en loop()
void qaFsmUpdate();

// Retorna el estado actual
QaState qaFsmGetState();

// Retorna el nombre en texto del estado
const char* qaFsmGetStateName(QaState state);

// Dispara manualmente el inicio de la prueba (vía botón BOOT o Serial)
void qaFsmTriggerStart();

// Dispara una prueba mecánica de banco autónoma (motores + giro 90°) sin requerir visión
void qaFsmTriggerBenchTest();

// Retorna la cantidad de bordes completados
int qaFsmGetEdgesCompleted();

#endif // QA_FSM_H
