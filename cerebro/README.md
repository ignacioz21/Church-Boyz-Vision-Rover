# Cerebro — monitor, registro y pruebas

Muestra en el navegador lo que publica la visión y lo que reporta cada rover, y
permite mandar comandos de **prueba**. No decide nada de la competencia: según el
reglamento (6.3, 11.2) eso corre a bordo, en `rover_*/src/strategy.cpp`.

```
vision-system/.venv/bin/python cerebro/brain.py     # -> http://localhost:8891
```

## Registro de rondas

`recorder.py` graba cada ronda en `registros/ronda_AAAAMMDD_HHMMSS.jsonl` (una línea
JSON por dato): el mundo según la visión 5 veces por segundo, lo que reporta cada
rover, los eventos (cambio de fase, cubo que entra o sale de su zona), los avisos del
vigilante y un resumen al cerrar. Empieza solo cuando la visión pasa a `READY` y
termina en `FINISHED` o `IDLE`; el botón **Grabar** del dashboard graba a mano.

El vigilante avisa (en el log del dashboard y en el archivo) cuando un rover deja de
reportar, pierde la telemetría o pasa 15 s sin avanzar en un estado en que debería
moverse. Solo observa: no le manda nada a los rovers.

`planner.py` es el planificador (reparto y orden de los cubos). Hoy lo usa el
simulador; falta que `brain.py` lo calcule en IDLE y cargue el plan en los rovers.

## Protocolo con los rovers (UDP)

| Dirección | Puerto | Formato |
|---|---|---|
| Cerebro → rover | 8889 (broadcast) | `10:r`, `11:f`, `*:r`, `10:M,0.4,0.4` |
| Rover → cerebro | 8888 | JSON de una línea, cada 200 ms: `{"id":10,"state":"...","fresh":true,"link":true,"phase":"IDLE","col":..,"row":..,"theta":..,"seq":..}` |

Comandos de prueba: `r` para, `f` avanza 1 s, `M,izq,der` potencia directa
(caduca a los 500 ms si no se renueva).

Las IP y los puertos se configuran en `rover_*/include/config.h` y al inicio de
`brain.py`; tienen que coincidir.
