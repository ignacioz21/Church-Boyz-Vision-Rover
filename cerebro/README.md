# Cerebro — monitor, registro y pruebas

Muestra en el navegador lo que publica la visión y lo que reporta cada rover, y
permite mandar comandos de **prueba**. No decide nada de la competencia: según el
reglamento (6.3, 11.2) eso corre a bordo, en `rover_*/src/strategy.cpp`.

```
vision-system/.venv/bin/python cerebro/brain.py     # -> http://localhost:8891
```

## Registro de rondas

`recorder.py` graba **siempre** (`registros/continuo/AAAAMMDD_HH.jsonl`, se borra solo a
los 3 días) y además arma un archivo por ronda, `registros/ronda_AAAAMMDD_HHMMSS.jsonl`,
que abre y cierra solo:

| Empieza cuando… | Termina cuando… |
|---|---|
| la visión pasa a `READY` (oficial) | la visión termina la ronda |
| un rover empieza a trabajar (práctica) | los tres cubos están en zona 2 s, o ningún rover trabaja 6 s |
| se pulsa **Grabar** (manual) | se pulsa **Cerrar ronda** |

Cada archivo incluye los 5 s anteriores al inicio y una ficha con la disposición de los
cubos, el nivel del generador, el firmware (`FW_VERSION` de `config.h`) y cómo empezó. Al
cerrar se anota en `registros/indice.csv` (una fila por ronda).

El vigilante avisa (en el log del dashboard y en el archivo) cuando un rover deja de
reportar, pierde la telemetría o pasa 15 s sin avanzar. Solo observa: no le manda nada
a los rovers.

## Disposiciones de práctica

En el dashboard, **Disposición de práctica**: se elige el nivel (0,2 a 0,6) y un número, y
el mapa muestra la sombra de dónde va cada cubo y cada rover; se marca ✓ cuando la cámara
lo ve en su lugar. Mismo nivel y mismo número dan siempre la misma disposición
(`generador.py`, con las reglas del generador oficial de `docs/index.html`). El nivel
queda en la ficha de la ronda. Las rondas hechas sin generar se clasifican por las
posiciones (`nivel.py`), con menos precisión.

## Modelo y planificador

| Comando (desde la raíz, con `vision-system/.venv/bin/python`) | Qué hace |
|---|---|
| `cerebro/catalogar.py` | Rearma `registros/indice.csv` con todas las rondas grabadas |
| `cerebro/episodios.py` | Rondas → una fila por intento de cubo (`datos/episodios.csv`) |
| `cerebro/modelo.py` | Ajusta tiempos y fallos por rover (`datos/modelo.json`) |

## Fase CEREBRO (antes de READY)

Con los rovers en reposo y los cubos quietos 1,5 s, `brain.py` calcula el plan y lo carga
en cada rover; lo repite hasta que el rover lo confirma en su estado (`plan` y `rt`), y
lo recalcula si algo se mueve. Desde `READY` no envía nada (reglamento 6.3, 11.2.7): la
fase queda en pausa. Es legal porque el plan se genera y se carga antes de `READY`
(6.2.5, 8.6.6). El botón **Cerebro: Apagar** del dashboard la desactiva y los rovers
planifican todo a bordo, como antes.

- **Qué lleva el plan:** un cubo para cada rover y el tercero ("comodín") al final de la
  lista de los dos: lo toma el que quede libre primero, y se avisan entre ellos. Para
  cada cubo: cómo tomarlo, la ruta de ida, cómo entregarlo y la ruta con el cubo.
- **De dónde salen las rutas:** del planificador del propio rover. `compilar.sh` compila
  el firmware de cada rover como biblioteca (`datos/librover_N.so`) y `rutas_pc.py` le
  pregunta; se recompila solo cuando cambia el firmware. No hay una copia en Python.
- **Si una ruta ya no sirve** (el cubo se movió, algo se cruzó), el rover la descarta y
  planifica él: el peor caso es el comportamiento sin cerebro.
- **Cómo se elige el reparto** (`planner.py`, `plan_completo`): el que antes termina
  según lo que hay que andar y girar, con un recargo si los recorridos se cruzan.
  `modelo.py` (ajuste por rover con las rondas grabadas) queda para análisis; hoy
  predice mal y no decide el reparto.
- **Práctica:** "Iniciar práctica: los dos" espera a que el plan quede cargado y arranca.
- **Probar en el simulador:** `sim/con_cerebro.py 40` compara las mismas rondas con y sin
  esta fase.

## Protocolo con los rovers (UDP)

| Dirección | Puerto | Formato |
|---|---|---|
| Cerebro → rover | 8889 (broadcast) | `10:r`, `11:f`, `*:r`, `10:M,0.4,0.4` |
| Rover → cerebro | 8888 | JSON de una línea, cada 200 ms: `{"id":10,"state":"...","fresh":true,"link":true,"phase":"IDLE","col":..,"row":..,"theta":..,"seq":..}` |

Comandos de prueba: `r` para, `f` avanza 1 s, `M,izq,der` potencia directa
(caduca a los 500 ms si no se renueva).
`V,crucero,tope` cambia la potencia de crucero y el tope de velocidad (celdas/s) hasta
el próximo reinicio; solo se acepta antes de la ronda. Subirla por encima de 0,50 no
está probado en la cancha: hacerlo de a poco y mirando que el rover no se reinicie.
El estado del rover trae además `rt` (rutas cargadas), `np` (veces que calculó una ruta
y milisegundos parado en eso), `spd` (crucero y tope vigentes) y `gs` (arrancó sin giroscopio).

**Bloqueo de ronda (reglamento 6.3, 9.5, 11.2).** Desde que la visión pasa a `READY` y
hasta que termina la ronda, el cerebro no transmite ningún comando (los botones quedan
sin efecto y el log dice `BLOQUEADO`) y los rovers descartan cualquier comando que les
llegue, incluido el STOP. Los botones sirven solo con la visión en `IDLE` o `FINISHED`.
El cerebro sigue escuchando y grabando, que es solo observar. Si los jueces no aceptan
ni eso, `STATUS_DURING_ROUND 0` en `config.h` hace que los rovers no reporten nada
durante la ronda.

Las IP y los puertos se configuran en `rover_*/include/config.h` y al inicio de
`brain.py`; tienen que coincidir.
