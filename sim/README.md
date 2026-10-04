# Simulador

Corre en la PC el **mismo código** del firmware contra un mundo simulado, para probar
la estrategia sin la cancha.

- Cada rover es su firmware real (`rover_10/src/` y `rover_11/src/`, cada uno con su
  `config.h`), empaquetado como una biblioteca aparte (`rover_lib.cpp`). No comparten
  nada: cada uno ve al otro solo por la telemetría, como en la cancha.
- `mundo.cpp` hace de cancha: mueve los robots según sus motores, arrastra los cubos
  que llevan en las pinzas, detecta roces y choques, y le entrega a cada rover la
  telemetría con **su** latencia.
- `cerebro/planner.py` (el planificador de la PC) decide el reparto de cubos, igual
  que lo hará el cerebro en IDLE.
- Los rovers **se hablan entre sí** (`coord.h`): el mundo les pasa los mensajes con un
  atraso y perdiendo algunos (`SIM_PEER=0` los deja sin radio; `SIM_PEER_LOSS=0.5`
  pierde la mitad).

```
./run.sh [semilla] [plan] [-v | -vv]
./run.sh 3 "P,1,10=rgb,11=" -v        # solo el rover 10, con los tres cubos
./run.sh 3 "P,1,10=g,11=rb" -v        # los dos rovers, con ese reparto
./run.sh 3 ninguno -v                 # los dos, sin plan de la PC
SIM_LEVEL=0.75 ./run.sh 3 ninguno -vv # cubos del generador oficial, dificultad 0.75

./lote.sh oficial 100 ninguno         # 100 escenarios por cada dificultad oficial
./analizar.py 100                     # informe completo: 1 rover / 2 sin radio / 2 con radio / 2 con radio mala
```

`run.sh` recompila; `lote.sh` y `analizar.py` usan el último binario. `-v` muestra los
cambios de estado de cada rover; `-vv`, además, pose y motores una vez por segundo.

Una corrida es **limpia** si los tres cubos quedan en su zona, sin rozar ningún cubo,
sin choques entre rovers y sin sobresalir de las líneas más de `SIM_OUT_TOL` celdas
(2 por defecto).

## De dónde salen los cubos

Con `SIM_LEVEL` (0 a 1) se usa un port exacto del **generador oficial**
(`docs/index.html`, https://universidad-cenfotec.github.io/Vision-Rover-Challenge/):
en 0 los cubos quedan junto a sus zonas; en 1, lejos y con las rutas cruzadas (el
verde dentro de la zona azul y el azul dentro de la verde). Sin `SIM_LEVEL` los cubos
caen al azar (`SIM_MARGIN` = qué tan cerca del borde).

El generador dibuja la cancha girada 90° respecto a lo que publica la visión y con
robots de 4 x 3 casillas; `mundo.cpp` convierte las coordenadas y separa las filas de
salida de los rovers para que quepa el robot real.

## Lo que el simulador supone (sin comprobar en la cancha)

- **Medidas del robot**: salen de una foto (`mundo.cpp` y `config.h`).
- **Cubo en las pinzas**: se queda adentro al avanzar y al pivotar en el sitio.
- **Salida de los rovers**: a 3,75 celdas de la línea, filas 12 y 31, mirando a la cancha.
- **Física**: sin ruido de cámara, sin ruedas que patinen; difiere ~10 % de la
  calibración de cada `config.h` (ver `loadRover` en `mundo.cpp`).

Que pase aquí no garantiza la cancha, pero lo que falla aquí casi seguro falla allá.
