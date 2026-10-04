# Simulador de la lógica del rover

Compila en la PC el **mismo código** del firmware (`rover_10/src/`: geometría,
movimiento, navegación, plan y estrategia) contra una física simple: un rover con
la forma real (cuerpo y pinzas), cubos que se toman y se llevan, y telemetría con
latencia. Sirve para probar cambios de estrategia sin la cancha.

```
./run.sh [semilla] [plan] [-v | -vv]
./run.sh 3 "P,1,10=rgb,11=" -v     # un rover lleva los tres cubos; -v muestra los estados
./run.sh 3 "P,1,10=rgb,11=" -vv    # además, pose y motores una vez por segundo
SIM_LEVEL=0.75 ./run.sh 3 -v       # cubos del generador oficial, dificultad 0.75

./lote.sh oficial 100              # 100 escenarios por cada dificultad oficial (0 a 1)
./lote.sh 150                      # cubos al azar en zona central
SIM_MARGIN=3 ./lote.sh 100         # cubos al azar hasta a 3 celdas del borde
```

## De dónde salen los cubos

Con `SIM_LEVEL` (0 a 1) se usa un port exacto del **generador oficial**
(`docs/index.html`, https://universidad-cenfotec.github.io/Vision-Rover-Challenge/):
en 0 los cubos quedan junto a sus zonas; en 1, lejos y con las rutas cruzadas (el
verde dentro de la zona azul y el azul dentro de la verde). Sin `SIM_LEVEL` los
cubos caen al azar.

El generador dibuja la cancha girada 90° respecto a lo que publica la visión y con
robots de 4 x 3 casillas; `sim.cpp` convierte las coordenadas y separa las filas de
salida de los rovers para que quepa el robot real. **Dónde colocará la organización
cada rover no está confirmado.**

`run.sh` recompila; `lote.sh` usa el último binario. Un escenario termina en `OK`
si el rover entregó todos sus cubos sin rozar ninguno y sin sobresalir de las
líneas más de `SIM_OUT_TOL` celdas (2 por defecto).

## Qué supone el simulador (y no se ha comprobado en la cancha)

- Las medidas del robot (`sim.cpp`, mismas que `config.h`) salen de una foto.
- Un cubo dentro de las pinzas se queda adentro al avanzar, al girar en arco y
  también al **pivotar en el sitio** (la pinza lo arrastra).
- La física real difiere ~10 % de la calibración de `config.h` (constantes `TRUE_*`),
  para que la estrategia no dependa de números exactos.

Que pase aquí no garantiza la cancha, pero lo que falla aquí casi seguro falla allá.
