#!/usr/bin/env bash
# Compila el firmware de cada rover como biblioteca para la fase CEREBRO (rutas_pc.py):
# la PC planifica con el MISMO código, la misma huella y la misma calibración que el rover.
# Hay que volver a correrlo cada vez que cambia el firmware (rutas_pc.py lo hace solo si
# ve que alguna fuente es más nueva que la biblioteca).
set -e
cd "$(dirname "$0")/../sim"
OUT=../cerebro/datos
mkdir -p $OUT
FLAGS="-std=c++17 -O1 -w -I shim"
for id in 10 11; do
  R=../rover_$id
  g++ $FLAGS -shared -fPIC -fvisibility=hidden -Wl,-Bsymbolic -I $R/include -o $OUT/librover_$id.so \
      rover_lib.cpp $R/src/geometry.cpp $R/src/motion.cpp $R/src/plan.cpp $R/src/nav.cpp $R/src/coord.cpp $R/src/strategy.cpp
done
echo "Bibliotecas de planificación listas en cerebro/datos/"
