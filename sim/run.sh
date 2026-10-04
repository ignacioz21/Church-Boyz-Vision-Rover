#!/usr/bin/env bash
# Compila el firmware de cada rover como biblioteca, el mundo, y corre un escenario.
#   ./run.sh [semilla] [plan] [-v | -vv]        (ver mundo.cpp)
set -e
cd "$(dirname "$0")"
FLAGS="-std=c++17 -O1 -w -I shim"
for id in 10 11; do
  R=../rover_$id
  g++ $FLAGS -shared -fPIC -fvisibility=hidden -Wl,-Bsymbolic -I $R/include -o /tmp/librover_$id.so \
      rover_lib.cpp $R/src/geometry.cpp $R/src/motion.cpp $R/src/plan.cpp $R/src/nav.cpp $R/src/coord.cpp $R/src/strategy.cpp
done
g++ $FLAGS -o /tmp/rover_sim mundo.cpp ../rover_10/src/geometry.cpp -ldl
/tmp/rover_sim "$@"
