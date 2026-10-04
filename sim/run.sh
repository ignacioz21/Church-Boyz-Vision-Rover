#!/usr/bin/env bash
# Compila la lógica del rover para la PC y corre un escenario.
#   ./run.sh [semilla] [plan] [-v]
set -e
cd "$(dirname "$0")"
R=../rover_10/src
g++ -std=c++17 -O1 -w -I shim -o /tmp/rover_sim sim.cpp $R/geometry.cpp $R/motion.cpp $R/plan.cpp $R/nav.cpp $R/strategy.cpp
/tmp/rover_sim "$@"
