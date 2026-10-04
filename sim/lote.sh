#!/usr/bin/env bash
# Corre muchos escenarios y resume por tipo de fallo. Usa el último binario de run.sh.
#   ./lote.sh [cantidad] [plan]        plan: "P,1,10=rgb,11=" (un rover), "P,1,10=g,11=rb", o "ninguno" (dos, sin plan)
#   SIM_LEVEL=0.5 ./lote.sh 100      posiciones del generador oficial, dificultad 0.5
#   ./lote.sh oficial [cantidad]     recorre las dificultades 0, 0.25, 0.5, 0.75 y 1
#   SIM_MARGIN=3 ./lote.sh 100       cubos al azar hasta a 3 celdas del borde
cd "$(dirname "$0")"
if [[ $1 == oficial ]]; then
  for d in 0 0.25 0.5 0.75 1; do printf "D=%-4s " $d; SIM_LEVEL=$d "$0" "${2:-100}" "$3" | head -1; done
  exit
fi
n=${1:-150}; plan=${2:-"P,1,10=rgb,11="}
ok=0; all=0; bump=0; out=0; stuck=0; crash=0; secs=0; fails=""
for s in $(seq 1 $n); do
  o=$(/tmp/rover_sim $s "$plan"); r=$(echo "$o" | grep -E "^(OK|FALLO)")
  if [[ $r == OK* ]]; then ok=$((ok+1)); t=$(echo "$r" | grep -oE 'en [0-9]+' | grep -oE '[0-9]+'); secs=$((secs+t)); else fails="$fails $s"; fi
  d=$(echo "$r" | grep -oE "[0-9]+/[0-9]+"); [[ ${d%/*} == ${d#*/} ]] && all=$((all+1))
  [[ $r != *"roces: 0 "* ]] && bump=$((bump+1))
  [[ $r == *"180.0 s"* ]] && stuck=$((stuck+1))
  echo "$o" | grep -q "fuera de las lineas" && out=$((out+1))
  echo "$o" | grep -q "primer choque" && crash=$((crash+1))
done
avg=$([[ $ok -gt 0 ]] && echo $((secs / ok)) || echo "-")
echo "limpios=$ok | entregó todos=$all | atascado=$stuck | choques entre rovers=$crash | con roces=$bump | fuera de tolerancia=$out | de $n | tiempo medio ${avg} s"
echo "semillas con fallo:$(echo $fails | cut -c1-200)"
