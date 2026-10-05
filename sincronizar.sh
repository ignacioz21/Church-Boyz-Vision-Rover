#!/usr/bin/env bash
# Copia el firmware de rover_10/ a rover_11/, conservando lo propio del 11:
# identidad, polaridad y escala de motores y calibración (config.h) y las credenciales (secrets.h).
set -e
cd "$(dirname "$0")"
for d in src include; do
  for f in rover_10/$d/*; do
    b=$(basename "$f")
    [[ $b == config.h || $b == secrets.h ]] && continue
    cp "$f" rover_11/$d/"$b"
  done
done
sed 's/^ \* Rover 10 —/ * Rover 11 —/' rover_10/rover_10.ino > rover_11/rover_11.ino
python3 - <<'PY'
import re
src = open('rover_10/include/config.h').read()
old = open('rover_11/include/config.h').read()
keep = lambda name: re.search(r'^#define %s .*$' % name, old, re.M).group(0)
out = src
for name in ['ROVER_ID', 'ROVER_PEER_ID', 'INVERT_MOTOR_L', 'MOTOR_SCALE', 'LOOP_LATENCY_MS', 'SPEED_GAIN', 'TURN_GAIN']:
    out = re.sub(r'^#define %s .*$' % name, lambda m, n=name: keep(n), out, flags=re.M)
out = re.sub(r'^// 4\. CALIBRACIÓN DE ESTE ROBOT.*$',
             lambda m: re.search(r'^// 4\. CALIBRACIÓN DE ESTE ROBOT.*$', old, re.M).group(0), out, flags=re.M)
open('rover_11/include/config.h', 'w').write(out)
PY
echo "rover_11 sincronizado. Diferencias con rover_10:"
diff -r rover_10 rover_11 | grep -E "^[<>]" | cut -c1-100 || true
