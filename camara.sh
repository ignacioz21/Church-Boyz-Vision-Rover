#!/usr/bin/env bash
# Ajustes de la cámara (Logitech C270) según la luz. Correr DESPUÉS de arrancar la visión:
# la visión fija exposición y balance desde vision-system/vision/config_vision.json, pero
# no la ganancia, y de la ganancia depende que el cubo azul se vea con poca luz.
#
#   ./camara.sh             muestra los valores actuales
#   ./camara.sh dia         luz de día fuerte        (medido 5-oct-2026, 9 a 11 h)
#   ./camara.sh tarde       luz baja o de lámpara    (medido 5-oct-2026, 16 h y 23 h; "noche" es lo mismo)
#   ./camara.sh 90 4800 150 a mano: exposición, balance de blancos, ganancia
#
# Regla para ajustar a ojo: el tablero blanco tiene que verse blanco (ni azulado ni
# amarillento: eso es el balance) y el cubo azul tiene que verse azul y no negro (eso
# es la ganancia). Exposición lo más corta posible, para que los rovers no salgan movidos.
DEV=${CAMARA:-/dev/video0}
case "$1" in
  dia)   set -- 45 5600 0 ;;
  tarde|noche) set -- 80 4800 150 ;;
esac
if [[ -n $3 ]]; then
  v4l2-ctl -d "$DEV" --set-ctrl=auto_exposure=1,white_balance_automatic=0
  v4l2-ctl -d "$DEV" --set-ctrl=exposure_time_absolute="$1",white_balance_temperature="$2",gain="$3"
fi
v4l2-ctl -d "$DEV" --get-ctrl=exposure_time_absolute,white_balance_temperature,gain
