# Algoritmo de Navegación y Control (Vision Rover Challenge)

Este documento describe el paso a paso de las decisiones y acciones que tomarán los rovers desde el encendido hasta la finalización del reto.

## FASE 0: INICIALIZACIÓN Y DIAGNÓSTICO
Antes de iniciar, cada rover ejecuta una Matriz de Salud:
1. Verifica voltaje de batería.
2. Verifica conexión Wi-Fi (Telemetría) y ESP-NOW (Rover a Rover).
3. Verifica lecturas iniciales de sensores (Ultrasónico, Infrarrojo, Color, IMU).

---

## FASE 1: ESTADO `IDLE` (1 Minuto)
* **Objetivo:** Esperar la señal de inicio sin moverse.
* **Acciones:**
  - Los rovers escuchan la telemetría oficial (TCP Puerto 2026).
  - Mantienen los motores apagados.
  - Sincronizan sus relojes internos con la frecuencia de paquetes recibidos (20Hz esperado).

---

## FASE 2: ESTADO `READY` (Pre-Planificación)
* **Objetivo:** Calcular la estrategia completa antes de moverse.
* **Asignación de Cubos:**
  - Se identifican las coordenadas de los 3 cubos y de los 2 rovers.
  - **Rover de 1 Parada:** Se le asigna el cubo más cercano a él.
  - **Rover de 2 Paradas:** Se le asigna el otro cubo cercano. El 3er cubo (restante) será recogido por este rover después de su primera entrega.
* **Gestión de Bordes Invisibles (Riesgo de Caída):**
  - Se analizan las coordenadas de los cubos. Si un cubo está a menos de 5 cm del borde de la mesa, el cálculo de aproximación lineal recta es peligroso.
  - **Solución (Diagonal de Seguridad):** Se proyecta una ruta de aproximación en ángulo diagonal seguro. El riesgo de desalineación de esta maniobra se calcula y mitiga matemáticamente desde este momento para garantizar que, tras el impacto diagonal, el rover pueda re-alinearse hacia el acopio sin caerse.

---

## FASE 3: ESTADO `RUNNING` (Ejecución)
Al cambiar la bandera a `RUNNING`, ambos rovers arrancan simultáneamente.

### 3.1. Navegación al Cubo
* Los rovers se dirigen al punto de "pre-aproximación" calculado (detrás de su cubo asignado).
* **Control de Tráfico (Mutex de Cruce mediante ESP-NOW):**
  - La comunicación entre rovers para cederse el paso será mediante **ESP-NOW** (independiente de la red Wi-Fi del torneo, garantizando respuesta ultra-rápida).
  - Si las trayectorias de los dos rovers se van a cruzar con un margen de menos de 2 segundos, **el Rover de 1 Parada SIEMPRE cede el paso**, enviando un estado de "Frenado" vía ESP-NOW hasta que el Rover de 2 Paradas haya despejado la intersección.

### 3.2. Confirmación de Contacto (3 Etapas)
1. **Visión:** Confirma que el rover está cerca del cubo.
2. **Ultrasónico:** Detecta proximidad física inminente.
3. **Color + Infrarrojo:** Confirman que se ha llegado a la cara del cubo.

### 3.3. Empuje (Push)
* El rover empuja en línea recta hacia la zona de acopio.
* **Tolerancia de Lag (Interruptor de Hombre Muerto):**
  - El marcador del rover siempre es visible para la cámara. Sin embargo, si el servidor Wi-Fi se satura y el paquete de telemetría supera los **500 ms de antigüedad (`age_ms > 500`)**, el rover frenará automáticamente para no avanzar a ciegas.
* **Fusión de Sensores Infrarrojo / IMU [ESTADO CRÍTICO PENDIENTE]:**
  - El cuenta-pasos se basará en el Infrarrojo apuntando al tablero (confiando en la sombra constante bajo el chasis).
  - *Pendiente de Pruebas Físicas:* Validar la resistencia/esfuerzo de los motores usando el Acelerómetro (IMU) para confirmar físicamente el avance y detectar patinaje.

---

## FASE 4: DEPÓSITO Y RETIRADA SEGURA (Safe Retreat)
* Una vez que la cámara (o telemetría) confirma que el cubo está 100% dentro del área válida de acopio, el rover detiene el empuje.
* **Retirada Curva en Reversa:**
  - Para evitar golpear o barrer accidentalmente el cubo recién depositado (lo cual lo descalificaría), el rover ejecutará un **giro amplio hacia atrás (curva en reversa)** para alejarse de manera rápida y segura de la zona de acopio.

---

## FASE 5: FINALIZACIÓN DE TAREA (Post-Job)
* **Rover de 2 Paradas:** Tras dejar su primer cubo y hacer la retirada segura, procede a buscar el 3er cubo siguiendo el mismo ciclo.
* **Rover de 1 Parada (Inactivo):** 
  - Tras terminar su retirada segura, ejecuta su plan "Post-Job".
  - Se desplazará a una "zona de estacionamiento" predefinida (ej. esquina más lejana o un área libre que no estorbe la ruta del otro rover) y se apagará lógicamente.
  - Esto se hace porque las reglas 10.3.1 y 10.5 dictan que el reto termina en el instante en que el 3er cubo toca la zona de acopio, siendo la posición final de los rovers totalmente irrelevante.
