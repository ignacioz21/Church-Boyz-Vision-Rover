# QA 01: Patrullaje de Bordes con Holgura y Giros a 90°

Este programa pertenece a la suite de **Control de Calidad de Datos (QA)** para el robot CenfoBot (ESP32 / IdeaBoard).

---

## 🎯 Objetivo de la Prueba

Verificar de forma cuantitativa en cancha:
1. **Recepción y procesamiento de telemetría a 20 Hz** vía TCP (puerto 2026).
2. **Detección y frenado determinista en los límites del mapa** respetando una holgura de seguridad de $5.0\text{ celdas}$ ($10.0\text{ cm}$) para evitar caídas o impactos.
3. **Precisión del giro sobre su propio eje a $90^\circ$ a la izquierda** (medición de error angular y sobrepaso).
4. **Patrullaje perimetral continuo** a lo largo de los 4 lados de la cancha.
5. **Lectura simultánea de sensores de piso (4IR) y sensor ultrasónico**.

---

## ⚙️ Parámetros Clave (`qa_config.h`)

| Parámetro | Valor | Descripción |
| :--- | :--- | :--- |
| `EDGE_CLEARANCE_CELLS` | `5.0f` ($10\text{ cm}$) | Holgura de seguridad respecto a la orilla del mapa |
| `CORNER_ARRIVE_TOL_CELLS` | `1.8f` ($3.6\text{ cm}$) | Tolerancia de llegada para frenar en la esquina |
| `TURN_TOL_DEG` | `3.0f` | Tolerancia angular para dar por finalizado el giro a 90° |
| `QA_SPEED_CRUISE` | `0.35f` | Potencia de avance controlada |
| `QA_SPEED_PIVOT` | `0.45f` | Potencia de giro diferencial sobre su eje |
| `MAX_EDGES_TO_PATROL` | `4` | Cantidad de bordes a recorrer (1 vuelta completa) |

---

## 🚀 Cómo Ejecutar la Prueba

### Opción A: Desde Arduino IDE
1. Conecta la tarjeta IdeaBoard por USB al computador.
2. Abre el archivo [`qa_01_borde_patrol.ino`](qa_01_borde_patrol.ino).
3. Selecciona la placa **ESP32 Dev Module** y el puerto COM/tty correspondiente.
4. Presiona **Subir** (Upload) y abre el Monitor Serie a **115200 baudios**.

### Opción B: Desde PlatformIO / VS Code
```bash
cd rover_qa/qa_01_borde_patrol
pio run --target upload
pio device monitor -b 115200
```

---

## 🚦 Modos de Disparo de Inicio

1. **Modo Torneo / Visión (Guiado por Cámara):**
   - Disparo automático en cuanto la visión pase a fase `RUNNING`.
   - Disparo manual: Enviar **`s`** por el Monitor Serie o presionar **brevemente** el botón **`BOOT`** (GPIO 0).
   - *Requiere que el rover esté conectado al servidor de visión y que la cámara detecte el marcador ArUco.*
2. **Modo Banco de Pruebas (Autónomo sin Cámara / Sin Visión):**
   - Enviar **`t`** por el Monitor Serie o **mantener presionado el botón `BOOT` durante 2 segundos**.
   - Ejecuta una rutina de verificación directa de hardware: avance de 1.2s, parada de 0.6s, giro a la izquierda de 90° (0.6s a potencia 0.45) y reporte de lecturas de sensores (ultrasonido y 4IR de piso).
   - *Ideal para comprobar el funcionamiento mecánico, puentes H y batería del CenfoBot.*

---

## 💡 Códigos de Color LED NeoPixel

| Color | Estado | Significado |
| :--- | :--- | :--- |
| **Blanco** | `INIT` | Inicializando microcontrolador y periféricos |
| **Azul** | `WAIT_START` | Conectado a Wi-Fi y esperando señal de inicio |
| **Cian** | `APPROACH_FIRST_EDGE` | Avanzando hacia el primer borde en su rumbo |
| **Amarillo** | `CORNER_STOP` | Detenido en la esquina / holgura |
| **Magenta** | `PIVOT_LEFT_90` | Girando en su eje 90° a la izquierda |
| **Verde** | `PATROL_EDGE` | Patrullando a lo largo del borde hacia la siguiente esquina |
| **Verde Brillante** | `REPORT_FINISHED` | **Prueba completada exitosamente** (4 bordes recorridos) |
| **Rojo parpadeante**| `FAILSAFE_LAG` | Pérdida de telemetría (`age_ms > 500 ms`) |
