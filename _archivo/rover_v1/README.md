# Rover V1 — Software Autónomo de Navegación (C/C++ ESP32)

Este directorio contiene el sistema completo y autónomo de control y navegación para el robot **CenfoBot** (IdeaBoard con microcontrolador ESP32) desarrollado para el **Vision Rover Challenge**, cumpliendo estrictamente con el [`reglamento.md`](../reglamento.md) y el [`ALGORITMO.md`](../ALGORITMO.md).

---

## 1. Contexto del Reto y Cumplimiento Normativo

El objetivo del reto es que dos rovers autónomos identifiquen, alcancen y empujen 3 cubos de colores (`red`, `green`, `blue`) hacia sus respectivas zonas de acopio oficiales en la menor cantidad de tiempo posible.

### Reglas Clave Cumplidas:
* **Autonomía Total (Regla 4.3 y 6.3):** A partir de que la visión pasa a `READY` y `RUNNING`, toda la toma de decisiones, cálculo de trayectorias y control de motores se realiza **exclusivamente a bordo del ESP32**. La computadora externa no envía comandos de movimiento ni calcula rutas.
* **Lenguaje y Firmware (Regla 4.2.9 y 1.10):** Se utiliza C/C++ nativo para el ESP32, garantizando alto rendimiento y tiempos de respuesta deterministas sin pausas de recolección de basura (*Garbage Collection*).
* **Protocolo de Visión Oficial (Regla 7.1 y CONTRATO.md):** El rover se conecta como cliente TCP a la IP de la computadora en el puerto `2026`, procesando telemetría en formato **NDJSON** a 20 Hz.

---

## 2. Arquitectura de Software Multinúcleo (FreeRTOS)

El firmware aprovecha la arquitectura de **doble núcleo (Xtensa dual-core @ 240MHz)** del ESP32 mediante FreeRTOS para desacoplar por completo la red de la física del robot:

```
┌─────────────────────────────────────────────────────────────┐
│                    ESP32 (CenfoBot)                         │
├──────────────────────────────┬──────────────────────────────┤
│    CORE 0: Tarea de Red      │   CORE 1: Bucle de Control   │
│       (FreeRTOS Task)        │        (loop() a 50 Hz)      │
├──────────────────────────────┼──────────────────────────────┤
│ 1. Conexión Wi-Fi TCP a 2026 │ 1. Lectura de telemetría     │
│ 2. Buffer de líneas NDJSON   │ 2. Máquina de Estados (FSM)  │
│ 3. Parseo con ArduinoJson    │ 3. Álgebra vectorial 2D      │
│ 4. Watchdog de latencia      │ 4. Control proporcional PID  │
│ 5. Escritura atómica estado  │ 5. PWM LEDC a motores (50Hz) │
└──────────────────────────────┴──────────────────────────────┘
```

* **Sin bloqueos:** Si la red Wi-Fi presenta fluctuaciones o una trama JSON tarda en llegar, el Core 1 sigue controlando los motores de forma segura a 50 Hz sin congelarse.
* **Seguridad (Deadman Switch):** Si la telemetría supera los 500 ms de antigüedad (`age_ms > 500`), el robot corta de inmediato los motores para evitar desbordes o accidentes.

---

## 3. Mapa de Hardware y Pines (IdeaBoard)

La tarjeta **CRCibernetica IdeaBoard** utiliza la siguiente asignación de pines:

| Periférico | Pin ESP32 | Función / Descripción |
| :--- | :--- | :--- |
| **Motor 1 (Izquierdo)** | `IO12` y `IO14` | Salidas PWM H-Bridge (Canal 0 y 1, 50 Hz) |
| **Motor 2 (Derecho)** | `IO13` y `IO15` | Salidas PWM H-Bridge (Canal 2 y 3, 50 Hz) |
| **Sonar HC-SR04 (Trigger)** | `IO25` | Disparo del sensor ultrasónico |
| **Sonar HC-SR04 (Echo)** | `IO26` | Retorno de pulso del sensor ultrasónico |
| **Sensores IR de Piso (4IR)**| `IO36, IO39, IO34, IO35` | Detección analógica de reflectancia |
| **LED RGB NeoPixel** | `IO4` / `IO2` | Indicador visual del estado de la FSM |
| **I2C (IMU LSM6DS3TRC)** | `IO21` (SDA), `IO22` (SCL) | Giroscopio y acelerómetro integrado |

---

## 4. Estructura del Directorio

```
rover_v1/
├── README.md                 # Este documento
├── platformio.ini            # Configuración para PlatformIO / VS Code
├── rover_v1.ino              # Archivo principal para Arduino IDE
│
├── docs/                     # Especificaciones teóricas y matemáticas
│   ├── FSM_ESTADOS.md        # Documentación formal de la FSM y diagrama Mermaid
│   └── MATEMATICAS_NAVEGACION.md # Álgebra de vectores, rumbos, bordes y depósitos
│
├── include/                  # Cabeceras C++ (.h)
│   ├── config.h              # Credenciales Wi-Fi, pines, IP y constantes de ajuste
│   ├── types.h               # Estructuras de datos (Pose, Cubo, Acopio, Telemetría)
│   ├── hardware.h            # Interfaz de bajo nivel (motores, sensores, LED)
│   ├── telemetry.h           # Cliente TCP y parseo JSON en Core 0
│   ├── navigation.h          # Algoritmos de navegación y mitigación de bordes
│   └── fsm.h                 # Definición de la máquina de estados
│
├── src/                      # Código fuente C++ (.cpp)
│   ├── hardware.cpp          # Driver LEDC PWM y lectura de sensores
│   ├── telemetry.cpp         # Tarea FreeRTOS y conexión TCP socket
│   ├── navigation.cpp        # Funciones de trigonometría y cálculo de waypoints
│   └── fsm.cpp               # Transiciones y lógica de estados autónomos
│
└── tools/                    # Utilidades de prueba en PC
    └── test_pc_telemetry.py  # Emulador para pruebas locales de red y telemetría
```

---

## 5. Guía de Configuración Rápida

Antes de compilar, abre el archivo [`include/config.h`](include/config.h) y ajusta:

```cpp
// 1. Red Wi-Fi
#define WIFI_SSID           "MI_RED_WIFI"
#define WIFI_PASSWORD       "MI_CONTRASENA"

// 2. Servidor de Visión
#define VISION_HOST         "192.168.1.47"  // IP de la laptop que corre la cámara
#define VISION_PORT         2026            // Puerto oficial según CONTRATO.md

// 3. Identificador del Rover
#define ROVER_ID            10              // 10 u 11 (según el marcador ArUco pegado al rover)
```

---

## 6. Instrucciones de Compilación y Carga

### Opción A: Mediante Arduino IDE (Recomendada)
1. Instala el soporte de placas **esp32 by Espressif Systems** en el Gestor de Placas del Arduino IDE.
2. Ve a **Herramientas > Administrar Bibliotecas** e instala:
   - **`ArduinoJson`** (versión 6 o 7 por Benoît Blanchon).
   - **`Adafruit NeoPixel`** (opcional, para el LED de estado).
3. Conecta la IdeaBoard por USB al computador (ejemplo: `/dev/ttyUSB0` en Linux o `COM3` en Windows).
4. Selecciona la placa: **ESP32 Dev Module** (o **NodeMCU-32S**).
5. Abre [`rover_v1.ino`](rover_v1.ino) y presiona **Subir** (Upload).

### Opción B: Mediante PlatformIO
Si usas VS Code con la extensión PlatformIO:
```bash
cd rover_v1
pio run --target upload
pio device monitor -b 115200
```

---

## 7. Códigos de Colores del LED (NeoPixel)

El LED RGB de la IdeaBoard indica en todo momento el estado interno de la FSM:

| Color | Estado | Significado |
| :--- | :--- | :--- |
| **Blanco** | `BOOT_INIT` | Inicializando microcontrolador y pines |
| **Azul fijo** | `IDLE` | Conectado a Wi-Fi y esperando señal `READY` de visión |
| **Cian fijo** | `READY_PLAN` | Planificando cubo y pre-aproximación |
| **Verde parpadeante** | `NAV_PREAPPROACH` | Navegando detrás del cubo |
| **Verde fijo** | `PUSH_TO_DEPOT` | Empujando cubo hacia su depósito |
| **Magenta** | `SAFE_RETREAT` | Retirada en reversa curvada de seguridad |
| **Amarillo** | `CHECK_NEXT_CUBE`| Seleccionando siguiente cubo pendiente |
| **Arcoíris / Verde** | `MISSION_FINISHED`| Todos los cubos entregados (Misión cumplida) |
| **Rojo parpadeante**| `FAILSAFE_LAG` | **ALERTA**: Pérdida de telemetría (`age_ms > 500`) |
