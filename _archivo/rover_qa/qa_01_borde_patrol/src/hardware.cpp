#include "../include/hardware.h"
#include "../include/config.h"

// Macro auxiliar para compatibilidad entre ESP32 Core v2.x y v3.x
static void writeMotorPwm(int channel, int pin, int duty) {
    duty = constrain(duty, 0, 1023);
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    ledcWrite(pin, duty);
#else
    ledcWrite(channel, duty);
#endif
}

void hardwareInit() {
    // 1. Configurar canales PWM para los motores
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    ledcAttach(PIN_M1A, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttach(PIN_M1B, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttach(PIN_M2A, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttach(PIN_M2B, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
#else
    ledcSetup(PWM_CH_M1A, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_M1A, PWM_CH_M1A);
    ledcSetup(PWM_CH_M1B, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_M1B, PWM_CH_M1B);
    ledcSetup(PWM_CH_M2A, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_M2A, PWM_CH_M2A);
    ledcSetup(PWM_CH_M2B, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_M2B, PWM_CH_M2B);
#endif

    stopMotors();

    // 2. Configurar pines del Sensor Ultrasónico
    pinMode(PIN_SONAR_TRIG, OUTPUT);
    pinMode(PIN_SONAR_ECHO, INPUT);
    digitalWrite(PIN_SONAR_TRIG, LOW);

    // 3. Configurar pines de Sensores Infrarrojos
    pinMode(PIN_IR_FL, INPUT);
    pinMode(PIN_IR_FR, INPUT);
    pinMode(PIN_IR_RL, INPUT);
    pinMode(PIN_IR_RR, INPUT);

    // 4. Inicializar NeoPixel apagado
    setLedColor(0, 0, 0);
}

static bool current_invert_l = INVERT_MOTOR_L;
static bool current_invert_r = INVERT_MOTOR_R;

void setMotorInversions(bool invert_l, bool invert_r) {
    current_invert_l = invert_l;
    current_invert_r = invert_r;
}

bool getMotorInvertL() {
    return current_invert_l;
}

bool getMotorInvertR() {
    return current_invert_r;
}

void setMotors(float left, float right) {
    if (current_invert_l) left = -left;
    if (current_invert_r) right = -right;

    // Limitar entre -1.0 y 1.0
    left = constrain(left, -1.0f, 1.0f);
    right = constrain(right, -1.0f, 1.0f);

    // Motor 1 (Izquierdo)
    if (left > 0.05f) {
        int duty = (int)(left * 1023.0f);
        writeMotorPwm(PWM_CH_M1A, PIN_M1A, duty);
        writeMotorPwm(PWM_CH_M1B, PIN_M1B, 0);
    } else if (left < -0.05f) {
        int duty = (int)(-left * 1023.0f);
        writeMotorPwm(PWM_CH_M1A, PIN_M1A, 0);
        writeMotorPwm(PWM_CH_M1B, PIN_M1B, duty);
    } else {
        writeMotorPwm(PWM_CH_M1A, PIN_M1A, 0);
        writeMotorPwm(PWM_CH_M1B, PIN_M1B, 0);
    }

    // Motor 2 (Derecho)
    if (right > 0.05f) {
        int duty = (int)(right * 1023.0f);
        writeMotorPwm(PWM_CH_M2A, PIN_M2A, duty);
        writeMotorPwm(PWM_CH_M2B, PIN_M2B, 0);
    } else if (right < -0.05f) {
        int duty = (int)(-right * 1023.0f);
        writeMotorPwm(PWM_CH_M2A, PIN_M2A, 0);
        writeMotorPwm(PWM_CH_M2B, PIN_M2B, duty);
    } else {
        writeMotorPwm(PWM_CH_M2A, PIN_M2A, 0);
        writeMotorPwm(PWM_CH_M2B, PIN_M2B, 0);
    }
}

void stopMotors() {
    writeMotorPwm(PWM_CH_M1A, PIN_M1A, 0);
    writeMotorPwm(PWM_CH_M1B, PIN_M1B, 0);
    writeMotorPwm(PWM_CH_M2A, PIN_M2A, 0);
    writeMotorPwm(PWM_CH_M2B, PIN_M2B, 0);
}

float readUltrasonicCm() {
    digitalWrite(PIN_SONAR_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_SONAR_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_SONAR_TRIG, LOW);

    // Timeout de 25000 us = ~4 metros de distancia máxima
    long duration_us = pulseIn(PIN_SONAR_ECHO, HIGH, 25000);
    if (duration_us == 0) {
        return 999.0f; // Sin eco / fuera de rango
    }
    return (float)duration_us * 0.0343f / 2.0f;
}

void readFloorSensors(int &fl, int &fr, int &rl, int &rr) {
    fl = analogRead(PIN_IR_FL);
    fr = analogRead(PIN_IR_FR);
    rl = analogRead(PIN_IR_RL);
    rr = analogRead(PIN_IR_RR);
}

void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
    // Control nativo de WS2812B en ESP32 con brillo al 35%
    uint8_t br = (uint8_t)(r * 0.35f);
    uint8_t bg = (uint8_t)(g * 0.35f);
    uint8_t bb = (uint8_t)(b * 0.35f);
    neopixelWrite(PIN_NEOPIXEL, br, bg, bb);
}

void setLedForState(RoverFsmState state) {
    switch (state) {
        case STATE_BOOT_INIT:
            setLedColor(255, 255, 255); // Blanco
            break;
        case STATE_HEALTH_CHECK:
            setLedColor(255, 200, 0);   // Amarillo
            break;
        case STATE_IDLE:
            setLedColor(0, 0, 255);     // Azul
            break;
        case STATE_READY_PLAN:
            setLedColor(0, 255, 255);   // Cian
            break;
        case STATE_NAV_PREAPPROACH:
            setLedColor(0, 255, 0);     // Verde
            break;
        case STATE_PUSH_TO_DEPOT:
            setLedColor(0, 255, 128);   // Verde agua
            break;
        case STATE_SAFE_RETREAT:
            setLedColor(255, 0, 255);   // Magenta
            break;
        case STATE_CHECK_NEXT_CUBE:
            setLedColor(255, 255, 0);   // Amarillo
            break;
        case STATE_MISSION_FINISHED:
            setLedColor(0, 255, 0);     // Verde completo
            break;
        case STATE_FAILSAFE_LAG:
            setLedColor(255, 0, 0);     // Rojo alerta
            break;
        case STATE_UNSTUCK_REVERSE:
            setLedColor(255, 100, 0);   // Naranja desatasco
            break;
    }
}
