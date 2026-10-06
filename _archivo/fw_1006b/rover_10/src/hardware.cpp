#include "../include/hardware.h"
#include "../include/config.h"
#include <Wire.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// Compatibilidad entre ESP32 Arduino Core 2.x (canales) y 3.x (pines)
static void writeMotorPwm(int channel, int pin, int duty) {
    duty = constrain(duty, 0, (1 << PWM_RESOLUTION_BITS) - 1);
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    ledcWrite(pin, duty);
#else
    ledcWrite(channel, duty);
#endif
}

static void attachPwm(int channel, int pin) {
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    ledcAttach(pin, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
#else
    ledcSetup(channel, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(pin, channel);
#endif
}

// Un lado del puente H: el signo elige qué pin lleva el PWM
static void driveSide(float power, int ch_a, int pin_a, int ch_b, int pin_b) {
    int duty = (int)(fabs(power) * ((1 << PWM_RESOLUTION_BITS) - 1));
    if (power > 0.05f) {
        writeMotorPwm(ch_a, pin_a, duty);
        writeMotorPwm(ch_b, pin_b, 0);
    } else if (power < -0.05f) {
        writeMotorPwm(ch_a, pin_a, 0);
        writeMotorPwm(ch_b, pin_b, duty);
    } else {
        writeMotorPwm(ch_a, pin_a, 0);
        writeMotorPwm(ch_b, pin_b, 0);
    }
}

// --- Giroscopio (LSM6DS3TR-C) ------------------------------------------------------------
static bool imu_ok = false;
static uint8_t imu_addr = 0;
static float gyro_bias_dps = 0.0f;
static int imu_errors = 0;
static const float GYRO_DPS_PER_LSB = 0.070f;      // Escala de ±2000 °/s

static bool imuWrite(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(imu_addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool imuReadZ(float &dps) {
    Wire.beginTransmission(imu_addr);
    Wire.write(0x26);                               // OUTZ_L_G
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)imu_addr, 2) != 2) return false;
    int16_t raw = (int16_t)(Wire.read() | (Wire.read() << 8));
    dps = raw * GYRO_DPS_PER_LSB;
    return true;
}

// Si la placa se reinició en medio de una lectura, el sensor puede quedar sujetando la
// línea de datos y el bus no arranca más. Nueve pulsos de reloj a mano lo sueltan.
static void i2cFreeBus() {
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, OUTPUT_OPEN_DRAIN);
    digitalWrite(PIN_I2C_SCL, HIGH);
    for (int i = 0; i < 9 && digitalRead(PIN_I2C_SDA) == LOW; i++) {
        digitalWrite(PIN_I2C_SCL, LOW);
        delayMicroseconds(10);
        digitalWrite(PIN_I2C_SCL, HIGH);
        delayMicroseconds(10);
    }
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);
}

static void imuInit() {
#if !USE_GYRO
    Serial.println("[IMU] Giroscopio desactivado en config.h (USE_GYRO 0)");
    return;
#endif
    i2cFreeBus();
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000);
    Wire.setTimeOut(10);
    for (uint8_t addr : { (uint8_t)0x6B, (uint8_t)0x6A }) {
        Wire.beginTransmission(addr);
        Wire.write(0x0F);                           // WHO_AM_I
        if (Wire.endTransmission(false) != 0 || Wire.requestFrom((int)addr, 1) != 1) continue;
        uint8_t who = Wire.read();
        if (who != 0x6A && who != 0x69 && who != 0x6C) continue;
        imu_addr = addr;
        break;
    }
    if (imu_addr == 0) { Serial.println("[IMU] No responde: se sigue sin giroscopio"); return; }
    // Datos coherentes + giroscopio a 104 Hz, ±2000 °/s
    if (!imuWrite(0x12, 0x44) || !imuWrite(0x11, 0x4C)) { Serial.println("[IMU] No se pudo configurar"); return; }
    delay(120);
    // Cero del giroscopio: el robot está quieto al encender
    float sum = 0.0f, z;
    int n = 0;
    for (int i = 0; i < 40; i++) {
        if (imuReadZ(z)) { sum += z; n++; }
        delay(10);
    }
    if (n < 20) { Serial.println("[IMU] Lecturas fallidas"); return; }
    gyro_bias_dps = sum / n;
    imu_ok = true;
    Serial.printf("[IMU] Giroscopio listo (0x%02X, cero %.1f grados/s)\n", imu_addr, gyro_bias_dps);
}

bool imuReady() { return imu_ok; }

void gyroRezero() {
    float z;
    if (imu_ok && imuReadZ(z)) gyro_bias_dps += 0.08f * (z - gyro_bias_dps);
}

// El giroscopio se LEE como mucho una vez cada 8 ms; entre medio se devuelve el último
// valor. En un ciclo lo piden varias partes del programa, y cada lectura es tráfico por
// un bus que pasa al lado de los motores: cuanto menos, mejor. Si empieza a fallar, se
// apaga por el resto del encendido en vez de insistir (se sigue sin giroscopio).
static float gyro_last_dps = 0.0f;
static uint32_t gyro_read_ms = 0;
static uint16_t gyro_fail_total = 0;

float gyroZDps() {
    if (!imu_ok) return 0.0f;
    uint32_t now = millis();
    if (gyro_read_ms != 0 && now - gyro_read_ms < 8) return gyro_last_dps;
    gyro_read_ms = now;
    float z;
    if (!imuReadZ(z)) {
        gyro_fail_total++;
        if (++imu_errors > 5) {
            imu_ok = false;
            Wire.end();
            Serial.println("[IMU] Lecturas fallidas: giroscopio apagado hasta el proximo encendido");
        }
        return gyro_last_dps;
    }
    imu_errors = 0;
    gyro_last_dps = z - gyro_bias_dps;
    return gyro_last_dps;
}

uint16_t gyroFailures() { return gyro_fail_total; }

// --- Arranque suave de los motores -----------------------------------------------------
// Arrancar los dos motores de golpe, o invertir uno que todavía gira, pide un pico de
// corriente que hunde la batería y reinicia la placa (reinicio por "voltaje"). Por eso
// lo pedido con setMotors() es un OBJETIVO: una tarea lo aplica cada 10 ms subiendo la
// potencia en rampa y dejando una pausa en cero antes de invertir el sentido.
// Bajar la potencia o parar es inmediato.
struct Side {
    float target = 0.0f;        // Lo pedido
    float applied = 0.0f;       // Lo que hay en el puente H
    uint32_t hold_until_ms = 0; // Pausa en cero antes de invertir
};
static Side side_l, side_r;
static SemaphoreHandle_t motor_lock = nullptr;

static float rampSide(Side &m, uint32_t now, float step) {
    float t = m.target;
    bool reversing = (t > 0.05f && m.applied < -0.05f) || (t < -0.05f && m.applied > 0.05f);
    if (reversing) {
        m.applied = 0.0f;
        m.hold_until_ms = now + MOTOR_REVERSE_PAUSE_MS;
    } else if (now < m.hold_until_ms) {
        m.applied = 0.0f;
    } else if (fabsf(t) <= fabsf(m.applied)) {
        m.applied = t;                                      // Bajar: inmediato
    } else {
        // Subir: en rampa, partiendo de la potencia mínima que ya mueve la rueda
        float from = fmaxf(fabsf(m.applied), fminf(fabsf(t), MOTOR_RAMP_START * MOTOR_SCALE));
        float mag = fminf(fabsf(t), from + step);
        m.applied = t > 0 ? mag : -mag;
    }
    return m.applied;
}

static void motorTask(void *) {
    const uint32_t period_ms = 10;
    const float step = period_ms / (float)MOTOR_RAMP_MS;    // De 0 a 1.0 en MOTOR_RAMP_MS
    for (;;) {
        xSemaphoreTake(motor_lock, portMAX_DELAY);
        uint32_t now = millis();
        float l = rampSide(side_l, now, step);
        float r = rampSide(side_r, now, step);
        driveSide(l, PWM_CH_M1A, PIN_M1A, PWM_CH_M1B, PIN_M1B);
        driveSide(r, PWM_CH_M2A, PIN_M2A, PWM_CH_M2B, PIN_M2B);
        xSemaphoreGive(motor_lock);
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

void hardwareInit() {
#if !BROWNOUT_DETECTOR
    // Un bajón breve de voltaje (pico de los motores) no reinicia la placa
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif
    attachPwm(PWM_CH_M1A, PIN_M1A);
    attachPwm(PWM_CH_M1B, PIN_M1B);
    attachPwm(PWM_CH_M2A, PIN_M2A);
    attachPwm(PWM_CH_M2B, PIN_M2B);
    motor_lock = xSemaphoreCreateMutex();
    stopMotors();
    xTaskCreatePinnedToCore(motorTask, "motores", 3072, nullptr, 3, nullptr, 1);

    pinMode(PIN_SONAR_TRIG, OUTPUT);
    pinMode(PIN_SONAR_ECHO, INPUT);
    digitalWrite(PIN_SONAR_TRIG, LOW);

    pinMode(PIN_IR_FL, INPUT);
    pinMode(PIN_IR_FR, INPUT);
    pinMode(PIN_IR_RL, INPUT);
    pinMode(PIN_IR_RR, INPUT);

    setLedColor(0, 0, 0);
    imuInit();
}

void setMotors(float left, float right) {
    if (INVERT_MOTOR_L) left = -left;
    if (INVERT_MOTOR_R) right = -right;
    xSemaphoreTake(motor_lock, portMAX_DELAY);
    side_l.target = constrain(left * MOTOR_SCALE, -1.0f, 1.0f);
    side_r.target = constrain(right * MOTOR_SCALE, -1.0f, 1.0f);
    xSemaphoreGive(motor_lock);
}

void stopMotors() {
    xSemaphoreTake(motor_lock, portMAX_DELAY);
    side_l.target = side_l.applied = 0.0f;
    side_r.target = side_r.applied = 0.0f;
    writeMotorPwm(PWM_CH_M1A, PIN_M1A, 0);
    writeMotorPwm(PWM_CH_M1B, PIN_M1B, 0);
    writeMotorPwm(PWM_CH_M2A, PIN_M2A, 0);
    writeMotorPwm(PWM_CH_M2B, PIN_M2B, 0);
    xSemaphoreGive(motor_lock);
}

float readUltrasonicCm() {
    digitalWrite(PIN_SONAR_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_SONAR_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_SONAR_TRIG, LOW);
    long duration_us = pulseIn(PIN_SONAR_ECHO, HIGH, 25000);
    if (duration_us == 0) return 999.0f;
    return duration_us * 0.0343f / 2.0f;
}

void readFloorSensors(int &fl, int &fr, int &rl, int &rr) {
    fl = analogRead(PIN_IR_FL);
    fr = analogRead(PIN_IR_FR);
    rl = analogRead(PIN_IR_RL);
    rr = analogRead(PIN_IR_RR);
}

void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
    neopixelWrite(PIN_NEOPIXEL, r * 0.35f, g * 0.35f, b * 0.35f);
}
