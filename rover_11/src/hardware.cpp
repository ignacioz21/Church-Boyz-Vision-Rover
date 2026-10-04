#include "../include/hardware.h"
#include "../include/config.h"

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

void hardwareInit() {
    attachPwm(PWM_CH_M1A, PIN_M1A);
    attachPwm(PWM_CH_M1B, PIN_M1B);
    attachPwm(PWM_CH_M2A, PIN_M2A);
    attachPwm(PWM_CH_M2B, PIN_M2B);
    stopMotors();

    pinMode(PIN_SONAR_TRIG, OUTPUT);
    pinMode(PIN_SONAR_ECHO, INPUT);
    digitalWrite(PIN_SONAR_TRIG, LOW);

    pinMode(PIN_IR_FL, INPUT);
    pinMode(PIN_IR_FR, INPUT);
    pinMode(PIN_IR_RL, INPUT);
    pinMode(PIN_IR_RR, INPUT);

    setLedColor(0, 0, 0);
}

void setMotors(float left, float right) {
    if (INVERT_MOTOR_L) left = -left;
    if (INVERT_MOTOR_R) right = -right;
    driveSide(constrain(left, -1.0f, 1.0f), PWM_CH_M1A, PIN_M1A, PWM_CH_M1B, PIN_M1B);
    driveSide(constrain(right, -1.0f, 1.0f), PWM_CH_M2A, PIN_M2A, PWM_CH_M2B, PIN_M2B);
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
