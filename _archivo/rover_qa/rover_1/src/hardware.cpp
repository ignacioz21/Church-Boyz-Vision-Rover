#include "../include/hardware.h"
#include "../include/config.h"
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_LSM6DS3TRC.h>

// Objeto del sensor de color, modo "seguro" (si no está, no bloquea)

static Adafruit_LSM6DS3TRC lsm6ds;
static bool imuConnected = false;
static float imu_heading_raw = 0.0f;
static float imu_drift = 0.0f;
static float camera_offset = 0.0f; 
static unsigned long last_imu_time = 0;

void imuTask(void *pvParameters) {
    last_imu_time = micros();
    while (true) {
        if (imuConnected) {
            unsigned long now = micros();
            float dt = (now - last_imu_time) / 1000000.0f;
            last_imu_time = now;
            
            sensors_event_t accel, gyro, temp;
            lsm6ds.getEvent(&accel, &gyro, &temp);
            
            float gz_deg = (gyro.gyro.z - imu_drift) * 57.2958f; 
            imu_heading_raw -= gz_deg * dt; 
            
            while (imu_heading_raw > 180.0f) imu_heading_raw -= 360.0f;
            while (imu_heading_raw <= -180.0f) imu_heading_raw += 360.0f;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void imuInit() {
    if (lsm6ds.begin_I2C()) {
        Serial.println("[HARDWARE] IMU LSM6DS3TRC detectado.");
        lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
        lsm6ds.setGyroDataRate(LSM6DS_RATE_416_HZ);
        imuConnected = true;
        
        Serial.println("[HARDWARE] Calibrando IMU...");
        float drift_sum = 0;
        for (int i = 0; i < 100; i++) {
            sensors_event_t accel, gyro, temp;
            lsm6ds.getEvent(&accel, &gyro, &temp);
            drift_sum += gyro.gyro.z;
            delay(10);
        }
        imu_drift = drift_sum / 100.0f;
        
        xTaskCreatePinnedToCore(imuTask, "IMU_Task", 4096, NULL, 5, NULL, 1);
    } else {
        Serial.println("[HARDWARE] IMU no detectado.");
    }
}

float getImuHeading() {
    float fused = imu_heading_raw + camera_offset;
    while (fused > 180.0f) fused -= 360.0f;
    while (fused <= -180.0f) fused += 360.0f;
    return fused;
}

void syncImuToCamera(float camera_heading) {
    camera_offset = camera_heading - imu_heading_raw;
}

static Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
static bool colorSensorConnected = false;

// Compatibilidad transparente entre ESP32 Core v2.x y v3.x
static void writeMotorPwm(int channel, int pin, int duty) {
    duty = constrain(duty, 0, 1023);
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    ledcWrite(pin, duty);
#else
    ledcWrite(channel, duty);
#endif
}

void hardwareInit() {
    // 1. Configuración de canales PWM para puentes H
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

    // 2. Sensor Ultrasónico HC-SR04
    pinMode(PIN_SONAR_TRIG, OUTPUT);
    pinMode(PIN_SONAR_ECHO, INPUT);
    digitalWrite(PIN_SONAR_TRIG, LOW);

    // 3. Sensores Infrarrojos de Piso (4IR)
    pinMode(PIN_IR_FL, INPUT);
    pinMode(PIN_IR_FR, INPUT);
    pinMode(PIN_IR_RL, INPUT);
    pinMode(PIN_IR_RR, INPUT);

    // 4. LED RGB NeoPixel
    setLedColor(0, 0, 0);

    // 5. Sensor de Color I2C (TCS34725)
    Wire.begin(21, 22); // Pines I2C estándar del ESP32
    if (tcs.begin()) {
        Serial.println("[HARDWARE] Sensor TCS34725 detectado y listo.");
        colorSensorConnected = true;
    } else {
        Serial.println("[HARDWARE] Sensor TCS34725 no detectado. (Saltando validaciones de color)");
        colorSensorConnected = false;
    }
    
    imuInit();
}

// Variables dinámicas de polaridad (permiten invertir en caliente desde Serial)
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

static float current_trim_l = MOTOR_TRIM_L;
static float current_trim_r = MOTOR_TRIM_R;

void setMotorTrim(float left, float right) {
    current_trim_l = left;
    current_trim_r = right;
}
float getMotorTrimL() { return current_trim_l; }
float getMotorTrimR() { return current_trim_r; }

void setMotors(float left, float right) {
    if (current_invert_l) left = -left;
    if (current_invert_r) right = -right;

    // Aplicar multiplicador de calibración de balance (Trim)
    left *= current_trim_l;
    right *= current_trim_r;

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

    long duration_us = pulseIn(PIN_SONAR_ECHO, HIGH, 25000);
    if (duration_us == 0) {
        return 999.0f;
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
    uint8_t br = (uint8_t)(r * 0.35f);
    uint8_t bg = (uint8_t)(g * 0.35f);
    uint8_t bb = (uint8_t)(b * 0.35f);
    neopixelWrite(PIN_NEOPIXEL, br, bg, bb);
}

void setLedForState(RoverFsmState state) {
    switch (state) {
        case STATE_BOOT_INIT:         setLedColor(255, 255, 255); break;
        case STATE_HEALTH_CHECK:      setLedColor(255, 200, 0);   break;
        case STATE_IDLE:              setLedColor(0, 0, 255);     break;
        case STATE_READY_PLAN:        setLedColor(0, 255, 255);   break;
        case STATE_NAV_PREAPPROACH:   setLedColor(0, 255, 0);     break;
        case STATE_PUSH_TO_DEPOT:     setLedColor(0, 255, 128);   break;
        case STATE_SAFE_RETREAT:      setLedColor(255, 0, 255);   break;
        case STATE_MISSION_COMPLETE:  setLedColor(0, 255, 255);   break;
        case STATE_OBSTACLE_AVOID:    setLedColor(255, 100, 0);   break;
        case STATE_FAILSAFE_HOLD:     setLedColor(255, 0, 0);     break;
    }
}

bool verifyCubeColor(CubeColor expected_color) {
    if (!colorSensorConnected) {
        // Si no hay sensor, devolver false para obligar al fallback de telemetria
        return false; 
    }

    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    
    // Evitar división por 0
    if (c == 0) return false;

    // Normalizar a porcentaje
    float red = (float)r / c;
    float green = (float)g / c;
    float blue = (float)b / c;

    CubeColor detected = COLOR_UNKNOWN;

    // Lógica simple de mayor predominancia
    if (red > green && red > blue && red > 0.40f) {
        detected = COLOR_RED;
    } else if (green > red && green > blue && green > 0.35f) {
        detected = COLOR_GREEN;
    } else if (blue > red && blue > green && blue > 0.35f) {
        detected = COLOR_BLUE;
    }

    Serial.printf("[COLOR] Lectura R:%.2f G:%.2f B:%.2f -> Detectado: %d (Esperado: %d)\n", red, green, blue, detected, expected_color);

    return (detected == expected_color);
}
