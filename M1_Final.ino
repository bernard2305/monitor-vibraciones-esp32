// ==========================================
// MAESTRO 1 — 2x MPU6050 + ESP-NOW
// v2.1 — Delay calibracion + accel + gyro
// Sensores: M1_S1 (0x68) y M1_S2 (0x69)
// ==========================================
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>

#define MASTER_ID    1
#define SENSOR_ID_A  1        // M1_S1 — AD0 = GND  (0x68)
#define SENSOR_ID_B  2        // M1_S2 — AD0 = VCC  (0x69)
#define LED_PIN      2
#define LED_ON_PIN   4

uint8_t macEsclavo[] = {0xA8, 0x42, 0xE3, 0x5B, 0x24, 0xC8};

typedef struct {
    uint8_t  master_id;
    uint8_t  sensor_id;
    uint32_t timestamp_ms;
    float    accel_x, accel_y, accel_z;
    float    gyro_x,  gyro_y,  gyro_z;
} SensorData;

MPU6050 mpuA(0x68);   // S1 — AD0 = GND
MPU6050 mpuB(0x69);   // S2 — AD0 = VCC

// Offsets giroscopio (raw counts)
float offGxA, offGyA, offGzA;
float offGxB, offGyB, offGzB;

// Offsets acelerometro (raw counts)
float offAxA, offAyA, offAzA;
float offAxB, offAyB, offAzB;

esp_now_peer_info_t peerInfo;
SensorData paquete;

const int   MUESTRAS_CAL = 200;
const float ESCALA_ACCEL = 16384.0;
const float ESCALA_GYRO  = 131.0;

// ── Calibra giroscopio Y acelerómetro ────────────────────────────
void calibrar(MPU6050 &mpu,
              float &ox, float &oy, float &oz,   // gyro offsets
              float &oax, float &oay, float &oaz) // accel offsets
{
    double sGx=0, sGy=0, sGz=0;
    double sAx=0, sAy=0, sAz=0;

    for (int i = 0; i < MUESTRAS_CAL; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sGx += gx; sGy += gy; sGz += gz;
        sAx += ax; sAy += ay; sAz += az;
        delay(5);
    }

    // Gyro: promedio directo (en reposo debe ser 0)
    ox  = sGx / MUESTRAS_CAL;
    oy  = sGy / MUESTRAS_CAL;
    oz  = sGz / MUESTRAS_CAL;

    // Accel X, Y: promedio directo (en reposo deben ser 0)
    oax = sAx / MUESTRAS_CAL;
    oay = sAy / MUESTRAS_CAL;
    // Accel Z: restar 1g (16384 counts) para que el offset sea solo el error
    oaz = (sAz / MUESTRAS_CAL) - ESCALA_ACCEL;
}

void onDataSent(const uint8_t *mac, esp_now_send_status_t status) { }

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    pinMode(LED_ON_PIN, OUTPUT);
    digitalWrite(LED_ON_PIN, LOW);  // apagado mientras calibra
    digitalWrite(LED_PIN, LOW);

    Wire.begin();

    mpuA.initialize();
    mpuB.initialize();

    if (!mpuA.testConnection()) Serial.println("MPU S1 no responde!");
    if (!mpuB.testConnection()) Serial.println("MPU S2 no responde!");

    // ── Countdown ────────────────────────────────────────────────
    Serial.println(">>> PON EL DISPOSITIVO EN REPOSO");
    Serial.println(">>> Calibrando en 3 segundos, NO LO MUEVAS...");
    for (int i = 3; i > 0; i--) {
        Serial.print(i); Serial.println("...");
        digitalWrite(LED_PIN, HIGH); delay(250);
        digitalWrite(LED_PIN, LOW);  delay(750);
    }

    // ── Calibración gyro + accel ──────────────────────────────────
    Serial.println("Calibrando S1...");
    calibrar(mpuA, offGxA, offGyA, offGzA, offAxA, offAyA, offAzA);

    Serial.println("Calibrando S2...");
    calibrar(mpuB, offGxB, offGyB, offGzB, offAxB, offAyB, offAzB);

    // Imprimir offsets para verificar
    Serial.printf("S1 Gyro  off: X=%.2f  Y=%.2f  Z=%.2f\n", offGxA, offGyA, offGzA);
    Serial.printf("S1 Accel off: X=%.2f  Y=%.2f  Z=%.2f\n", offAxA, offAyA, offAzA);
    Serial.printf("S2 Gyro  off: X=%.2f  Y=%.2f  Z=%.2f\n", offGxB, offGyB, offGzB);
    Serial.printf("S2 Accel off: X=%.2f  Y=%.2f  Z=%.2f\n", offAxB, offAyB, offAzB);

    // 3 destellos rapidos = OK
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH); delay(100);
        digitalWrite(LED_PIN, LOW);  delay(100);
    }
    Serial.println("Calibracion OK. Iniciando transmision.");

    // ── ESP-NOW ───────────────────────────────────────────────────
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error ESP-NOW");
        return;
    }
    esp_now_register_send_cb(onDataSent);

    memcpy(peerInfo.peer_addr, macEsclavo, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Error agregando peer");
        return;
    }
    
    // LED de "encendido" fijo al terminar calibración
    digitalWrite(LED_ON_PIN, HIGH);
    Serial.println("Sistema listo. LED encendido.");
}

void enviarSensor(MPU6050 &mpu, uint8_t sensorId,
                  float ox,  float oy,  float oz,
                  float oax, float oay, float oaz)
{
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    paquete.master_id    = MASTER_ID;
    paquete.sensor_id    = sensorId;
    paquete.timestamp_ms = millis();

    // Accel: resta offset y convierte a g
    paquete.accel_x = (ax - oax) / ESCALA_ACCEL;
    paquete.accel_y = (ay - oay) / ESCALA_ACCEL;
    paquete.accel_z = (az - oaz) / ESCALA_ACCEL;

    // Gyro: resta offset y convierte a °/s
    paquete.gyro_x  = (gx - ox) / ESCALA_GYRO;
    paquete.gyro_y  = (gy - oy) / ESCALA_GYRO;
    paquete.gyro_z  = (gz - oz) / ESCALA_GYRO;

    esp_now_send(macEsclavo, (uint8_t *)&paquete, sizeof(paquete));
}

void loop() {
    unsigned long inicio = millis();

    enviarSensor(mpuA, SENSOR_ID_A,
                 offGxA, offGyA, offGzA,
                 offAxA, offAyA, offAzA);

    enviarSensor(mpuB, SENSOR_ID_B,
                 offGxB, offGyB, offGzB,
                 offAxB, offAyB, offAzB);

    // Mantener 50 Hz
    long restante = 20 - (long)(millis() - inicio);
    if (restante > 0) delay(restante);
}
