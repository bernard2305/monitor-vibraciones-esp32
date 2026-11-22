// ==========================================
// ESCLAVO RECEPTOR — SD + DS3231 + Mutex
// v3.0 — Buffer en lote, sin Serial en ISR
// ==========================================
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <sys/time.h>
#include "RTClib.h"

const int CS_SD_PIN = 5;
const int BOTON_PIN = 4;
const int LED_PIN   = 2;

RTC_DS3231 rtc;
File archivoDatos;

typedef struct {
    uint8_t  master_id;
    uint8_t  sensor_id;
    uint32_t timestamp_ms;
    float    accel_x, accel_y, accel_z;
    float    gyro_x,  gyro_y,  gyro_z;
} SensorData;

#define TAMANO_BUFFER 500
SensorData bufferDatos[TAMANO_BUFFER];
volatile int head = 0;
volatile int tail = 0;

SemaphoreHandle_t bufferMutex;

volatile int datosDescartados = 0;
bool grabando          = false;
bool estadoAnteriorBtn = HIGH;
unsigned long ultimoRebote = 0;
const unsigned long REBOTE = 50;
int contadorFlush = 0;

// ── ISR: SIN Serial.println ───────────────────────────────────────
void onDataReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len != sizeof(SensorData)) return;
    if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(5))) {
        int nextHead = (head + 1) % TAMANO_BUFFER;
        if (nextHead != tail) {
            memcpy(&bufferDatos[head], data, sizeof(SensorData));
            head = nextHead;
        } else {
            datosDescartados++;   // solo contar, nunca imprimir aquí
        }
        xSemaphoreGive(bufferMutex);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(BOTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    bufferMutex = xSemaphoreCreateMutex();

    Wire.begin();
    if (!rtc.begin()) {
        Serial.println("Error DS3231");
        while (1) delay(10);
    }
    if (rtc.lostPower()) {
        Serial.println("RTC sin energia. Reajustando...");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    DateTime ahora = rtc.now();
    struct timeval tv = { .tv_sec = ahora.unixtime(), .tv_usec = 0 };
    settimeofday(&tv, NULL);
    Serial.println("RTC OK.");

    Serial.print("Inicializando SD... ");
    if (!SD.begin(CS_SD_PIN)) {
        Serial.println("FALLO.");
        while (1) delay(10);
    }
    Serial.println("OK.");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(onDataReceive);

    Serial.println("=== Esclavo LISTO. Presiona boton para grabar. ===");
}

void loop() {
    // ── Debounce del boton ────────────────────────────────────────
    int btnActual = digitalRead(BOTON_PIN);
    if (btnActual != estadoAnteriorBtn) ultimoRebote = millis();

    if ((millis() - ultimoRebote) > REBOTE && btnActual == LOW) {
        if (!grabando) {
            DateTime ahora = rtc.now();
            char nombreArchivo[30];
            sprintf(nombreArchivo, "/sismo_%02d%02d%02d.csv",
                    ahora.hour(), ahora.minute(), ahora.second());

            archivoDatos = SD.open(nombreArchivo, FILE_WRITE);
            if (archivoDatos) {
                grabando = true;
                contadorFlush = 0;
                datosDescartados = 0;
                digitalWrite(LED_PIN, HIGH);
                Serial.print(">> GRABANDO en: ");
                Serial.println(nombreArchivo);
                archivoDatos.println("Timestamp,Sensor_ID,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z");
            } else {
                Serial.println("Error abriendo archivo en SD.");
            }
        } else {
            grabando = false;
            digitalWrite(LED_PIN, LOW);
            if (archivoDatos) archivoDatos.close();
            Serial.print(">> CAPTURA DETENIDA. Archivo cerrado.");
            Serial.print(" Datos descartados: ");
            Serial.println(datosDescartados);
        }
        while (digitalRead(BOTON_PIN) == LOW) delay(10);
    }
    estadoAnteriorBtn = btnActual;

    // ── Drenar buffer EN LOTE (siempre, grabe o no) ───────────────
    const int MAX_BATCH = 15;
    for (int i = 0; i < MAX_BATCH; i++) {
        SensorData dato;
        bool hayDato = false;

        if (xSemaphoreTake(bufferMutex, 0)) {
            if (head != tail) {
                dato = bufferDatos[tail];
                tail = (tail + 1) % TAMANO_BUFFER;
                hayDato = true;
            }
            xSemaphoreGive(bufferMutex);
        }

        if (!hayDato) break;
        if (!grabando) continue;  // descartar si no se graba

        struct timeval tv_now;
        gettimeofday(&tv_now, NULL);
        DateTime now(tv_now.tv_sec);
        char ts[28];
        sprintf(ts, "%04d-%02d-%02d %02d:%02d:%02d.%06ld",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second(),
            (long)tv_now.tv_usec);

        archivoDatos.print(ts);  archivoDatos.print(",");

        if      (dato.sensor_id == 1) archivoDatos.print("M1_S1");
        else if (dato.sensor_id == 2) archivoDatos.print("M1_S2");
        else if (dato.sensor_id == 3) archivoDatos.print("M2_S3");
        else if (dato.sensor_id == 4) archivoDatos.print("M2_S4");
        else                          archivoDatos.print("UNKN");
        archivoDatos.print(",");

        archivoDatos.print(dato.accel_x, 5); archivoDatos.print(",");
        archivoDatos.print(dato.accel_y, 5); archivoDatos.print(",");
        archivoDatos.print(dato.accel_z, 5); archivoDatos.print(",");
        archivoDatos.print(dato.gyro_x,  5); archivoDatos.print(",");
        archivoDatos.print(dato.gyro_y,  5); archivoDatos.print(",");
        archivoDatos.println(dato.gyro_z, 5);

        if (++contadorFlush >= 20) {
            archivoDatos.flush();
            contadorFlush = 0;
        }
    }
}
