#include <LoRa.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// ================= LORA =================
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26
#define LORA_FREQ 915E6

// ================= OLED =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

// ================= ESP-NOW =================
uint8_t broadcastAddress[] = {0xc8, 0x2e, 0x18, 0x90, 0xb8, 0x40};

typedef struct {
  float humidity;
  float temperature;
  int help;
  int motion;
  float lat;
  float lng;
  bool newData;
} SensorData;

SensorData sharedData;
SemaphoreHandle_t dataMutex;

// ===== FreeRTOS Handles =====
TaskHandle_t TaskLoRaHandle;
TaskHandle_t TaskDisplayHandle;
TaskHandle_t TaskESPNowHandle;

esp_now_peer_info_t peerInfo;

// ======================================================
// ===================== SETUP ==========================
// ======================================================

void setup() {

  Serial.begin(115200);
  delay(2000);

  // -------- OLED --------
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(50);
  digitalWrite(OLED_RST, HIGH);

  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED ERROR");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("RECEPTOR RTOS");
  display.display();

  // -------- LORA --------
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa ERROR");
    while (true);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();

  Serial.println("LoRa OK");

  // -------- ESP-NOW --------
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW ERROR");
    while (true);
  }

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_now_add_peer(&peerInfo);

  // -------- Inicializar datos --------
  sharedData = {0,0,0,0,0,0,false};
  dataMutex = xSemaphoreCreateMutex();

  // -------- Crear tareas --------
  xTaskCreatePinnedToCore(TaskLoRa, "TaskLoRa", 4096, NULL, 2, &TaskLoRaHandle, 0);
  xTaskCreatePinnedToCore(TaskDisplay, "TaskDisplay", 4096, NULL, 1, &TaskDisplayHandle, 1);
  xTaskCreatePinnedToCore(TaskESPNow, "TaskESPNow", 4096, NULL, 1, &TaskESPNowHandle, 1);

  Serial.println("Sistema RTOS listo");
}

// ======================================================
// ================= TASK LORA ==========================
// ======================================================

void TaskLoRa(void *pvParameters) {

  StaticJsonDocument<512> doc;

  for (;;) {

    int packetSize = LoRa.parsePacket();

    if (packetSize > 0) {

      String incoming = "";

      while (LoRa.available()) {
        incoming += (char)LoRa.read();
      }

      Serial.println("JSON RX:");
      Serial.println(incoming);

      DeserializationError error = deserializeJson(doc, incoming);

      if (!error) {

        xSemaphoreTake(dataMutex, portMAX_DELAY);

        sharedData.humidity = doc["humidity"] | 0.0;
        sharedData.temperature = doc["temperature"] | 0.0;
        sharedData.help = doc["help"] | 0;
        sharedData.motion = doc["motion"] | 0;
        sharedData.lat = doc["lat"] | 0.0;
        sharedData.lng = doc["lng"] | 0.0;
        sharedData.newData = true;

        xSemaphoreGive(dataMutex);

        Serial.println("Datos actualizados");
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ======================================================
// ================= TASK OLED ==========================
// ======================================================

void TaskDisplay(void *pvParameters) {

  SensorData localCopy;

  for (;;) {

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    localCopy = sharedData;
    xSemaphoreGive(dataMutex);

    display.clearDisplay();
    display.setCursor(0,0);

    display.print("T: "); display.println(localCopy.temperature);
    display.print("H: "); display.println(localCopy.humidity);
    display.print("Help: "); display.println(localCopy.help);
    display.print("Mov: "); display.println(localCopy.motion);
    display.print("Lat: "); display.println(localCopy.lat,4);
    display.print("Lng: "); display.println(localCopy.lng,4);

    display.display();

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// ======================================================
// ================= TASK ESPNOW ========================
// ======================================================

void TaskESPNow(void *pvParameters) {

  SensorData localCopy;

  for (;;) {

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    localCopy = sharedData;
    sharedData.newData = false;
    xSemaphoreGive(dataMutex);

    if (localCopy.newData) {

      esp_now_send(broadcastAddress,
                   (uint8_t *)&localCopy,
                   sizeof(localCopy));

      Serial.println("Enviado por ESP-NOW");
    }

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void loop() {
  // No se usa
}

