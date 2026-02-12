#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <TinyGPSPlus.h>

// ================= PINES =================
#define DHTPIN 13
#define DHTTYPE DHT11

#define SW420_PIN 21
#define BUTTON_PIN 25
#define BUZZER_PIN 23

#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

#define GPS_RX 33
#define GPS_TX 32

#define LORA_FREQ 915E6
#define SEND_INTERVAL 10000

// ================= OBJETOS =================
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST);
DHT dht(DHTPIN, DHTTYPE);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// ================= ESTRUCTURA =================
typedef struct {
  float temperature;
  float humidity;
  bool motion;
  bool help;
  double lat;
  double lng;
  int satellites;
  bool gpsValid;
} SensorData;

SensorData currentData;

// ================= FreeRTOS =================
TaskHandle_t TaskSensorsHandle = NULL;
TaskHandle_t TaskLoRaHandle = NULL;
SemaphoreHandle_t dataMutex;

// ======================================================
void TaskSensors(void *pvParameters);
void TaskLoRa(void *pvParameters);
// ======================================================

void setup() {

  Serial.begin(115200);
  delay(1500);

  Serial.println("\n===== SISTEMA LoRa TRANSMISOR =====");

  // ===== OLED =====
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(50);
  digitalWrite(OLED_RST, HIGH);

  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Error OLED");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("OLED OK");
  display.display();
  delay(1000);

  // ===== PINES =====
  pinMode(SW420_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  dht.begin();

  // ===== GPS =====
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS iniciado");

  // ===== LoRa =====
  SPI.begin(5,19,27,18);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("Error LoRa");
    while (true);
  }

  Serial.println("LoRa OK");

  currentData = {0,0,false,false,0,0,0,false};
  dataMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(TaskSensors, "TaskSensors", 4096, NULL, 1, &TaskSensorsHandle, 1);
  xTaskCreatePinnedToCore(TaskLoRa, "TaskLoRa", 4096, NULL, 1, &TaskLoRaHandle, 0);

  Serial.println("Sistema iniciado correctamente");
}

// ======================================================
// ================== TASK SENSORES =====================
// ======================================================
void TaskSensors(void *pvParameters) {

  for(;;) {

    float temp = dht.readTemperature();
    float hum = dht.readHumidity();
    bool motion = digitalRead(SW420_PIN);
    bool helpBtn = digitalRead(BUTTON_PIN) == LOW;

    digitalWrite(BUZZER_PIN, helpBtn);  // Buzzer simple ON/OFF

    // ===== GPS LECTURA CONTINUA =====
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }

    double lat = 0;
    double lng = 0;
    int sats = gps.satellites.value();
    bool valid = gps.location.isValid();

    if (gps.location.isUpdated()) {
      lat = gps.location.lat();
      lng = gps.location.lng();
    }

    if (!isnan(temp) && !isnan(hum)) {

      xSemaphoreTake(dataMutex, portMAX_DELAY);

      currentData.temperature = temp;
      currentData.humidity = hum;
      currentData.motion = motion;
      currentData.help = helpBtn;
      currentData.lat = lat;
      currentData.lng = lng;
      currentData.satellites = sats;
      currentData.gpsValid = valid;

      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// ======================================================
// ================== TASK LORA =========================
// ======================================================
void TaskLoRa(void *pvParameters) {

  unsigned long lastSend = 0;

  for(;;) {

    SensorData localCopy;

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    localCopy = currentData;
    xSemaphoreGive(dataMutex);

    bool sendNow = false;

    if (localCopy.motion || localCopy.help) sendNow = true;
    if (millis() - lastSend > SEND_INTERVAL) sendNow = true;

    if (sendNow) {

      String json = "{";
      json += "\"net_id\":1,";
      json += "\"humidity\":" + String(localCopy.humidity,1) + ",";
      json += "\"temperature\":" + String(localCopy.temperature,1) + ",";
      json += "\"help\":" + String(localCopy.help) + ",";
      json += "\"motion\":" + String(localCopy.motion) + ",";
      json += "\"lat\":" + String(localCopy.lat,6) + ",";
      json += "\"lng\":" + String(localCopy.lng,6);
      json += "}";

      LoRa.beginPacket();
      LoRa.print(json);
      LoRa.endPacket();

      Serial.println("\n--- LoRa TX ---");
      Serial.println(json);

      lastSend = millis();
    }

    // ===== OLED =====
    display.clearDisplay();
    display.setCursor(0,0);

    display.println("TEMP: " + String(localCopy.temperature));
    display.println("HUM: " + String(localCopy.humidity));
    display.println("MOV: " + String(localCopy.motion));
    display.println("HELP: " + String(localCopy.help));

    display.println("----------------");

    if (localCopy.gpsValid) {
      display.println("GPS OK");
      display.print("Lat:");
      display.println(localCopy.lat,4);
      display.print("Lng:");
      display.println(localCopy.lng,4);
    } else {
      display.println("GPS BUSCANDO...");
      display.print("Sat: ");
      display.println(localCopy.satellites);
    }

    display.display();

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void loop() {
  // FreeRTOS maneja todo
}
