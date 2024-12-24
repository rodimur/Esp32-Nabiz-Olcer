#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <FirebaseESP32.h>

//----------------------------------------SH1106 OLED initialization
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // Reset pin not used
//----------------------------------------

#define PulseSensor_PIN 34
#define Buzzer_PIN 25 // Buzzer bağlı olduğu pin

// Firebase configuration
FirebaseConfig config;
FirebaseAuth auth;

// WiFi credentials
const char* WIFI_SSID = "Ruru";
const char* WIFI_PASSWORD = "afli9322";

FirebaseData firebaseData;

unsigned long previousMillisGetHB = 0; // Store the last time Millis (to get Heartbeat) was updated.
unsigned long previousMillisResultHB = 0; // Store the last time Millis (to get BPM) was updated.

const long intervalGetHB = 20; // Interval for reading heart rate (Heartbeat) = 20ms.
const long intervalResultHB = 1000; // The reading interval for the result of the Heart Rate calculation.

int timer_Get_BPM = 0;
int PulseSensorSignal; // Variable to hold the signal value from the sensor.
int UpperThreshold = 320; // Signal threshold to "count as a beat".
int LowerThreshold = 300;

int cntHB = 0; // Counter for heartbeats.
boolean ThresholdStat = true; // Trigger for calculating heartbeats.
int BPMval = 0; // BPM value.

int x = 0; // x-axis for graph.
int y = 0; // y-axis for graph.
int lastx = 0; // Previous x-axis value.
int lasty = 0; // Previous y-axis value.

//----------------------------------------'Heart_Icon', 16x16px
const unsigned char Heart_Icon [] PROGMEM = {
  0x00, 0x00, 0x18, 0x30, 0x3c, 0x78, 0x7e, 0xfc, 0xff, 0xfe, 0xff, 0xfe, 0xee, 0xee, 0xd5, 0x56, 
  0x7b, 0xbc, 0x3f, 0xf8, 0x1f, 0xf0, 0x0f, 0xe0, 0x07, 0xc0, 0x03, 0x80, 0x01, 0x00, 0x00, 0x00
};
//----------------------------------------

// Mutex for shared BPM value
SemaphoreHandle_t xBPM_Mutex;

// Task handles
TaskHandle_t MeasureTaskHandle;
TaskHandle_t FirebaseTaskHandle;

void DrawGraph() {
  if (x > 127) {
    u8g2.clearBuffer(); // Clear display when graph reaches end of screen width.
    x = 0;
    lastx = 0;
  }

  int ySignal = PulseSensorSignal;
  ySignal = constrain(ySignal, 350, 850);
  int ySignalMap = map(ySignal, 350, 850, 0, 40);
  y = 40 - ySignalMap;

  u8g2.drawLine(lastx, lasty, x, y);
  u8g2.sendBuffer();

  lastx = x;
  lasty = y;
  x++;
}

void MeasureHeartRate(void* parameter) {
  while (true) {
    unsigned long currentMillisGetHB = millis();

    if (currentMillisGetHB - previousMillisGetHB >= intervalGetHB) {
      previousMillisGetHB = currentMillisGetHB;
      PulseSensorSignal = analogRead(PulseSensor_PIN); // Read the PulseSensor's value.

      if (PulseSensorSignal > UpperThreshold && ThresholdStat == true) {
        cntHB++;
        ThresholdStat = false;
      }

      if (PulseSensorSignal < LowerThreshold) {
        ThresholdStat = true;
      }

      DrawGraph();
    }

    unsigned long currentMillisResultHB = millis();

    if (currentMillisResultHB - previousMillisResultHB >= intervalResultHB) {
      previousMillisResultHB = currentMillisResultHB;
      timer_Get_BPM++;

      if (timer_Get_BPM > 10) {
        timer_Get_BPM = 1;

        // Critical section to update BPM value
        xSemaphoreTake(xBPM_Mutex, portMAX_DELAY);
        BPMval = cntHB * 6; // Calculate BPM
        cntHB = 0;
        xSemaphoreGive(xBPM_Mutex);

        u8g2.clearBuffer();
        u8g2.drawXBMP(0, 48, 16, 16, Heart_Icon);
        u8g2.drawLine(0, 43, 127, 43);
        u8g2.setFont(u8g2_font_ncenB14_tr);
        u8g2.setCursor(20, 58);
        u8g2.print(BPMval);
        u8g2.print(" BPM");
        u8g2.sendBuffer();

        // Buzzer kontrolü
        if (BPMval < 50 || BPMval > 125) {
          digitalWrite(Buzzer_PIN, HIGH); // Buzzer çalıştır
        } else {
          digitalWrite(Buzzer_PIN, LOW); // Buzzer kapat
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to allow FreeRTOS scheduler to run
  }
}

void SendToFirebase(void* parameter) {
  while (true) {
    // Critical section to read BPM value
    xSemaphoreTake(xBPM_Mutex, portMAX_DELAY);
    int currentBPM = BPMval;
    xSemaphoreGive(xBPM_Mutex);

    // Firebase data send
    if (Firebase.setInt(firebaseData, "/heart_rate", currentBPM)) {
      Serial.print("Firebase'e gönderildi: ");
      Serial.println(currentBPM);
    } else {
      Serial.print("Firebase gönderim hatası: ");
      Serial.println(firebaseData.errorReason());
    }

    vTaskDelay(10000 / portTICK_PERIOD_MS); // 10 saniyede bir gönderim
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  analogReadResolution(10);
  
  pinMode(Buzzer_PIN, OUTPUT); // Buzzer pinini çıkış olarak ayarla
  digitalWrite(Buzzer_PIN, LOW); // Başlangıçta buzzer kapalı olsun

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(30, 20, "ESP32");
  u8g2.drawStr(10, 40, "HEART MONITOR");
  u8g2.sendBuffer();
  delay(2000);

  u8g2.clearBuffer();
  u8g2.drawLine(0, 43, 127, 43);
  u8g2.setCursor(10, 58);
  u8g2.print("HeartBeat");
  u8g2.sendBuffer();

  // WiFi bağlantısı
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("WiFi bağlantısı bekleniyor...");
  }
  Serial.println("WiFi'ye bağlanıldı!");

  // Firebase ayarları
  config.database_url = "https://nabizolcer-2ff75-default-rtdb.firebaseio.com/";
  config.signer.tokens.legacy_token = "AIzaSyDA5HRw5Vw2hBQ3HFcIxpB7AxbjIDAmoGs";

  Firebase.begin(&config, &auth);

  // Mutex oluştur
  xBPM_Mutex = xSemaphoreCreateMutex();

  // Görevleri oluştur
  xTaskCreatePinnedToCore(MeasureHeartRate, "MeasureHeartRate", 10000, NULL, 1, &MeasureTaskHandle, 0); // Core 0
  xTaskCreatePinnedToCore(SendToFirebase, "SendToFirebase", 10000, NULL, 1, &FirebaseTaskHandle, 1);    // Core 1
}

void loop() {
  // Ana döngü kullanılmıyor, tüm işler FreeRTOS görevlerinde
}