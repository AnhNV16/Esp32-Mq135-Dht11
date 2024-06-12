#include <WiFi.h>
#include <ThingSpeak.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <HTTPClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// WiFi
const char* ssid = "Redmi K50";
const char* password = "0379165877";

// ThingSpeak
unsigned long channelID = 2544269;
const char* apiKey = "WWJAJ6VMT8646IKN";

// Google Sheet URL
String Web_App_URL = "https://script.google.com/macros/s/AKfycby0OsaYJB5j6xA-1jM8X_fW_lwbl2LQhW5QkutN9ivpgIoFbOE1u26GLMPjDRoVQLC5/exec";

String Status_Read_Sensor = "";
int value1;
int value2;
int value3;

const int ANALOG_PIN = 32; // Chân analog của MQ135
const int DHTPIN = 4; // Chân data của DHT11
const int DHTTYPE = DHT11; // Loại cảm biến DHT11
DHT dht(DHTPIN, DHTTYPE); // Khai báo đối tượng cảm biến DHT11

LiquidCrystal_I2C lcd(0x27, 16, 2); 

// Khởi tạo WifiClient
WiFiClient client;

// Khai báo semaphore để đồng bộ hóa
SemaphoreHandle_t xMutex;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Kết nối Wifi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Khởi tạo ThingSpeak
  ThingSpeak.begin(client);

  // Khởi tạo cảm biến DHT11
  dht.begin();
  // Khởi tạo màn hình LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Khởi tạo semaphore
  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL) {
    Serial.println("Failed to create mutex");
    while (1); // Dừng chương trình nếu không thể tạo semaphore
  }

  // Tạo các task FreeRTOS
  xTaskCreate(readSensorTask, "ReadSensorTask", 4096, NULL, 1, NULL);
  xTaskCreate(sendDataToSheetTask, "SendDataToSheetTask", 8192, NULL, 1, NULL);
  xTaskCreate(sendDataToThingSpeakTask, "SendDataToThingSpeakTask", 8192, NULL, 1, NULL);
  xTaskCreate(updateLCDTask, "UpdateLCDTask", 4096, NULL, 1, NULL);
}

void loop() {
}

int getAirQuality() {
  int airQuality = analogRead(ANALOG_PIN);
  return airQuality;
}

void readSensorTask(void* pvParameters) {
  while (1) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      value1 = dht.readTemperature();
      value2 = dht.readHumidity();
      value3 = getAirQuality();

      if (isnan(value1) || isnan(value2)) {
        Serial.println("Failed to read from DHT sensor!");
        Status_Read_Sensor = "Failed";
        value1 = 0;
        value2 = 0;
      } else {
        Status_Read_Sensor = "Success";
      }

      Serial.print("Temperature: ");
      Serial.print(value1);
      Serial.print(" Humidity: ");
      Serial.print(value2);
      Serial.print(" Air Quality: ");
      Serial.println(value3);

      xSemaphoreGive(xMutex);
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS); // Delay 10 giây
  }
}

void sendDataToSheetTask(void* pvParameters) {
  while (1) {
    if (WiFi.status() == WL_CONNECTED) {
      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        String Send_Data_URL = Web_App_URL + "?sts=write";
        Send_Data_URL += "&srs=" + Status_Read_Sensor;
        Send_Data_URL += "&temp=" + String(value1);
        Send_Data_URL += "&humd=" + String(value2);
        Send_Data_URL += "&airq=" + String(value3);

        Serial.println("Sending data to Google Spreadsheet...");
        Serial.print("URL: ");
        Serial.println(Send_Data_URL);

        HTTPClient http;
        http.begin(Send_Data_URL.c_str());
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        int httpCode = http.GET();
        Serial.print("HTTP Status Code: ");
        Serial.println(httpCode);

        if (httpCode > 0) {
          String payload = http.getString();
          Serial.println("Payload: " + payload);
        }

        http.end();
        xSemaphoreGive(xMutex);
      }
    }
    vTaskDelay(30000 / portTICK_PERIOD_MS); // Delay 30 giây
  }
}

void sendDataToThingSpeakTask(void* pvParameters) {
  while (1) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      ThingSpeak.setField(1, value1);
      ThingSpeak.setField(2, value2);
      ThingSpeak.setField(3, value3);
      int response = ThingSpeak.writeFields(channelID, apiKey);
      Serial.print("ThingSpeak response: ");
      Serial.println(response);
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(30000 / portTICK_PERIOD_MS); // Delay 30 giây
  }
}

void updateLCDTask(void* pvParameters) {
  while (1) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      lcd.setCursor(0, 0);
      lcd.print(value1);
      lcd.print("'C");
      lcd.setCursor(7, 0);
      lcd.print(value2);
      lcd.print("%");
      lcd.setCursor(0, 1);
      lcd.print(value3);
      lcd.print(" PPM");
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS); // Delay 10 giây
  }
}
