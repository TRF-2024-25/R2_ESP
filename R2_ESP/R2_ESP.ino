#if !defined(ESP32)
#error This code is intended to run only on the ESP32 boards! Please check your Tools->Board setting.
#endif

#include <WiFiClientSecure.h>
#include <WebSocketsServer_Generic.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define LED         2
#define rst_stm     14  // STM32 reset
#define wifi        4
#define WS_PORT     80

Adafruit_BNO055 bno = Adafruit_BNO055(55);
WebSocketsServer webSocket = WebSocketsServer(WS_PORT);

char Z_Val[4];
char jsonBuffer[128];
StaticJsonDocument<256> json;
String controllerdata = "{\"LOC\":\"S000400000\"}";  // Default packet
String recievedData = "";
unsigned long previousmillis = 0;

void webSocketEvent(const uint8_t& num, const WStype_t& type, uint8_t* payload, const size_t& length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      digitalWrite(LED, LOW);
      break;

    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      digitalWrite(LED, HIGH);
      break;
    }

    case WStype_TEXT:
      recievedData = (char*)payload;

      // Check U command safely from new data
      if (recievedData.length() > 8 && recievedData.charAt(8) == 'U') {
        digitalWrite(rst_stm, HIGH);
        delay(100);
        digitalWrite(rst_stm, LOW);
        delay(100);
      }

      controllerdata = recievedData;
      break;

    default:
      break;
  }
}

void setup() {
  Serial.begin(38400);
  Serial1.begin(38400, SERIAL_8N1, 18, 19);
  Serial2.begin(38400);
Serial.println("Reset reason: " + String(esp_reset_reason()));
esp_reset_reason_t reason = esp_reset_reason();
switch (reason) {
  case ESP_RST_POWERON: Serial.println("Power-On Reset"); break;
  case ESP_RST_EXT: Serial.println("External Reset via Reset Pin"); break;
  case ESP_RST_SW: Serial.println("Software Reset"); break;
  case ESP_RST_PANIC: Serial.println("Software Crash / Panic"); break;
  case ESP_RST_BROWNOUT: Serial.println("Brownout Reset (Low Power)"); break;
  case ESP_RST_WDT: Serial.println("Watchdog Timer Reset"); break;
  default: Serial.println("Unknown Reset Reason: " + String(reason)); break;
}
  pinMode(LED, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(rst_stm, OUTPUT);
  digitalWrite(27, 1);

  Wire.begin();
  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);

  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.begin("TRF_R2_25", "123456789");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("\nWiFi connected");
  Serial.print("WebSocket Server started @ IP: ");
  Serial.println(WiFi.localIP());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  delay(1000);
  digitalWrite(27, 0);
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);
  int bnovalue = (int)(event.orientation.x);

  snprintf(Z_Val, sizeof(Z_Val), "%03d", bnovalue);

  unsigned long currentMillis = millis();
  if (currentMillis - previousmillis >= 50) {
    Serial1.print(controllerdata);
    
    json["B"] = Z_Val;
    size_t len = serializeJson(json, jsonBuffer, sizeof(jsonBuffer));

    if (len > 0) {
      Serial.print(jsonBuffer);
      Serial.print("  ");
      Serial.println(controllerdata);

      Serial2.write((uint8_t*)jsonBuffer, len);
      webSocket.broadcastTXT((uint8_t*)jsonBuffer, len);
    }

    json.clear();
    previousmillis = currentMillis;
  }

  webSocket.loop();
}
