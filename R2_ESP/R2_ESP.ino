#if !defined(ESP32)
#error This code is intended to run only on the ESP32 boards ! Please check your Tools->Board setting.
#endif

#include <WiFiClientSecure.h>
#include <WebSocketsServer_Generic.h>
#include "ArduinoJson.h"
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define WEBSOCKETS_LOGLEVEL 2
#define LED 2
#define rst_stm 14  //stm reset
#define wifi 4
#define WS_PORT 80
String Z_json = "\{\"B\"\:\"000\"\}";
Adafruit_BNO055 bno = Adafruit_BNO055(55);
long previousmillis = 0;
WebSocketsServer webSocket = WebSocketsServer(WS_PORT);
String controllerdata = "\{\"LOC\"\:\"S000400000\"\}";
char Z_Val[4];
DynamicJsonDocument json(200);

void webSocketEvent(const uint8_t& num, const WStype_t& type, uint8_t* payload, const size_t& length) {
  switch (type) {
    case WStype_DISCONNECTED:
      {
        Serial.printf("[%u] Disconnected!\n", num);
        digitalWrite(LED, LOW);
        delay(10);
        controllerdata = "\{\"LOC\"\:\"S000400000\"\}";
        Serial1.print(controllerdata);
        break;
      }
    case WStype_CONNECTED:
      {
        Serial.print("");
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        digitalWrite(LED, HIGH);
        delay(10);
        json.clear();
        String d = "";
        json["S"] = "1";
        serializeJson(json, d);
        webSocket.broadcastTXT(d);
        json.clear();
        break;
      }
    case WStype_TEXT:
      {
        controllerdata = (char*)payload;
        Serial.println(controllerdata);
        if (controllerdata.charAt(8) == 'U') {
          digitalWrite(rst_stm, HIGH);
          delay(100);
          digitalWrite(rst_stm, LOW);
          delay(100);
        }

        break;
      }

    case WStype_ERROR:
      break;
    default:
      break;
  }
}
void setup() {
  Serial.begin(38400);
  Serial1.begin(38400, SERIAL_8N1, 18, 19);
  Serial2.begin(38400);
  Wire.begin();
  bno.begin();

  bno.setExtCrystalUse(true);
  delay(500);
  long currentt = millis();
  pinMode(LED, OUTPUT);
  pinMode(rst_stm, OUTPUT);
  digitalWrite(LED, LOW);
  digitalWrite(rst_stm, LOW);
  delay(100);
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.begin("TRF_R2_25", "123456789");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(800);
  }

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.print("WebSocket Server started @ IP address: ");
  Serial.println(WiFi.localIP());
  delay(800);
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);
  int bnovalue = event.orientation.x;
  sprintf(Z_Val, "%03d", bnovalue);
  unsigned long tx_millis = millis();
  if (tx_millis - previousmillis > 30) {

    json["B"] = Z_Val;
    serializeJson(json, Z_json);
    Serial.print(Z_json);
    Serial.print("  ");
    Serial.println(controllerdata);
    Serial1.print(controllerdata);
    Serial1.flush();
    Serial2.print(Z_json);
    Serial2.flush();
    webSocket.broadcastTXT(Z_json);
    json.clear();

    previousmillis = tx_millis;
  }

  webSocket.loop();
  delay(10);
}