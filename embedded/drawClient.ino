#include <WiFiManager.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>

#define PIN 16

const char* MQTT_SERVER = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;

Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(8, 8, PIN,
  NEO_MATRIX_TOP     + NEO_MATRIX_RIGHT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE,
  NEO_GRB            + NEO_KHZ800);

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

const char DRAW_TOPIC[] = "nitek/draw";
const char SYNC_TOPIC[] = "nitek/draw/sync";
const char CONNECT_TOPIC[] = "nitek/draw/connect";

uint16_t data[8][8] = {0};

volatile bool updateMatrix = false;
volatile bool syncMatrix = false;

void startWifi() {
  Serial.println("Connecting Wifi");

  WiFiManager wifiManager;
  wifiManager.setDebugOutput(false);
  wifiManager.setEnableConfigPortal(false);
  wifiManager.setTimeout(0);
  uint8_t i = 0;
  while(!wifiManager.autoConnect("draw", "drawdraw") && i++ < 3) {
    Serial.println("Retry autoConnect");
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
  }
  if(!WiFi.isConnected()) {
    wifiManager.setEnableConfigPortal(true);
    wifiManager.autoConnect("draw", "drawdraw");
  }
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
		    xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  mqttClient.subscribe(CONNECT_TOPIC, 1);
  mqttClient.subscribe(DRAW_TOPIC, 1);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t length, size_t index, size_t total) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);

  if (strcmp(topic, DRAW_TOPIC) == 0) {
    if(length == 3) {
      uint8_t x = payload[0] >> 4;
      uint8_t y = payload[0] & 0x0F;

      if(x >= matrix.width() || y >= matrix.height()) {
        Serial.printf("Invalid coordinates %d,%d\n", x, y);
        return;
      }

      uint16_t color = payload[2] * 256 + payload[1];

      if(data[matrix.width()-x][matrix.height()-y] != color) {
        data[matrix.width()-x][matrix.height()-y] = color;

        matrix.drawPixel(x, y, payload[1] * 256 + payload[2]);
        updateMatrix = true;
      }
    } else if (length == sizeof(data)) {
      strncpy((char*)data, payload, sizeof(data));
      syncMatrix = true;
    } else {
      Serial.print("Wrong msg size: ");
      Serial.println(length);
      return;
    }
  } else if (strcmp(topic, CONNECT_TOPIC) == 0) {
    sync();
  } else {
    Serial.print("Wrong topic?! ");
    Serial.println(topic);
  }
}

void setup() {
  Serial.begin(115200);

  matrix.begin();
  matrix.setBrightness(20);
  matrix.setRotation(1);
  matrix.fill(0);
  matrix.show();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(startWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

  startWifi();

}

void sync() {
  Serial.print("Sync");
  mqttClient.publish(SYNC_TOPIC, 1, false, (char*)data, sizeof(data));
  Serial.println();
}

void loop() {
  if(syncMatrix) {
    for(uint8_t x = 0; x < matrix.width(); x++) {
      for(uint8_t y = 0; y < matrix.height(); y++) {
        matrix.drawPixel(x, y, data[matrix.width()-x][matrix.height()-y]);
      }
    }
    matrix.show();
    syncMatrix = false;
  }
  if(updateMatrix) {
    matrix.show();
    updateMatrix = false;
  }
}