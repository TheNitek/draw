#include <WiFiManager.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <inttypes.h>
#include <AsyncMqttClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#include "images.h"

#define PIN 16

#define MATRIX_WIDTH 8
#define MATRIX_HEIGHT 8

const char* MQTT_SERVER = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;

Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(MATRIX_WIDTH, MATRIX_HEIGHT, PIN,
  NEO_MATRIX_TOP     + NEO_MATRIX_RIGHT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE,
  NEO_GRB            + NEO_KHZ800);

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

const char DRAW_TOPIC_TPL[] = "draw/%" PRIx64;
const char SYNC_TOPIC_TPL[] = "draw/%" PRIx64 "/sync";
const char CONNECT_TOPIC_TPL[] = "draw/%" PRIx64 "/connect";

char DRAW_TOPIC[50] = "";
char SYNC_TOPIC[50] = "";
char CONNECT_TOPIC[50] = "";

uint16_t pixelData[MATRIX_WIDTH][MATRIX_HEIGHT] = {{0}};

volatile bool updateMatrix = false;
volatile bool syncMatrixNeeded = false;
volatile bool onFire = false;

uint8_t currentImage = 0;

const uint16_t touchThreshold = 30;

class TouchData {
  public:
    volatile unsigned long touched = 0;
    bool touchHandled = false;
    bool isTouched() {
      if((touched != 0) && (millis() - touched > 100)) {
        touchHandled = false;
        touched = 0;
      }
      if(touched && !touchHandled) {
        touchHandled = true;
        return true;
      }
      return false;
    }
};

TouchData touch0;
TouchData touch3;
TouchData touch4;
TouchData touch5;

void IRAM_ATTR handleT0Touch() { touch0.touched = millis();}
void IRAM_ATTR handleT3Touch() { touch3.touched = millis();}
void IRAM_ATTR handleT4Touch() { touch4.touched = millis();}
void IRAM_ATTR handleT5Touch() { touch5.touched = millis();}

class Fire {
  private:
    uint16_t data[MATRIX_WIDTH][MATRIX_HEIGHT] = {{0}};

    //these values are subtracted from the generated values to give a shape to the animation
    const uint8_t valueMask[MATRIX_WIDTH][MATRIX_HEIGHT] = {
        {255, 192, 160, 128, 128, 160, 192, 255},
        {255, 160, 128, 96 , 96 , 128, 160, 255},
        {192, 128, 96 , 64 , 64 , 96 , 128, 192},
        {160, 96 , 64 , 32 , 32 , 64 , 96 , 160},
        {128, 64 , 32 , 0  , 0  , 32 , 64 , 128},
        {96 , 32 , 0  , 0  , 0  , 0  , 32 , 96 },
        {64 , 0  , 0  , 0  , 0  , 0  , 0  , 64 },
        {32 , 0  , 0  , 0  , 0  , 0  , 0  , 32 }
    };

    //these are the hues for the fire, should be between 0 (red) to about 25 (yellow)
    const uint8_t hueMask[MATRIX_WIDTH][MATRIX_HEIGHT] = {
        {0 , 0 , 0 , 1 , 1 , 0 , 0 , 0},
        {0 , 0 , 1 , 5 , 5 , 1 , 0 , 0},
        {0 , 1 , 5 , 8 , 8 , 5 , 1 , 0},
        {1 , 5 , 11, 11, 11, 11, 5 , 1},
        {1 , 5 , 11, 13, 13, 13, 5 , 1},
        {1 , 8 , 17, 16, 19, 16, 8 , 1},
        {5 , 11, 19, 21, 25, 21, 11, 5},
        {8 , 18, 23, 25, 25, 24, 18, 8}
    };

    uint16_t line[MATRIX_WIDTH];
    uint8_t pcnt = 0;

    /**
     * Randomly generate the next line
     */
    void generateLine(){
      for(uint8_t x=0; x<MATRIX_WIDTH; x++) {
        line[x] = random(64, 255);
      }
    };

    /**
     * shift all values in the matrix up one row
     */
    void shiftUp() {
      for (uint8_t y=0; y<MATRIX_HEIGHT-1; y++) {
        memcpy(data[y], data[y+1], sizeof(uint16_t)*MATRIX_WIDTH);
      }

      memcpy(data[MATRIX_HEIGHT-1], line, sizeof(uint16_t)*MATRIX_WIDTH);
    };

    void drawFrame() {
      int nextv;
      
      //each row interpolates with the one before it
      for (uint8_t y=0; y<MATRIX_HEIGHT-1; y++) {
        for (uint8_t x=0; x<MATRIX_WIDTH; x++) {
          nextv = 
              (((100.0-pcnt)*data[y][x] 
            + pcnt*data[y+1][x])/100.0) 
            - valueMask[y][x];
          uint32_t color = Adafruit_NeoMatrix::ColorHSV(
            hueMask[y][x] << 8, // H
            255, // S
            (uint8_t)max(0, nextv) // V
          );
          uint16_t rgb = Adafruit_NeoMatrix::Color((color>>16), (color>>8), color);
          matrix.drawPixel(x, y, rgb);
        }
      };
      
      //first row interpolates with the "next" line
      for(uint8_t x=0; x<MATRIX_WIDTH; x++) {
        uint32_t color = Adafruit_NeoMatrix::ColorHSV(
          hueMask[MATRIX_HEIGHT-1][x] << 8, // H
          255,           // S
          (uint8_t)(((100.0-pcnt)*data[MATRIX_HEIGHT-1][x] + pcnt*line[x])/100.0) // V
        );
        uint16_t rgb = Adafruit_NeoMatrix::Color((color>>16), (color>>8), color);
        matrix.drawPixel(x, MATRIX_HEIGHT-1, rgb);
      }
    };
  public:
    Fire() {
      generateLine();
    };
    void burn() {
      if (pcnt >= 100) {
        shiftUp();
        generateLine();
        pcnt = 0;
      }
      drawFrame();
      matrix.show();
      pcnt+=1;
    };
    void reset() {
      memset(data, 0, sizeof(data));
      matrix.clear();
    }
};

Fire fire;

void startWifi() {
  Serial.println("Connecting Wifi");

  WiFiManager wifiManager;
  wifiManager.setDebugOutput(false);
  wifiManager.setEnableConfigPortal(false);
  wifiManager.setTimeout(5*60);
  wifiManager.setCleanConnect(true);
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
  if(!WiFi.isConnected()) {
    ESP.restart();
  }
}

void syncClients() {
  Serial.print("Sync Clients");
  mqttClient.publish(SYNC_TOPIC, 1, false, (char*)pixelData, sizeof(pixelData));
  Serial.println();
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
    default:
      // Ignore
      ;
    }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  mqttClient.subscribe(CONNECT_TOPIC, 1);
  mqttClient.subscribe(DRAW_TOPIC, 1);
  syncClients();
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void syncMatrix() {
  for(uint8_t x = 0; x < matrix.width(); x++) {
    for(uint8_t y = 0; y < matrix.height(); y++) {
      matrix.drawPixel(x, y, (pixelData[y][x] << 8) | (pixelData[y][x] >> 8));
    }
  }
  matrix.show();
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t length, size_t index, size_t total) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);

  if (strcmp(topic, DRAW_TOPIC) == 0) {
    if(length == 1) {
      if(payload[0] < IMAGE_COUNT) {
        Serial.printf("Picture %d\n", payload[0]);
        memcpy_P(pixelData, IMAGES[(uint8_t)payload[0]], sizeof(pixelData));
        syncMatrixNeeded = true;
        syncClients();
      }
    } else if(length == 3) {
      uint8_t x = payload[0] >> 4;
      uint8_t y = payload[0] & 0x0F;

      if(x >= matrix.width() || y >= matrix.height()) {
        Serial.printf("Invalid coordinates %d,%d\n", x, y);
        return;
      }

      uint16_t color = payload[2] * 256 + payload[1];

      if(pixelData[y][x] != color) {
        pixelData[y][x] = color;

        matrix.drawPixel(x, y, payload[1] * 256 + payload[2]);
        updateMatrix = true;
      }
    } else if (length == sizeof(pixelData)) {
      memcpy(pixelData, payload, sizeof(pixelData));
      syncMatrixNeeded = true;
    } else {
      Serial.print("Wrong msg size: ");
      Serial.println(length);
      return;
    }
  } else if (strcmp(topic, CONNECT_TOPIC) == 0) {
    onFire = false;
    syncMatrixNeeded = true;
    syncClients();
  } else {
    Serial.print("Wrong topic?! ");
    Serial.println(topic);
  }
}

void showImage(uint8_t imageNo) {
  Serial.printf("Image: %d\n", imageNo);
  currentImage = imageNo;
  onFire = false;
  memcpy_P(pixelData, IMAGES[imageNo], sizeof(pixelData));
  syncMatrix();
  matrix.show();
  syncClients();
}

void setup() {
  Serial.begin(115200);

  touch_pad_init();
  touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
  touch_pad_filter_start(500);
  delay(500);
  touchAttachInterrupt(T0, handleT0Touch, touchThreshold);
  touchAttachInterrupt(T3, handleT3Touch, touchThreshold);
  touchAttachInterrupt(T4, handleT4Touch, touchThreshold);
  touchAttachInterrupt(T5, handleT5Touch, touchThreshold);

  randomSeed(analogRead(0));

  matrix.begin();
  matrix.setBrightness(10);
  matrix.setRotation(1);
  // Show Random Image on Startup
  currentImage = random(IMAGE_COUNT);
  showImage(currentImage);

  //ESP.getEfuseMac();
  sprintf(DRAW_TOPIC, DRAW_TOPIC_TPL, ESP.getEfuseMac());
  sprintf(SYNC_TOPIC, SYNC_TOPIC_TPL, ESP.getEfuseMac());
  sprintf(CONNECT_TOPIC, CONNECT_TOPIC_TPL, ESP.getEfuseMac());

  Serial.println("Topics:");
  Serial.println(DRAW_TOPIC);
  Serial.println(SYNC_TOPIC);
  Serial.println(CONNECT_TOPIC);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(startWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

  startWifi();
}

void loop() {
  if(syncMatrixNeeded || (updateMatrix && onFire)) {
    syncMatrix();
    syncMatrixNeeded = false;
    onFire = false;
  }
  if(updateMatrix) {
    matrix.show();
    updateMatrix = false;
  }
  if(onFire) {
    fire.burn();
  }

  if(touch0.isTouched()) {
    showImage((currentImage ? currentImage-1 : IMAGE_COUNT-1));
  }
  if(touch3.isTouched()) {
    showImage((currentImage+1) % IMAGE_COUNT);
  }
  if(touch4.isTouched()) {
    Serial.println("On fire!");
    fire.reset();
    onFire = true;
  }
  if(touch5.isTouched()) {
    Serial.println("Clear screen");
    onFire = false;
    memset(pixelData, 0, sizeof(pixelData));
    matrix.clear();
    matrix.show();
    syncClients();
  }
}