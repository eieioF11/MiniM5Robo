#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>

bool ota_started          = false;
unsigned int ota_progress = 0;
std::string ota_error     = "";

std::string hostname = "MiniM5Robo";
const IPAddress ip(192, 168, 1, 11);
const IPAddress gateway(192, 168, 1, 1);
const IPAddress subnet(255, 255, 255, 0);
const IPAddress dns1(192, 168, 1, 1);

#if defined(ESP32_RTOS) && defined(ESP32)
void ota_handle(void* parameter) {
  for (;;) {
    ArduinoOTA.handle();
    vTaskDelay(pdMS_TO_TICKS(3500));
  }
}
#endif

void ota_begin() {
  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
        ota_started = true;
      })
      .onEnd([]() {
        Serial.println("\nEnd");
        ota_started = false;
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        ota_progress = (progress / (total / 100));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          ota_error = "Auth Failed";
        else if (error == OTA_BEGIN_ERROR)
          ota_error = "Begin Failed";
        else if (error == OTA_CONNECT_ERROR)
          ota_error = "Connect Failed";
        else if (error == OTA_RECEIVE_ERROR)
          ota_error = "Receive Failed";
        else if (error == OTA_END_ERROR)
          ota_error = "End Failed";
        Serial.println(ota_error.c_str());
        ota_started = false;
      });

  ArduinoOTA.begin();

#if defined(ESP32_RTOS)
  xTaskCreate(ota_handle,   /* Task function. */
              "OTA_HANDLE", /* String with name of task. */
              10000,        /* Stack size in bytes. */
              NULL,         /* Parameter passed as input of the task */
              1,            /* Priority of the task. */
              NULL);        /* Task handle. */
#endif
}

void setupOTA_AP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("f11esp32");
  WiFi.setHostname(hostname.c_str());
  /*use mdns for host name resolution*/
  if (!MDNS.begin(hostname.c_str())) { // http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  if (!WiFi.config(ip, gateway, subnet, dns1)) {
    Serial.println("Failed to configure!");
  }
  Serial.print("local IP address: ");
  Serial.println(WiFi.softAPIP());
  ota_begin();
}

void setupOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.begin();
  WiFi.setHostname(hostname.c_str());
  /*use mdns for host name resolution*/
  if (!MDNS.begin(hostname.c_str())) { // http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");

  ota_begin();
}

void setupOTA(const char* ssid, const char* password) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setHostname(hostname.c_str());
  /*use mdns for host name resolution*/
  if (!MDNS.begin(hostname.c_str())) { // http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");

  ota_begin();
}
