/*String lineBuffer = "";

void setup() {
  Serial.begin(115200);    //PC Monitor Connection
  Serial2.begin(115200, SERIAL_8N1, 16, 17);  //STM32 Connection
  Serial.println("ESP32 JSON test started...");
}

void loop() {
  while(Serial2.available()){
    char c = Serial2.read();

    if(c == '\n'){
      Serial.print("JSON received: ");
      Serial.println(lineBuffer);
      lineBuffer = "";
    }else if(c != 'r'){
      lineBuffer += c;
    }
   }
}
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi bilgileri
const char* ssid = "Vodafone-Rmz";
const char* password = "VANantep27**65";

// ThingSpeak MQTT Broker
const char* mqttServer = "mqtt3.thingspeak.com";
const int mqttPort = 1883;

// MQTT Credentials (My Profile -> MQTT)
const char* mqttUser = "LA4IARklJzcAOTsGFzoYIhI";
const char* mqttPassword = "mIjxxkH8YMY1p+MPDFajTJoP";
const char* mqttClientID = "LA4IARklJzcAOTsGFzoYIhI";

// Kanal bilgileri
const char* publishTopic = "channels/3100736/publish";

WiFiClient espClient;
PubSubClient client(espClient);

String lineBuffer = "";
unsigned long lastSend = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // UART2: RX=GPIO16, TX=GPIO17

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("WiFi baglantisi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" baglandi!");

  // MQTT
  client.setServer(mqttServer, mqttPort);
  while (!client.connected()) {
    Serial.print("MQTT baglantisi...");
    if (client.connect(mqttClientID, mqttUser, mqttPassword)) {
      Serial.println(" baglandi!");
    } else {
      Serial.print(" hata, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void loop() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      // JSON string geldi
      Serial.println("JSON alindi: " + lineBuffer);

      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, lineBuffer);

      if (!error) {
        float temperature = doc["temperature"];
        float acc_x = doc["acceleration_x"];

        Serial.printf("Temp=%.2f  AccX=%.3f\n", temperature, acc_x);

        // ThingSpeak'e göndermek (her 16 saniyede 1 kere)
        if (lastSend == 0 || millis() - lastSend > 16000) {
          String payload = "api_key=IXRFWJZA7I2LO5VE";
          payload += "&field1=" + String(temperature, 2);
          payload += "&field2=" + String(acc_x, 3);

          boolean result = client.publish(publishTopic, payload.c_str());
          if (result) {
            Serial.println("MQTT gonderildi: " + payload);
          } else {
            Serial.println("MQTT gonderme HATA!");
          }
          lastSend = millis();
        } else {
          Serial.println("-> 15sn limit! Veri bekletiliyor...");
        }
      } else {
        Serial.println("JSON parse hatasi!");
      }

      lineBuffer = "";
    } else if (c != '\r') {
      lineBuffer += c;
    }
  }

  client.loop();
}

