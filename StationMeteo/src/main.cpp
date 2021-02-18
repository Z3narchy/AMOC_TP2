// https://randomnerdtutorials.com/esp32-web-server-with-bme280-mini-weather-station/
// https://randomnerdtutorials.com/esp32-mqtt-publish-bme280-arduino/
// https://randomnerdtutorials.com/cloud-weather-station-esp32-esp8266/
// https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/

#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DNSServer.h>

#define SEALEVELPRESSURE_HPA (1013.25)

// WIFI
const char *ssid = "StationMeteo";
const char *password = "stationmeteo";

// MQTT
const char *mqttServer = "192.168.43.210";
const int mqttPort = 1883;
const char *mqttUser = "homeassistant";
const char *mqttPassword = "";

// OBJETS
Adafruit_BME280 bme;
WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wifiManager;

void setup()
{
  Serial.begin(115200);

  // Verifie si le BME est bien trouvé
  if (!bme.begin(0x76))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }

  //Connection au réseau
  //wifiManager.resetSettings();
  if (!wifiManager.autoConnect())
  {
    Serial.println("Connection failed");
  }

  // SET MQTT SERVER
  client.setServer(mqttServer, mqttPort);

  while (!client.connected())
  {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP32Client", mqttUser, mqttPassword))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void loop()
{
  // PUBLIE
  String string1 = String(bme.readTemperature());
  String string2 = String(bme.readHumidity());
  String string3 = String(bme.readPressure() / 100.0F);

  client.publish("stationMeteo/Temperature", string1.c_str());
  client.publish("stationMeteo/Humidite", string2.c_str());
  client.publish("stationMeteo/Pression", string3.c_str());
  client.loop();

  // PRINT LN VALEURS
  Serial.println(bme.readTemperature());

  Serial.println(bme.readPressure() / 100.0F);

  Serial.println(bme.readHumidity());

  Serial.println();

  delay(500);
}