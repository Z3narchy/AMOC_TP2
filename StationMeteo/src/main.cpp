// https://randomnerdtutorials.com/esp32-web-server-with-bme280-mini-weather-station/
// https://randomnerdtutorials.com/esp32-mqtt-publish-bme280-arduino/
// https://randomnerdtutorials.com/cloud-weather-station-esp32-esp8266/
// https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

// WIFI
const char *ssid = "Mettre ici le nom de ton résea ex:Mimi";
const char *password = "Mette ici ton mot de passe de ton réseau Wifi ex:123456";

// MQTT
const char *mqttServer = "Mettre ici l'adresse ip de ta VM ex: 192.168.3.10";
const int mqttPort = 1883;
const char *mqttUser = "homeassistant";
const char *mqttPassword = "Mettre ici la clé de home Assistance ex:oaBimohtaik0ceigeec0liewah7Bo4iegoos8egheHeo";

// OBJETS
Adafruit_BME280 bme;
WiFiClient espClient;
PubSubClient client(espClient);

void setup()
{

  // Verifie si le BME est bien trouvé
  if (!bme.begin(0x76)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

  Serial.begin(115200);

  // DÉMARRE LA CONNEXION WIFI
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");


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

  client.publish("environnementQuebec/Temperature", string1.c_str());
  client.publish("environnementQuebec/Humidite", string2.c_str());
  client.publish("environnementQuebec/Pression", string3.c_str());
  client.loop();

  // PRINT LN VALEURS
    Serial.println(bme.readTemperature());

    Serial.println(bme.readPressure() / 100.0F);

    Serial.println(bme.readHumidity());

    Serial.println();

  delay(500);
}