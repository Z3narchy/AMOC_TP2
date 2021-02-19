// https://randomnerdtutorials.com/esp32-web-server-with-bme280-mini-weather-station/
// https://randomnerdtutorials.com/esp32-mqtt-publish-bme280-arduino/
// https://randomnerdtutorials.com/cloud-weather-station-esp32-esp8266/
// https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/


/******* INCLUSION DES BIBLIOTHÈQUES *******/
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DNSServer.h>
#include <Credential.h>


/******* WIFI *******/
const char *ssid = MYSSID;
const char *password = MYPSW;


/******* SUJET_PUBLICATION *******/
#define temperature_topic "stationMeteo/Temperature" 
#define humidite_topic "stationMeteo/Humidite"      
#define pression_topic "stationMeteo/Pression"  


/******* MQTT *******/
const char *mqttServer = "192.168.43.198";
const int mqttPort = 1883;
const char *mqttUser = "homeassistant";
const char *mqttPassword = "ohZ0Ualoo3bae7thie4Nookase2aghaeph5ap4yei2ADae6NeewohthohFee1ohN";


/******* VARIABLES *******/
float temperature;
float humidite;
float pression;
unsigned long lastTime = 0;
unsigned long timerDelay = 3000;
String temperatureConverti;
String humiditeConverti;
String pressionConverti;


/******* OBJETS *******/
Adafruit_BME280 bme;
WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wifiManager;


/******* MÉTHODES *******/
void ObtenirInformationCapteur();
void ConvertiMesureRecueilli();
void PublierInformation();
void AfficherMesureConsole();


void setup()
{
  Serial.begin(115200);

  if (!bme.begin(0x76))
  {
    Serial.println("Echec de lecture! Svp, verifiez les conncetion du capteur BME280.");
    while (1);
  }

  if (!wifiManager.autoConnect())
  {
    Serial.println("Connexion echouee!");
  }

  client.setServer(mqttServer, mqttPort);

  while (!client.connected())
  {
    Serial.println("Connexion au serveur MQTT...");

    if (client.connect("ESP32Client", mqttUser, mqttPassword))
    {
      Serial.println("Connexion reussie!");
    }
    else
    {
      Serial.print("Echec avec erreur ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void loop()
{
  if ((millis() - lastTime) > timerDelay)
  {
    ObtenirInformationCapteur();
    ConvertiMesureRecueilli();
    PublierInformation();
    AfficherMesureConsole();
    client.loop();
    lastTime = millis();
  }
}

void ObtenirInformationCapteur()
{
  temperature = bme.readTemperature();
  humidite = bme.readHumidity();
  pression = bme.readPressure() / 100.0F;
}

void ConvertiMesureRecueilli()
{
  temperatureConverti = String(temperature);
  humiditeConverti = String(humidite);
  pressionConverti = String(pression);
}

void PublierInformation()
{
  client.publish(temperature_topic, temperatureConverti.c_str());
  client.publish(humidite_topic, humiditeConverti.c_str());
  client.publish(pression_topic, pressionConverti.c_str());
}

void AfficherMesureConsole()
{
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Humidite:    ");
  Serial.println(humidite);
  Serial.print("Pression:    ");
  Serial.println(pression);
  Serial.println();
}
/*
RESET = digitalRead(PIN_RESET_BUTTON);
if( RESET == HIGH) {                                 
      Serial.println("Erase settings and restart ...");
      delay(1000);
      wm.resetSettings();  
      ESP.restart();  
}



*/