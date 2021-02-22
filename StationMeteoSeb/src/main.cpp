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
#include <Arduino.h>

#include "CredentialsCourtierDeMessages.h"

#define nombreDonnees 3

class Bouton
{
private:
    int pin;
    int etat;

public:
    Bouton(int p_pin)
    {
        this->pin = p_pin;
        pinMode(pin, INPUT);
        etat = 1;
    }

    void ChangerEtat()
    {
        int etatLu = LireEtat();
        if (etat != etatLu)
        {
            etat = etatLu;
        }
    }

    int LireEtat()
    {
        int etatLu = digitalRead(pin);
        return etatLu;
    }

    int getEtat()
    {
        return this->etat;
    }
};

class LED
{
private:
    int pin;
    int etat;

public:
    LED(int p_pin)
    {
        this->pin = p_pin;
        pinMode(pin, OUTPUT);
    }

    void Allumer()
    {
        digitalWrite(this->pin, HIGH);
        this->etat = 1;
    }

    void Eteindre()
    {
        digitalWrite(this->pin, LOW);
        this->etat = 0;
    }

    int getEtat()
    {
        return this->etat;
    }
};

class Fenetre
{
private:
    unsigned int numeroFenetre;
    bool estOuverte;

public:
    Fenetre(){};
    Fenetre(unsigned int p_numeroFenetre)
    {
        this->numeroFenetre = p_numeroFenetre;
        this->estOuverte = false;
    }

    void Ouvrir()
    {
        this->estOuverte = true;
    }

    void Fermer()
    {
        this->estOuverte = false;
    }
};

class GestionnaireDeFenetres
{
private:
    //Nombre fenetres fixe utilisé pour démonstration, on devrait normalement prendre un tableau de fenêtres
    //en paramêtre dans le constructeur et l'attribuer à la variable.
    const int nombreFenetres = 5;
    Fenetre fenetres[5] = {Fenetre(1), Fenetre(2), Fenetre(3), Fenetre(4), Fenetre(5)};

    Bouton activationManuelle = Bouton(25);
    Bouton renitialisationConnexion = Bouton(26);
    LED temoinActivation = LED(27);
    LED temoinFenetresOuvertes = LED(17);
    LED temoinFenetresFermees = LED(16);

    const unsigned long tempsClignotement = 2000;
    unsigned long delaisPrecedent = 0;
    int estOuvertureCompletee = 0;
    int estFermetureCompeletee = 0;

public:
    GestionnaireDeFenetres() {}
    void Executer()
    {
        this->IntialiserFenetres();
        activationManuelle.ChangerEtat();

        if (activationManuelle.getEtat())
        {
            Serial.println(activationManuelle.getEtat());
            if (temoinFenetresOuvertes.getEtat() && !estFermetureCompeletee)
            {
                this->FermerFenetres();
            }
            else if (temoinFenetresFermees.getEtat() && !estOuvertureCompletee)
            {
                this->OuvrirFenetres();
            }
        }
    }

    void IntialiserFenetres()
    {
        if (!temoinFenetresFermees.getEtat() && !temoinFenetresOuvertes.getEtat())
        {
            temoinFenetresFermees.Allumer();
            estFermetureCompeletee = 1;
        }
    }

    void OuvrirFenetres()
    {
        if ((millis() - delaisPrecedent) < tempsClignotement)
        {
            temoinActivation.Allumer();
        }
        else
        {
            for (int fenetre = 0; fenetre < nombreFenetres; fenetre++)
            {
                fenetres[fenetre].Ouvrir();
            }

            temoinActivation.Eteindre();
            temoinFenetresFermees.Eteindre();
            temoinFenetresOuvertes.Allumer();
            estOuvertureCompletee = 1;
            estFermetureCompeletee = 0;
        }
        delaisPrecedent = millis();
    };

    void FermerFenetres()
    {
        if ((millis() - delaisPrecedent) < tempsClignotement)
        {
            temoinActivation.Allumer();
        }
        else if ((millis() - delaisPrecedent) > tempsClignotement)
        {
            for (int fenetre = 0; fenetre < nombreFenetres; fenetre++)
            {
                fenetres[fenetre].Fermer();
            }

            temoinActivation.Eteindre();
            temoinFenetresFermees.Allumer();
            temoinFenetresOuvertes.Eteindre();
            estFermetureCompeletee = 1;
            estOuvertureCompletee = 0;
        }
        delaisPrecedent = millis();
    };
};

class ClientCourtierDeMessages
{
private:
    WiFiClient espClient;
    PubSubClient client;

public:
    ClientCourtierDeMessages()
    {
        this->client = PubSubClient(espClient);
    };
    void PublierDonnees(String temperature, String humidite, String pression)
    {
        client.publish("stationMeteo/Temperature", temperature.c_str());
        client.publish("stationMeteo/Humidite", humidite.c_str());
        client.publish("stationMeteo/Pression", pression.c_str());
        client.loop();
    };

    void Configurer()
    {
        client.setServer(mqttServer, mqttPort);

        while (!client.connected())
        {
            Serial.println("Connection à MQTT...");

            if (client.connect("espClient", mqttUser, mqttPassword))
            {
                Serial.println("Connecté");
            }
            else
            {
                Serial.print("La connexion a échoué avec le code: ");
                Serial.print(client.state());
                Serial.println("");
            }

            delay(2000);
        }
    };

    //Fonction pour debuggage
    void AfficherDonneesConsole(String temperature, String humidite, String pression)
    {
        Serial.print("Temperature: ");
        Serial.println(temperature);
        Serial.print("Humidite:    ");
        Serial.println(humidite);
        Serial.print("Pression:    ");
        Serial.println(pression);
        Serial.println();
    }
};

class StationMeteo
{
private:
    Adafruit_BME280 bme280;
    GestionnaireDeFenetres gestionnaireFenetres;
    WiFiManager gestionnaireConnexion;
    ClientCourtierDeMessages clientCourtier;

public:
    StationMeteo(){};
    bool estConfigure = false;
    unsigned long delaisPrecedentStation = 0;
    unsigned long delaisExecution = 3000;
    unsigned long delaisClignotant = 10000;

    void Executer()
    {
        if (!bme280.begin(0x76))
        {
            Serial.println("Echec de lecture! Svp, verifiez les connections du capteur BME280.");
            while (1)
                ;
        }
        gestionnaireFenetres.Executer();

        if (!estConfigure)
        {
            if (!gestionnaireConnexion.autoConnect())
            {
                Serial.println("Échec de la connection");
            }

            clientCourtier.Configurer();
            estConfigure = true;
        }

        if ((millis() - delaisPrecedentStation) > delaisExecution)
        {

            clientCourtier.PublierDonnees(
                String(bme280.readTemperature()),
                String(bme280.readHumidity()),
                String(bme280.readPressure() / 100.0f));
            clientCourtier.AfficherDonneesConsole(
                String(bme280.readTemperature()),
                String(bme280.readHumidity()),
                String(bme280.readPressure() / 100.0f));
            delaisPrecedentStation = millis();
        }
    }
};

StationMeteo station;

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    station.Executer();
    // digitalWrite(13, HIGH);
    // digitalWrite(14, HIGH);
    // digitalWrite(17, HIGH);
}