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
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson

#include "CredentialsCourtierDeMessages.h"
#include "CredentialsWiFiManager.h"

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
        pinMode(p_pin, INPUT_PULLUP);
        this->etat = 1;
    }

    void ChangerEtat()
    {
        int etatLu = LireEtat();
        if (etat != etatLu)
        {
            this->etat = etatLu;
        }
    }

    int LireEtat()
    {
        int etatLu = digitalRead(this->pin);
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

class GestionnaireDeWifi
{
private:
    WiFiManager gestionnaireWifi;
    Bouton demarragePortail = Bouton(26);
    int estPortailDemarre = 0;
    const char *nombreFenetre = "";
    int nombreFenetreConverti = 0;

public:
    GestionnaireDeWifi(){};

    void Configurer()
    {
        gestionnaireWifi.setConfigPortalTimeout(90);
        AjouterParametreConfiguration();

        if (!gestionnaireWifi.autoConnect(SSIDPortail, MDPPortail))
        {
            Serial.println("Échec de la connection");
        }
    }

    void ActiverPortail()
    {
        demarragePortail.ChangerEtat();
        if (!demarragePortail.getEtat())
        {
            gestionnaireWifi.startConfigPortal(SSIDPortail, MDPPortail);
        }
    }

    void AjouterParametreConfiguration()
    {
        gestionnaireWifi.setWiFiAutoReconnect(1);
        WiFiManagerParameter custom_mqtt_server(" server ", " mqtt server ", mqttServer, 40);
        gestionnaireWifi.addParameter(&custom_mqtt_server);
        mqttServer = custom_mqtt_server.getValue();
        gestionnaireWifi.setWiFiAutoReconnect(1);
        WiFiManagerParameter custom_mqtt_User(" server ", " mqtt user ", mqttUser, 25);
        gestionnaireWifi.addParameter(&custom_mqtt_User);
        mqttUser = custom_mqtt_User.getValue();
        gestionnaireWifi.setWiFiAutoReconnect(1);
        WiFiManagerParameter custom_mqtt_Password(" server ", " mqtt port ", mqttPassword, 65);
        gestionnaireWifi.addParameter(&custom_mqtt_Password);
        mqttPassword = custom_mqtt_Password.getValue();
        gestionnaireWifi.setWiFiAutoReconnect(1);
        /*
        WiFiManagerParameter custom_nombre_Fenetre(" maison ", " maison nb fenêtre ", nombreFenetre, 25);
        gestionnaireWifi.addParameter(&custom_nombre_Fenetre);
        nombreFenetre = custom_nombre_Fenetre.getValue();
        nombreFenetreConverti = nombreFenetre.toInt();
        */
    }
};

class PanneauDeControle
{
private:
    //Nombre fenetres fixe utilisé à des fins de démonstration, on devrait normalement prendre un tableau de fenêtres
    //en paramêtre dans le constructeur et l'attribuer à la variable.
    //Cette classe contient trop de varibales et de méthodes, elle devrait être redécoupée, si vous pouvez lire
    //ce commentaire c'est que nous n'avons pas eu le temps de le faire.

    const int nombreFenetres = 5;
    Fenetre fenetres[5] = {Fenetre(1), Fenetre(2), Fenetre(3), Fenetre(4), Fenetre(5)};

    Bouton activationManuelle = Bouton(25);
    //Ajouter objet gestion fenetres auto.
    LED temoinActivation = LED(17);
    LED temoinFenetresOuvertes = LED(27);
    LED temoinFenetresFermees = LED(16);

    const unsigned long tempsActivation = 10000;
    unsigned long tempsDebutOperation;
    int estOperationCompletee;
    int estOperationEnCours;
    int estGestionAutomatique = 1;

public:
    PanneauDeControle() {}

    //*****-----Gerer veille et economie d'energie-----*****

    void Executer()
    {
        this->IntialiserTemoinsFenetres();

        if (estGestionAutomatique)
        {
            //Ajouter methodes utilisé pour verifier données et agir sur les fenetres automatiquement
        }

        if (!estOperationEnCours)
        {
            activationManuelle.ChangerEtat();
        }

        if (!activationManuelle.getEtat() || !estOperationCompletee)
        {
            if (temoinFenetresOuvertes.getEtat())
            {
                this->FermerFenetres();
            }
            else if (temoinFenetresFermees.getEtat())
            {
                this->OuvrirFenetres();
            }
        }
    }

    void IntialiserTemoinsFenetres()
    {
        if (!temoinFenetresFermees.getEtat() && !temoinFenetresOuvertes.getEtat())
        {
            temoinFenetresFermees.Allumer();
            estOperationCompletee = 1;
        }
    }

    void OuvrirFenetres()
    {
        if (!estOperationEnCours)
        {
            this->DemarrerOperations();
        }

        if ((millis() - tempsDebutOperation) > tempsActivation)
        {
            for (int fenetre = 0; fenetre < nombreFenetres; fenetre++)
            {
                fenetres[fenetre].Ouvrir();
            }

            this->CompleterOuverture();
        }
    };

    void FermerFenetres()
    {
        if (!estOperationEnCours)
        {
            this->DemarrerOperations();
        }
        else if ((millis() - tempsDebutOperation) > tempsActivation)
        {
            for (int fenetre = 0; fenetre < nombreFenetres; fenetre++)
            {
                fenetres[fenetre].Fermer();
            }
            this->CompleterFermeture();
        }
    };

    void DemarrerOperations()
    {
        estOperationCompletee = 0;
        tempsDebutOperation = millis();
        estOperationEnCours = 1;
        temoinActivation.Allumer();
    }

    void CompleterOuverture()
    {
        this->GererTemoinsApresOuverture();
        CompleterOperations();
    }

    void CompleterFermeture()
    {
        GererTemoinsApresFermeture();
        CompleterOperations();
    }

    void CompleterOperations()
    {
        estOperationCompletee = 1;
        estOperationEnCours = 0;
    }

    void GererTemoinsApresFermeture()
    {
        temoinActivation.Eteindre();
        temoinFenetresFermees.Allumer();
        temoinFenetresOuvertes.Eteindre();
    }

    void GererTemoinsApresOuverture()
    {
        temoinFenetresOuvertes.Allumer();
        temoinActivation.Eteindre();
        temoinFenetresFermees.Eteindre();
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
    PanneauDeControle panneauControle;
    GestionnaireDeWifi gestionnaireConnexion;
    ClientCourtierDeMessages clientCourtier;

    int estConfigure = 0;
    unsigned long delaisPrecedentStation = 0;
    const unsigned long delaisExecution = 3000;

public:
    StationMeteo(){};

    void Executer()
    {
        if (!estConfigure)
        {
            Configurer();
        }

        //gestionnaireConnexion.ActiverPortail();
        panneauControle.Executer();

        if ((millis() - delaisPrecedentStation) > delaisExecution)
        {

            clientCourtier.PublierDonnees(
                String(bme280.readTemperature()),
                String(bme280.readHumidity()),
                String(bme280.readPressure() / 100.0f));

            // clientCourtier.AfficherDonneesConsole(
            //     String(bme280.readTemperature()),
            //     String(bme280.readHumidity()),
            //     String(bme280.readPressure() / 100.0f));

            delaisPrecedentStation = millis();
        }
    }

    void Configurer()
    {
        if (!bme280.begin(0x76))
        {
            Serial.println("Echec de lecture! Svp, verifiez les connections du capteur BME280.");
            while (1)
                ;
        }
        gestionnaireConnexion.Configurer();
        clientCourtier.Configurer();
        estConfigure = 1;
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
}