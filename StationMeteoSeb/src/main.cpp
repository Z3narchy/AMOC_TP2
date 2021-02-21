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

    void Clignoter()
    {
        unsigned long delaisCourant = 0;
        int delaisClignotement = 500;

        if ((millis() - delaisCourant) >= delaisClignotement)
        {
            if (this->etat)
            {
                Eteindre();
            }
            else
            {
                Allumer();
            }
            delaisCourant = millis();
        }
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

    Bouton activationManuelle = Bouton(1013);
    Bouton renitialisationConnexion = Bouton(1012);
    LED temoinActivation = LED(1014);
    LED temoinFenetresOuvertes = LED(1027);
    LED temoinFenetresFermees = LED(1016);

    const unsigned long tempsClignotement = 5000;
    unsigned long delaisPrecedent = 0;

public:
    GestionnaireDeFenetres() {}
    void Executer()
    {
        temoinFenetresFermees.Allumer();
        temoinFenetresOuvertes.Allumer();
        temoinActivation.Allumer();

        if (!activationManuelle.getEtat())
        {
            if (temoinFenetresOuvertes.getEtat())
            {
                this->FermerFenetres();
            }
            else if (!temoinFenetresOuvertes.getEtat())
            {
                this->OuvrirFenetres();
            }

            activationManuelle.ChangerEtat();
        }
    }

    // !!! NE PAS OUBLIER DE GERER LE CLIGNOTEMENT !!!
    void OuvrirFenetres()
    {
        Serial.println("Ouverture des fenetres...");
        if ((millis() - delaisPrecedent) < tempsClignotement)
        {
            temoinActivation.Clignoter();
            delaisPrecedent = millis();
        }
        else
        {
            for (int fenetre = 0; fenetre < nombreFenetres; fenetre++)
            {
                fenetres[fenetre].Ouvrir();
            }

            temoinFenetresFermees.Eteindre();
            temoinFenetresOuvertes.Allumer();
        }
    };

    void FermerFenetres()
    {
        Serial.println("Fermeture des fenetres...");
        if ((millis() - delaisPrecedent) < tempsClignotement)
        {
            temoinActivation.Clignoter();
            delaisPrecedent = millis();
        }
        else
        {
            for (int fenetre = 0; fenetre < nombreFenetres; fenetre++)
            {
                fenetres[fenetre].Fermer();
                temoinFenetresFermees.Allumer();
                temoinFenetresOuvertes.Eteindre();
            }
        }
    };
};

class Capteur
{
private:
    Adafruit_BME280 capteurBME280;

    float *LireCapteurs()
    {
        float *donneesLues = new float[3];
        donneesLues[0] = capteurBME280.readTemperature();
        donneesLues[1] = capteurBME280.readHumidity();
        donneesLues[2] = capteurBME280.readPressure() / 100.0F;
        return donneesLues;
    };

    String *ConvertirDonneesLues()
    {
        float *donnees = LireCapteurs();
        String *donneesConverties = new String[nombreDonnees];

        for (int donnee = 0; donnee > nombreDonnees; donnee++)
        {
            donneesConverties[donnee] = String(donnees[donnee]);
        }

        return donneesConverties;
    }

public:
    Capteur(){};
    void ValiderConfiguration()
    {
        if (!capteurBME280.begin(0x76))
        {
            Serial.println("Could not find a valid BME280 sensor, check wiring!");
            while (1)
                ;
        }
    }

    String *EnvoyerDonnees()
    {
        return ConvertirDonneesLues();
    }
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
    void PublierDonnees(String *donnees)
    {
        // client.publish("stationMeteo/Temperature", donnees[0].c_str());
        // client.publish("stationMeteo/Humidite", donnees[1].c_str());
        // client.publish("stationMeteo/Pression", donnees[2].c_str());
        client.publish("stationMeteo/Temperature", String("Le BME280").c_str());
        client.publish("stationMeteo/Humidite", String("est mort.").c_str());
        client.publish("stationMeteo/Pression", String("2020-2021 - R.I.P").c_str());
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
    void AfficherDonneesConsole(String *donnees)
    {
        Serial.print("Temperature: ");
        Serial.println(donnees[0]);
        Serial.print("Humidite:    ");
        Serial.println(donnees[1]);
        Serial.print("Pression:    ");
        Serial.println(donnees[2]);
        Serial.println();
    }
};

class StationMeteo
{
private:
    Capteur bme280;
    GestionnaireDeFenetres gestionnaireFenetres;
    WiFiManager gestionnaireConnexion;
    ClientCourtierDeMessages clientCourtier;

public:
    StationMeteo(){};
    bool estConfigure = false;
    unsigned long delaisPrecedent = 0;
    unsigned long delaisExecution = 3000;
    unsigned long delaisClignotant = 10000;

    void Executer()
    {
        bme280.ValiderConfiguration();
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

        if ((millis() - delaisPrecedent) > delaisExecution)
        {
            String *donneesAPublier = bme280.EnvoyerDonnees();
            clientCourtier.PublierDonnees(donneesAPublier);
            clientCourtier.AfficherDonneesConsole(donneesAPublier);
            delaisPrecedent = millis();
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
}