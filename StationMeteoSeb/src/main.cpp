#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DNSServer.h>
#include <Arduino.h>
#include <ArduinoJson.h>

#include "CredentialsCourtierDeMessages.h"
#include "CredentialsWiFiManager.h"

class Bouton
{
private:
    int pin;

public:
    Bouton(int p_pin)
    {
        this->pin = p_pin;
        pinMode(p_pin, INPUT_PULLUP);
    }

    bool LireEtat()
    {
        return digitalRead(this->pin);
    }
};

class LED
{
private:
    int pin;
    bool etat;

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
    //Nous sommes conscient que la gestion des fenêtres demanderait beaucoup plus de manipulations que ce qui est contenu dans cette classe,
    //mais afin de simplifier l'exercice qui demandait de simuler le processus nous nous sommes limités au code ci-dessous.
private:
    unsigned int numeroFenetre;
    bool estOuverte = 0;

public:
    Fenetre(){};
    Fenetre(unsigned int p_numeroFenetre)
    {
        this->numeroFenetre = p_numeroFenetre;
        this->estOuverte = 0;
    }

    void Ouvrir()
    {
        this->estOuverte = 1;
    }

    void Fermer()
    {
        this->estOuverte = 0;
    }
};

class TemoinsLumineux
{
private:
    LED temoinActivation = LED(17);
    LED temoinFenetresOuvertes = LED(27);
    LED temoinFenetresFermees = LED(16);
    bool sontInitialises = 0;

public:
    TemoinsLumineux(){};

    void GererTemoinsApresFermeture()
    {
        temoinActivation.Eteindre();
        temoinFenetresOuvertes.Eteindre();
        temoinFenetresFermees.Allumer();
    }

    void GererTemoinsApresOuverture()
    {
        temoinFenetresOuvertes.Allumer();
        temoinActivation.Eteindre();
        temoinFenetresFermees.Eteindre();
    };

    void AllumerTemoinActivation()
    {
        temoinActivation.Allumer();
    }

    void InitialiserTemoins()
    {
        if (!sontInitialises)
        {
            temoinFenetresFermees.Allumer();
            sontInitialises = 1;
        }
    }
};

class ControlesDuPanneau
{
private:
    Bouton activationManuelle = Bouton(25);
    Bouton activationAutomatique = Bouton(26);
    Bouton activationPortail = Bouton(13);
    bool estActivationManuelle;
    bool estActivationAutomatique;
    bool estPortailDemande;

    void LireEtatBoutons()
    {
        //Utilisation des états inverses car boutons en mode INPUT_PULLUP.

        estActivationManuelle = !activationManuelle.LireEtat();
        estPortailDemande = !activationPortail.LireEtat();
        if (!estActivationAutomatique)
        {
            estActivationAutomatique = !activationAutomatique.LireEtat();
        }
    }

public:
    ControlesDuPanneau()
    {
        estActivationManuelle = 0;
        estActivationAutomatique = 1;
        estPortailDemande = 0;
    };

    bool getEstActivationAutomatique()
    {
        return estActivationAutomatique;
    }

    bool getEstPortailDemande()
    {
        return estPortailDemande;
    }

    bool getEstActivationManuelle()
    {
        return estActivationManuelle;
    }

    void SelectionnerTypeOperation()
    {
        LireEtatBoutons();
        if (estActivationManuelle)
        {
            this->estActivationAutomatique = 0;
        }
    }
};

class EvaluateurConditionsMeteo
{
private:
    Adafruit_BME280 bme280;
    bool estMeteoAcceptable;
    float temperature;
    float humidite;
    float pression;

    void EvaluerConditionsMeteo()
    {
        this->estMeteoAcceptable = EvaluerTemperature() &&
                                   EvaluerHumidite() &&
                                   EvaluerPression();
    }

    bool EvaluerTemperature()
    {
        return this->temperature > 18 && this->temperature < 26;
    }

    bool EvaluerHumidite()
    {
        return this->humidite < 50;
    }

    bool EvaluerPression()
    {
        return this->pression > 1010;
    }

    void LireBme280()
    {
        this->temperature = bme280.readTemperature();
        this->humidite = bme280.readHumidity();
        this->pression = bme280.readPressure();
    }

public:
    EvaluateurConditionsMeteo(){};

    void Executer()
    {
        LireBme280();
        EvaluerConditionsMeteo();
    }

    float getTemperature()
    {
        return temperature;
    }

    float getHumidite()
    {
        return humidite;
    }

    float getPression()
    {
        return pression;
    }

    bool getEstMeteoAcceptable()
    {
        return estMeteoAcceptable;
    }

    void ConfigurerCapteur()
    {
        if (!bme280.begin(0x76))
        {
            Serial.println("Echec de lecture! Svp, verifiez les connections du capteur BME280.");
            while (1)
                ;
        }
    }
};

class EtatsPanneauDeControle
{
private:
    bool sontFenetresOuvertes;
    bool estOperationCompletee;
    bool estOperationEnCours;

public:
    EtatsPanneauDeControle()
    {
        this->sontFenetresOuvertes = 0;
        this->estOperationCompletee = 1;
        this->estOperationEnCours = 0;
    };

    void InverserEtatFenetres()
    {
        this->sontFenetresOuvertes = !sontFenetresOuvertes;
    }

    void CommencerOperation()
    {
        this->estOperationCompletee = 0;
        this->estOperationEnCours = 1;
    }

    void TerminerOperation()
    {
        InverserEtatFenetres();
        this->estOperationCompletee = 1;
        this->estOperationEnCours = 0;
    }

    bool getSontFenetresOuvertes()
    {
        return sontFenetresOuvertes;
    }

    bool getEstOperationsEnCours()
    {
        return estOperationEnCours;
    }

    bool getEstOperationCompletee()
    {
        return estOperationCompletee;
    }
};

class PanneauDeControle
{
    //Nous savons que cette classe n'est pas optimale et si vous pouvez lire ce commentaire
    //c'est que nous avons manqué de temps pour la paufiner.
private:
    Fenetre *fenetres;
    TemoinsLumineux temoins;
    EtatsPanneauDeControle etats;
    ControlesDuPanneau controles;
    const unsigned long tempsActivationFenetres = 10000;
    unsigned long tempsDebutOperation;
    int nombreDeFenetres;

public:
    PanneauDeControle()
    {
        this->nombreDeFenetres = atoi(nombreFenetres);
    };

    void Executer(bool estMeteoAcceptable)
    {
        temoins.InitialiserTemoins();

        if (!etats.getEstOperationsEnCours())
        {
            controles.SelectionnerTypeOperation();
        }

        if (controles.getEstActivationAutomatique())
        {
            ExecuterOperationsAutomatiques(estMeteoAcceptable);
        }
        else if (controles.getEstActivationManuelle())
        {
            ExecuterOperationsManuelles();
        }
    }

    void ExecuterOperationsAutomatiques(bool estMeteoAcceptable)
    {
        bool sontFenetresOuvertes = etats.getSontFenetresOuvertes();

        if (estMeteoAcceptable && !sontFenetresOuvertes)
        {
            OuvrirFenetres();
        }
        if (!estMeteoAcceptable && sontFenetresOuvertes)
        {
            FermerFenetres();
        }
    }

    void ExecuterOperationsManuelles()
    {
        bool sontFenetresOuvertes = etats.getSontFenetresOuvertes();

        if (sontFenetresOuvertes)
        {
            FermerFenetres();
        }
        else if (!sontFenetresOuvertes)
        {
            OuvrirFenetres();
        }
    }

    void OuvrirFenetres()
    {
        if (!etats.getEstOperationsEnCours())
        {
            DemarrerOperations();
        }
        else if ((millis() - this->tempsDebutOperation) > this->tempsActivationFenetres)
        {
            for (int indice = 0; indice < nombreDeFenetres; indice++)
            {
                this->fenetres[indice].Ouvrir();
            }
            CompleterOuverture();
        }
    };

    void FermerFenetres()
    {
        if (!etats.getEstOperationsEnCours())
        {
            DemarrerOperations();
        }
        else if ((millis() - tempsDebutOperation) > tempsActivationFenetres)
        {
            for (int indice = 0; indice < nombreDeFenetres; indice++)
            {
                fenetres[indice].Fermer();
            }

            CompleterFermeture();
        }
    };

    bool getEstPortailDemande()
    {
        return controles.getEstPortailDemande();
    }

    void DemarrerOperations()
    {
        this->tempsDebutOperation = millis();
        this->etats.CommencerOperation();
        this->temoins.AllumerTemoinActivation();
    }

    void CompleterOuverture()
    {
        this->temoins.GererTemoinsApresOuverture();
        this->etats.TerminerOperation();
    }

    void CompleterFermeture()
    {
        this->temoins.GererTemoinsApresFermeture();
        this->etats.TerminerOperation();
    }

    void InitialiserFenetres()
    {
        this->fenetres = new Fenetre[nombreDeFenetres];
        for (int indiceFenetre = 0; indiceFenetre < nombreDeFenetres; indiceFenetre++)
        {
            this->fenetres[indiceFenetre] = Fenetre(indiceFenetre);
        };
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

    void PublierDonnees(String temperature, String humidite, String pression)
    {
        client.publish("stationMeteo/Temperature", temperature.c_str());
        client.publish("stationMeteo/Humidite", humidite.c_str());
        client.publish("stationMeteo/Pression", pression.c_str());
        client.loop();
    };

    void Configurer()
    {
        client.setServer(mqttServer, atoi(mqttPort));

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
};

class GestionnaireDeWifi
{
private:
    WiFiManager gestionnaireWifi;
    WiFiManagerParameter custom_mqtt_server;
    WiFiManagerParameter custom_mqtt_User;
    WiFiManagerParameter custom_mqtt_Port;
    WiFiManagerParameter custom_mqtt_Password;
    WiFiManagerParameter custom_nombre_fenetres;
    DynamicJsonDocument parametres;
    bool estSerialise = 0;

public:
    GestionnaireDeWifi()
        : custom_mqtt_server("mqttServer", "Serveur MQTT", mqttServer, 40),
          custom_mqtt_User("mqttUser", "Utilisateur MQTT", mqttUser, 25),
          custom_mqtt_Port("mqttPort", "Port Serveur MQTT", mqttPort, 5),
          custom_mqtt_Password("mqttPassword", "Mot de passe MQTT", "", 65),
          custom_nombre_fenetres("nombreFenetres", "Nombre de fenêtres", 0, 2),
          parametres(200)
    {
    }

    void Configurer()
    {
        AjouterParametresConfiguration();
        gestionnaireWifi.setConfigPortalTimeout(90);
        gestionnaireWifi.setWiFiAutoReconnect(1);

        if (!gestionnaireWifi.autoConnect(SSIDPortail, MDPPortail))
        {
            Serial.println("Échec de la connection");
            estSerialise = 0;
        }

        SauvegarderParametres();
    }

    void AjouterParametresConfiguration()
    {
        gestionnaireWifi.addParameter(&custom_mqtt_server);
        gestionnaireWifi.addParameter(&custom_mqtt_User);
        gestionnaireWifi.addParameter(&custom_mqtt_Port);
        gestionnaireWifi.addParameter(&custom_mqtt_Password);
        gestionnaireWifi.addParameter(&custom_nombre_fenetres);
    }

    bool ActiverPortail()
    {
        if (!gestionnaireWifi.startConfigPortal(SSIDPortail, MDPPortail))
        {
            gestionnaireWifi.autoConnect();
        }
    }

    void SauvegarderParametres()
    {
        if (!estSerialise)
        {
            String mqttServer = custom_mqtt_server.getValue();
            String mqttUser = custom_mqtt_User.getValue();
            String mqttPort = custom_mqtt_Port.getValue();
            String mqttPassword = custom_mqtt_Password.getValue();
            String nombreFenetres = custom_nombre_fenetres.getValue();

            parametres["serveurMQTT"] = mqttServer;
            parametres["utilisateurMQTT"] = mqttUser;
            parametres["portMQTT"] = mqttPort;
            parametres["motDePasseMQTT"] = mqttPassword;
            parametres["nombreDeFenetres"] = nombreFenetres;

            serializeJsonPretty(parametres, Serial);
            Serial.println("");
        }
        estSerialise = 1;
    }

    void RecupererParametres()
    {
        char json[] = "{\"serveur\":\"serveurMQTT\",\"utilisateur\":\"utilisateurMQTT\", \"port\":\"portMQTT\", \"motDePasse\":\"motDePasseMQTT\", \"nombreFenetres\":\"nombreDeFenetres\"}";

        DynamicJsonDocument parametres(200);
        DeserializationError erreur = deserializeJson(parametres, json);

        if (erreur)
        {
            Serial.println("Deserialization failed: ");
            Serial.println(erreur.f_str());
        }

        const char *serveur = parametres["serveur"];
        const char *utilisateur = parametres["utilisateur"];
        const char *port = parametres["port"];
        const char *motDePasse = parametres["motDePasse"];
        const char *nombreFenetres = parametres["nombreFenetres"];

        //Reste a pousser dans config esp.
    }
};

class StationMeteo
{
private:
    EvaluateurConditionsMeteo evaluateurMeteo;
    PanneauDeControle panneauControle;
    GestionnaireDeWifi gestionnaireConnexion;
    ClientCourtierDeMessages clientCourtier;

    bool estConfiguree = 0;
    unsigned long delaisPrecedentStation = 0;
    const unsigned long delaisExecution = 3000;

public:
    StationMeteo(){};

    void Executer()
    {
        if (!estConfiguree)
        {
            Configurer();
        }

        if (panneauControle.getEstPortailDemande())
        {
            gestionnaireConnexion.ActiverPortail();
            clientCourtier.Configurer();
        }

        evaluateurMeteo.Executer();

        panneauControle.Executer(evaluateurMeteo.getEstMeteoAcceptable());

        if ((millis() - delaisPrecedentStation) > delaisExecution)
        {
            //Devaient initialement être reçues via un Array mais cela faisait paniquer
            //le processeur donc nous avons opté pour 3 accesseurs.

            float temperature = evaluateurMeteo.getTemperature();
            float humidite = evaluateurMeteo.getHumidite();
            float pression = evaluateurMeteo.getPression();

            clientCourtier.PublierDonnees(
                String(temperature),
                String(humidite),
                String(pression / 100.0f));

            this->delaisPrecedentStation = millis();
        }
    }

    void Configurer()
    {
        panneauControle.InitialiserFenetres();
        evaluateurMeteo.ConfigurerCapteur();
        gestionnaireConnexion.Configurer();
        clientCourtier.Configurer();
        this->estConfiguree = 1;
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