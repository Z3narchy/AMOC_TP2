#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DNSServer.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <esp_bt_main.h>
#include <SPIFFS.h>
#include "FS.h"

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
        return (this->pression / 100.0f) > 1010.0f;
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
    PanneauDeControle(){};

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

    void InitialiserFenetres(int p_nombreDeFenetres)
    {
        this->nombreDeFenetres = p_nombreDeFenetres;
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

    // char nomFileDeMessageMqtt[60];
    // char *nomfileTemperature;
    // char *nomfileHumidite;
    // char *nomfilePression;

    //Tentative infructueuse d'utiliser un nom de file sur mesure. Manque de temps pour completer une version fonctionnelle.

    // void ConfigurerNomsFilesDeMessages()
    // {
    //     char buffer[100];

    //     strcpy(buffer, nomFileDeMessageMqtt);
    //     strcat(buffer, "/Temperature");
    //     strcpy(this->nomfileTemperature, buffer);
    //     memset(buffer, 0, 50);

    //     strcpy(buffer, nomFileDeMessageMqtt);
    //     strcat(buffer, "/Humidite");
    //     strcpy(this->nomfileHumidite, buffer);
    //     memset(buffer, 0, 50);

    //     strcpy(buffer, nomFileDeMessageMqtt);
    //     strcat(buffer, "/Pression");
    //     strcpy(this->nomfilePression, buffer);

    //     Serial.println(nomfileTemperature);
    //     Serial.println(nomfileHumidite);
    //     Serial.println(nomfilePression);
    // }

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

    void Configurer(char *serveurMqtt, int portMqtt, char *nomUtilisateurMqtt, char *motDePasseMqtt /* char *nomFileMqtt */)
    {
        client.setServer(serveurMqtt, portMqtt);
        //strcpy(this->nomFileDeMessageMqtt, nomFileMqtt);
        //ConfigurerNomsFilesDeMessages();à

        int tentatives = 0;
        while (!client.connected() && tentatives < 5)
        {

            Serial.println("Connection à MQTT...");

            Serial.println(serveurMqtt);
            Serial.println(portMqtt);
            Serial.println(nomUtilisateurMqtt);
            Serial.println(motDePasseMqtt);

            if (client.connect("esp32Client", nomUtilisateurMqtt, motDePasseMqtt))
            {
                Serial.println("Connecté");
            }
            else
            {
                Serial.print("La connexion a échoué avec le code: ");
                Serial.print(client.state());
                Serial.println("");

                Serial.println(serveurMqtt);
                Serial.println(portMqtt);
                Serial.println(nomUtilisateurMqtt);
                Serial.println(motDePasseMqtt);
            }

            tentatives++;
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
    WiFiManagerParameter custom_nom_fileDeMessages;

    char serveurMqtt[40];
    char nomUtilisateurMqtt[25];
    char portServeurMqtt[5];
    char motDePasseMqtt[65];
    char nomFileMqtt[60];
    char nombreFenetres[2];
    static bool estConfigurationMqttSauvegardee;

public:
    GestionnaireDeWifi()
        : custom_mqtt_server("mqttServer", "IP Serveur MQTT", "", 40),
          custom_mqtt_User("mqttUser", "Utilisateur MQTT", "stationMeteo", 25),
          custom_mqtt_Port("mqttPort", "Port Serveur MQTT", "1883", 5),
          custom_mqtt_Password("mqttPassword", "Mot de passe MQTT", "", 65),
          custom_nombre_fenetres("nombreFenetres", "Nombre de fenêtres", 0, 2),
          custom_nom_fileDeMessages("mqttNomFile", "Nom de la file de messages", "", 60)
    {
    }

    void Configurer()
    {
        AjouterParametresConfiguration();

        gestionnaireWifi.setConfigPortalTimeout(90);
        gestionnaireWifi.setWiFiAutoReconnect(1);
        gestionnaireWifi.setSaveConfigCallback(ModifierEstConfigSauvegardee);

        if (!gestionnaireWifi.autoConnect(SSIDPortail, MDPPortail))
        {
            Serial.println("Échec de la connection");
        }
        else
        {
            RecupererConfiguration();
        }
    }

    static void ModifierEstConfigSauvegardee()
    {
        estConfigurationMqttSauvegardee = 1;
    }

    void AjouterParametresConfiguration()
    {
        gestionnaireWifi.addParameter(&custom_mqtt_server);
        gestionnaireWifi.addParameter(&custom_mqtt_Port);
        gestionnaireWifi.addParameter(&custom_mqtt_User);
        gestionnaireWifi.addParameter(&custom_mqtt_Password);
        gestionnaireWifi.addParameter(&custom_nombre_fenetres);
        gestionnaireWifi.addParameter(&custom_nom_fileDeMessages);
    }

    void ActiverPortail()
    {
        if (!gestionnaireWifi.startConfigPortal(SSIDPortail, MDPPortail))
        {
            Serial.println("Échec de la connexion. Reconnexion avec l'ancienne configuration...");
            gestionnaireWifi.autoConnect();
        }
        else
        {
            estConfigurationMqttSauvegardee = 0;
        }
    }

    void SauvegarderConfiguration()
    {
        if (!estConfigurationMqttSauvegardee)
        {
            DynamicJsonDocument bufferJson(6000);
            File cfg = SPIFFS.open("/config.json", "w+");

            bufferJson["serveurMQTT"] = custom_mqtt_server.getValue();
            bufferJson["portMQTT"] = custom_mqtt_Port.getValue();
            bufferJson["utilisateurMQTT"] = custom_mqtt_User.getValue();
            bufferJson["motDePasseMQTT"] = custom_mqtt_Password.getValue();
            bufferJson["nombreDeFenetres"] = custom_nombre_fenetres.getValue();
            bufferJson["nomFileDeMessagesMQTT"] = custom_nom_fileDeMessages.getValue();

            serializeJson(bufferJson, cfg);
            cfg.close();
            bufferJson.clear();
        }
    }

    void RecupererConfiguration()
    {
        DynamicJsonDocument bufferJson(6000);
        File cfg = SPIFFS.open("/config.json", "r+");
        auto erreur = deserializeJson(bufferJson, cfg);

        if (erreur)
        {
            Serial.println("Deserialisation échouée: ");
            Serial.println(erreur.f_str());
        }

        serializeJsonPretty(bufferJson, Serial);
        strcpy(this->serveurMqtt, bufferJson["serveurMQTT"]);
        strcpy(this->portServeurMqtt, bufferJson["portMQTT"]);
        strcpy(this->nomUtilisateurMqtt, bufferJson["utilisateurMQTT"]);
        strcpy(this->motDePasseMqtt, bufferJson["motDePasseMQTT"]);
        strcpy(this->nomFileMqtt, bufferJson["nomFileDeMessagesMQTT"]);
        strcpy(this->nombreFenetres, bufferJson["nombreDeFenetres"]);
    }

    //Je ne veux même pas en parler...
    char *getServeurMqtt()
    {
        return this->serveurMqtt;
    }

    int getPortMqtt()
    {
        return atoi(this->portServeurMqtt);
    }

    char *getNomUtilisateurMqtt()
    {
        return this->nomUtilisateurMqtt;
    }

    char *getMotDePasseMqtt()
    {
        return this->motDePasseMqtt;
    }

    char *getNomFileMqtt()
    {
        return this->nomFileMqtt;
    }

    int getNombreFenetres()
    {
        return atoi(this->nombreFenetres);
    }
};

bool GestionnaireDeWifi::estConfigurationMqttSauvegardee = 0;

class StationMeteo
{
private:
    EvaluateurConditionsMeteo evaluateurMeteo;
    PanneauDeControle panneauControle;
    GestionnaireDeWifi gestionnaireConnexion;
    ClientCourtierDeMessages clientCourtier;

    bool estConfiguree = 0;
    unsigned long delaisPrecedentPublication = 0;
    const unsigned long delaisPublication = 5000;

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
            ActiverPortail();
        }

        evaluateurMeteo.Executer();
        panneauControle.Executer(evaluateurMeteo.getEstMeteoAcceptable());

        if ((millis() - delaisPrecedentPublication) > delaisPublication)
        {
            PublierDonnees();
        }
    }

    void Configurer()
    {
        evaluateurMeteo.ConfigurerCapteur();
        gestionnaireConnexion.Configurer();

        clientCourtier.Configurer(
            gestionnaireConnexion.getServeurMqtt(),
            gestionnaireConnexion.getPortMqtt(),
            gestionnaireConnexion.getNomUtilisateurMqtt(),
            gestionnaireConnexion.getMotDePasseMqtt());
        //gestionnaireConnexion.getNomFileMqtt());

        panneauControle.InitialiserFenetres(gestionnaireConnexion.getNombreFenetres());

        this->estConfiguree = 1;
    }

    void ActiverPortail()
    {
        gestionnaireConnexion.ActiverPortail();
        gestionnaireConnexion.SauvegarderConfiguration();
        gestionnaireConnexion.RecupererConfiguration();
        clientCourtier.Configurer(
            gestionnaireConnexion.getServeurMqtt(),
            gestionnaireConnexion.getPortMqtt(),
            gestionnaireConnexion.getNomUtilisateurMqtt(),
            gestionnaireConnexion.getMotDePasseMqtt());
        //gestionnaireConnexion.getNomFileMqtt());
    }

    void PublierDonnees()
    {
        float temperature = evaluateurMeteo.getTemperature();
        float humidite = evaluateurMeteo.getHumidite();
        float pression = evaluateurMeteo.getPression();

        Serial.println("Publication des données");
        clientCourtier.PublierDonnees(
            String(temperature),
            String(humidite),
            String(pression / 100.0f));

        this->delaisPrecedentPublication = millis();
    }
};

StationMeteo station;

void setup()
{
    Serial.begin(115200);
    esp_bluedroid_disable();
    SPIFFS.begin(true);
}

void loop()
{
    station.Executer();
}