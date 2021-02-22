#include <Arduino.h>

class Bouton
{
private:
    int etat = 1;
    int pin;

public:
    Bouton(int p_pin)
    {
        this->pin = p_pin;
        pinMode(p_pin, INPUT);
    };
    int LireEtat()
    {
        return digitalRead(this->pin);
    }
    void ChangerEtat()
    {
        int etatCourant = LireEtat();

        if (this->etat != etatCourant)
        {
            this->etat = etatCourant;
        }
    }
    int getEtat()
    {
        return this->etat;
    }
};

class LED
{
private:
    int etat = 0;
    int pin;
    unsigned long delaisAlternance = 500;
    unsigned long delaisCourant = 0;

public:
    LED(int p_pin)
    {
        this->pin = p_pin;
        pinMode(p_pin, OUTPUT);
    };
    void Allumer()
    {
        digitalWrite(pin, HIGH);
        etat = 1;
    }
    void Eteindre()
    {
        digitalWrite(pin, LOW);
        etat = 0;
    }
    void Clignoter()
    {
        if ((millis() - delaisCourant) > delaisAlternance)
        {
            if (etat == 0)
            {
                this->Allumer();
            }
            else if (etat == 1)
            {
                this->Eteindre();
            }
            delaisCourant = millis();
        }
    }
    int getEtat()
    {
        return this->etat;
    }
};

Bouton reset(6);
Bouton activate(5);

LED ouvertes(3);
LED fermes(4);
LED actives(2);

unsigned long delaisPrecedent = 0;
unsigned long dureeClignotement = 5000;
int estFermetureCompleteee = 0;
int estOuvertureComplete = 0;

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    if (!fermes.getEtat() && !ouvertes.getEtat())
    {
        fermes.Allumer();
    }

    activate.ChangerEtat();

    if (!activate.getEtat())
    {
        if (ouvertes.getEtat() && !estFermetureCompleteee)
        {
            estFermetureCompleteee = 0;
            Serial.println(delaisPrecedent);
            if ((millis() - delaisPrecedent) < dureeClignotement)
            {
                actives.Clignoter();
            }
            else
            {
                Serial.println("closing: got there");
                actives.Eteindre();
                ouvertes.Eteindre();
                fermes.Allumer();
                estFermetureCompleteee = 1;
                estOuvertureComplete = 0;
            }
        }

        else if (fermes.getEtat() && !estOuvertureComplete)
        {
            if ((millis() - delaisPrecedent) < dureeClignotement)
            {
                actives.Clignoter();
            }
            else
            {
                Serial.println("opening: got there");
                actives.Eteindre();
                fermes.Eteindre();
                ouvertes.Allumer();
                estFermetureCompleteee = 0;
                estOuvertureComplete = 1;
            }
        }
    }
    delaisPrecedent = millis();
}