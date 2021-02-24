// Faire un objet pour gérer fenêtre
// Méthode
// bool




// Version 1

etatFenetre = getEtatFenetre
estActiverManuellement = false;



if(etatFenetre)  // fenêtre ouvert
{
	if(temperature > 25 && humidite < 50)
	{
		if()
		{
			if(pression > 1010)
			{
				fermetureFenetre();
			}
		}
	}
}
else (!etatFenetre) // fenêtre fermé
{
	if(humidite < 50)
	{
		if(temperature < 20)
		{
			if(pression < 1000)
			{
				ouvertureFenetre();
			}
		}
	}
}

if(!estActiverManuellement)
{

	if(temperature > 26 && humidite > 50 && pression > 1000)   // Beau temps et très chaud mais devient humide
{
	fermetureFenetre();
}
if(temperature > 26 && humidite > 50 && pression < 1000)   // Beau temps et très chaud mais pluie
{
	fermetureFenetre();
}
if(temperature > 26 && humidite < 50 && pression > 1000)   // Beau temps et très chaud
{
	fermetureFenetre();
}
if(temperature > 26 && humidite < 50 && pression < 1000)   // Beau temps et très chaud annonce pluie
{
	fermetureFenetre();
}


if(temperature < 18 && humidite > 50 && pression > 1000)   // Beau temps et froid mais devient humide
{
	fermetureFenetre();
}
if(temperature < 18 && humidite > 50 && pression < 1000)   // Beau temps et froid mais pluie
{
	fermetureFenetre();
}
if(temperature < 18 && humidite < 50 && pression > 1000)   // Beau temps et froid
{
	fermetureFenetre();
}
if(temperature < 18 && humidite < 50 && pression < 1000)   // Beau temps mais froid annonce pluie
{
	fermetureFenetre();
}

else 
{
	ouvertureFenetre();
}

tempsInitial
tempsFinal
estAllumeeDernierTour


if(estEnfoncer)
{
	tempsInitial = millis();
	tempsFinal = millis() + 10;
	estActiver = false;
}

if(millis() <= tempsFinal)
{
	if(millis() <= tempsProchainCangement)
	{
		if(estAllumeeDernierTour)
		{
			Fermer
			estAllumeeDernierTour = false;
		}
		else
		{
		    estAllumeeDernierTour = true;
			Allumer
		}
		tempsProchainCangement = millis() + 500;
	}
}

// Temperature haute  > 30
// temperature moyen  entre 20 et 30
// Temperature basse  < 20


// humidite haute  > 60 %
// humidite basse < 60 %


// pression haute  > 1025
// pression moyen  entre 1000 et 1025
// pression basse  < 1000


// https://www.netatmo.com/fr-ca/guides/weather/weather-factors/measure/humidex

// Calculer l’indice humidex à partir de l’humidité relative

// Si l’humidité relative est connue, on calcule l’indice humidex de la manière suivante :

// Humidex = Ta + h
// h = 5/9; (e- 10.0)
// e = 6.112 ; 10 ( 7.5;Ta / (237.7+Ta) ) ; HR/100

// Où Ta = Température de l'air (°C), HR = Humidité relative (%) et e = pression de vapeur.

// Calculer l’indice humidex à partir du point de rosée

// En revanche si le point de rosée est connu, on le calcule de cette façon :

// Humidex = Ta + h
// h = 5/9 ; (e - 10.0)
// e = 6.11 ; exp (5417.7530 ; ( (1/273.16) - (1/Td) ))

// Où Ta = Température de l'air (°C), Td = Température du point de rosée (°K), e = pression de vapeur et exp = fonction exponentielle en base naturelle = 2,71828182845905.

// Une autre façon de présenter l’équation est la suivante :

// où Tair est la température de l'air (degré Celsius), e = 2,71828 et Trosee est le point de rosée (degré Celsius).

etatFenetre = getEtatFenetre


if(humidite > 50 && !etatFenetre)  // Ne cause pas de problème

if(humidite > 50 && etatFenetre)  // fenêtre ouvert
{
	if(temperature > 30)
	{
		fermetureFenetre();
	}
	


}
else if(humidite < 50 && !etatFenetre) // fenêtre fermé
{

	if(temperature > 20 && pression > 1010)
	{
		ouvertureFenetre();
	}



}

if(temperature > 26 && humidite > 50 && pression > 1000)   // Beau temps mais devient humide
{
	fermetureFenetre();
}

else if(temperature < 20 || humidite < 50 || pression > 1000)  // Temps beau mais froid
{
	fermetureFenetre();
}

if(temperature < 20 || humidite > 50 || pression < 1000)
{
	fermetureFenetre();
}
if(temperature < 20 || humidite > 50 || pression < 1000)
{
	fermetureFenetre();
}
























