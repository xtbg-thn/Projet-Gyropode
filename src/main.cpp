#include <Arduino.h>
#include <String.h>

char FlagCalcul = 0;
float Ve, Vs = 0;
float Te = 10;    // période d'échantillonage en ms
float Tau = 1000; // constante de temps du filtre en ms

// coefficient du filtre
float A, B;

void controle(void *parameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {

    Vs = A * Ve + B * Vs;

    FlagCalcul = 1;
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

void Vin(void *parameters)
{
  Ve = 1;
  while (1)
  {
    if (Ve == 1)
      Ve = 0;
    else
      Ve = 1;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.printf("Bonjour \n\r");

  xTaskCreate(
      controle,   // nom de la fonction
      "controle", // nom de la tache que nous venons de vréer
      10000,      // taille de la pile en octet
      NULL,       // parametre
      10,         // tres haut niveau de priorite
      NULL        // descripteur
  );
  xTaskCreate(
      Vin,   // nom de la fonction
      "Vin", // nom de la tache que nous venons de vréer
      10000, // taille de la pile en octet
      NULL,  // parametre
      1,     // bas niveau de priorite
      NULL   // descripteur
  );

  // calcul coeff filtre
  A = 1 / (1 + Tau / Te);
  B = Tau / Te * A;
}

void reception(char ch)
{

  static int i = 0;
  static String chaine = "";
  String commande;
  String valeur;
  int index, length;

  if ((ch == 13) or (ch == 10))
  {
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1)
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }

    if (commande == "Tau")
    {
      Tau = valeur.toFloat();
      // calcul coeff filtre
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }
    if (commande == "Te")
    {
      Te = valeur.toInt();
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }

    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}

void loop()
{
  if (FlagCalcul == 1)
  {
    Serial.printf("%lf %lf \n", Ve, Vs);

    FlagCalcul = 0;
  }
}

void serialEvent()
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    reception(Serial.read());
  }
}