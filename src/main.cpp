#include <Arduino.h>        // core Arduino API (Serial, delay, etc.)
#include <Wire.h>           // I2C for MPU6050
#include <MPU6050.h>        // MPu6050 library (ajoutée dans platformio.ini)
#include <String.h>         // String utilities

// --- Objet MPU et constantes ---
MPU6050 mpu;               // instance du capteur MPU6050
const float GYRO_SENS = 131.0f; // facteur de conversion brut->deg/s pour ±250°/s (à ajuster selon configuration)

// --- Variables partagées entre tâches (protégées par mutex) ---
SemaphoreHandle_t mutex;   // mutex FreeRTOS pour protéger les variables partagées
volatile float theta_meas = 0.0f;   // angle estimé (°) lu depuis le capteur / filtre
volatile float omega_meas = 0.0f;   // vitesse angulaire mesurée (°/s)
volatile float control_output = 0.0f; // commande calculée envoyée à l'actionneur

// --- Paramètres de contrôle modifiables via série ---
float Kp = 1.0f;           // gain proportionnel
float Kd = 0.0f;           // gain dérivatif (sur la vitesse angulaire)
float theta_set = 0.0f;    // consigne d'angle (°)

// --- Paramètres filtre / échantillonnage (comme dans main_asservi) ---
float Te = 10.0f;          // période d'échantillonnage contrôleur en ms
float Tau = 1000.0f;       // constante de temps du filtre en ms
float A = 0.0f, B = 0.0f;  // coefficients du filtre discret (calculés dans setup)

// --- Drapeau pour affichage série depuis la loop() principale ---
volatile bool FlagPrint = false; // indique qu'une nouvelle valeur est prête à être imprimée

// Prototype des tâches
void taskMPU(void *parameters);
void taskControl(void *parameters);
void taskSerial(void *parameters);

// Fonction utilitaire: calculer les coefficients du filtre (A et B)
void computeFilterCoeffs()
{
  // A = 1 / (1 + Tau / Te)
  // B = (Tau / Te) * A
  // Ces formules proviennent du filtre passe-bas discret utilisé précédemment.
  if (Te <= 0.0f)
  {
    A = 1.0f;
    B = 0.0f;
  }
  else
  {
    A = 1.0f / (1.0f + Tau / Te);
    B = (Tau / Te) * A;
  }
}

void setup()
{
  // --- Initialisation matérielle de base ---
  Serial.begin(115200);                 // démarre le port série à 115200 bauds
  Wire.begin();                         // initialise le bus I2C nécessaire au MPU6050

  // --- Créer le mutex pour protéger les variables partagées ---
  mutex = xSemaphoreCreateMutex();      // crée un mutex FreeRTOS
  // Si mutex == NULL -> gestion d'erreur possible (non faite ici pour garder le squelette simple)

  // --- Initialisation du MPU6050 (squelette) ---
  mpu.initialize();                     // initialise le capteur (driver fournit la fonction)
  // Note: vérifier mpu.testConnection() et configurer les plages si nécessaire (ex: setFullScaleGyroRange)

  // --- Calcul des coefficients du filtre ---
  computeFilterCoeffs();                // remplit A et B à partir de Te et Tau

  // --- Création des tâches FreeRTOS ---
  // Tâche lecture MPU : haute fréquence / dédiée à la mise à jour angle et vitesse
  xTaskCreate(
      taskMPU,          // fonction
      "MPU",            // nom de la tâche
      4096,             // taille pile (ajuster selon besoin)
      NULL,             // paramètre passé
      5,                // priorité (moyenne)
      NULL);

  // Tâche contrôleur : exécute la loi de commande à la période Te
  xTaskCreate(
      taskControl,
      "Control",
      4096,
      NULL,
      10,               // priorité plus haute pour respecter la périodicité
      NULL);

  // Tâche communication série : parse les commandes et ajuste paramètres
  xTaskCreate(
      taskSerial,
      "Serial",
      4096,
      NULL,
      1,                // priorité basse
      NULL);

  // Ne pas exécuter loop() en parallèle : supprimer la tâche setup()
  vTaskDelete(NULL);
}

void loop()
{
  // Cette loop() ne s'exécute pas car on supprime la tâche setup() avec vTaskDelete(NULL).
  // On garde la fonction vide pour compatibilité Arduino.
  if (FlagPrint)
  {
    // Protège l'accès aux variables partagées pendant l'impression
    if (xSemaphoreTake(mutex, (TickType_t)10) == pdTRUE)
    {
      // Impression simple des grandeurs utiles
      Serial.printf("Consigne: %.2f °, Mesure: %.2f °, Omega: %.2f °/s, Cmd: %.2f\n",
                    theta_set, theta_meas, omega_meas, control_output);
      xSemaphoreGive(mutex);
    }
    FlagPrint = false; // reset du drapeau
  }
  delay(10);
}

/* -----------------------
   Tâche de lecture MPU6050
   - lit les données brutes du capteur
   - calcule omega_meas (vitesse angulaire) en °/s
   - intègre omega pour estimer theta (simple intégration + filtre discret)
   - protège les variables partagéess avec mutex
   ----------------------- */
void taskMPU(void *parameters)
{
  // variables temporaires pour les lectures brutes
  int16_t ax, ay, az, gx, gy, gz;
  // variable de temps pour intégration
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(5); // fréquence de lecture MPU (5 ms = 200Hz)
  float theta_local = 0.0f; // variable locale d'estimation d'angle pour limiter sections critiques

  while (1)
  {
    // attendre la prochaine échéance
    vTaskDelayUntil(&lastWake, period);

    // lire les valeurs brutes (API courante: getMotion6)
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // retourne gx, gy, gz en unités brutes

    // conversion de la vitesse angulaire brute -> °/s
    // GYRO_SENS dépend de la configuration du capteur (±250°/s => 131 LSB/(°/s))
    float omega_z = (float)gz / GYRO_SENS; // rotation autour de l'axe Z en °/s

    // intégration simple pour obtenir un angle (theta) : theta += omega * dt
    // dt en secondes = period / 1000.0
    float dt = (float)pdTICKS_TO_MS(period) / 1000.0f; // périod en s
    theta_local = A * (omega_z * dt) + B * theta_local; // applique le filtre discret A,B (ex: filtrage / intégration)
    // NOTE: cette ligne applique un filtre simple inspiré de main_asservi; à adapter pour un vrai estimateur (complémentary/MAD/kalman)

    // protéger l'écriture des variables partagées
    if (xSemaphoreTake(mutex, (TickType_t)10) == pdTRUE)
    {
      omega_meas = omega_z;        // mise à jour de la vitesse angulaire mesurée
      theta_meas = theta_local;    // mise à jour de l'angle estimé
      xSemaphoreGive(mutex);
    }
  }
}

/* -----------------------
   Tâche contrôleur (périodique)
   - lit la consigne et la mesure protégées par mutex
   - calcule la commande e_c = Kp * erreur + Kd * (dérivée)
   - place la commande dans control_output et met à jour FlagPrint
   - ici la dérivée est prise directement de omega_meas (rate gyro)
   ----------------------- */
void taskControl(void *parameters)
{
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS((uint32_t)Te); // période de contrôle configurable

  while (1)
  {
    vTaskDelayUntil(&lastWake, period);

    // récupérer les grandeurs protégées
    float theta_loc = 0.0f;
    float omega_loc = 0.0f;
    float theta_cons = 0.0f;

    if (xSemaphoreTake(mutex, (TickType_t)10) == pdTRUE)
    {
      theta_loc = theta_meas;
      omega_loc = omega_meas;
      theta_cons = theta_set;
      xSemaphoreGive(mutex);
    }

    // calcul erreur simple
    float error = theta_cons - theta_loc;            // erreur angulaire
    // loi de commande simple P + D (D basé sur la vitesse angulaire)
    float ec = Kp * error + Kd * (-omega_loc);      // le signe sur la dérivée dépend de la convention (à valider expérimentalement)

    // protéger l'écriture de la commande
    if (xSemaphoreTake(mutex, (TickType_t)10) == pdTRUE)
    {
      control_output = ec;
      xSemaphoreGive(mutex);
    }

    // TODO: appeler ici la fonction d'actionneur (p.ex. PWM) pour appliquer control_output
    // Exemple (ESP32): ledcWrite(channel, map(constrain(ec,...), ...));
    // On met le drapeau pour afficher dans loop()
    FlagPrint = true;
  }
}

/* -----------------------
   Tâche série : parse des commandes simples pour ajuster Kp, Kd, Te, Tau, consigne
   Format attendu : "Kp 2.5" ou "Kd 0.1" ou "Te 10" ou "Tau 1000" ou "Set 5"
   Chaque commande se termine par CR/LF.
   ----------------------- */
void taskSerial(void *parameters)
{
  String line = "";
  while (1)
  {
    // lire ce qui est disponible sur la liaison série
    while (Serial.available() > 0)
    {
      char c = (char)Serial.read();
      if (c == '\r' || c == '\n')
      {
        if (line.length() > 0)
        {
          // découper commande / valeur
          int idx = line.indexOf(' ');
          String cmd = (idx == -1) ? line : line.substring(0, idx);
          String val = (idx == -1) ? "" : line.substring(idx + 1);
          float v = val.toFloat();

          // application des commandes (attention aux accès concurrents)
          if (cmd == "Kp")
          {
            if (xSemaphoreTake(mutex, (TickType_t)10) == pdTRUE)
            {
              Kp = v;
              xSemaphoreGive(mutex);
            }
            Serial.printf("Kp -> %.3f\n", v);
          }
          else if (cmd == "Kd")
          {
            if (xSemaphoreTake(mutex, (TickType_t)10) == pdTRUE)
            {
              Kd = v;
              xSemaphoreGive(mutex);
            }
            Serial.printf("Kd -> %.3f\n", v);
          }
          else if (cmd == "Te")
          {
            if (v > 0)
            {
              if (xSemaphoreTake(mutex, (TickType_t)10) == pdTRUE)
              {
                Te = v;
                xSemaphoreGive(mutex);
              }
              computeFilterCoeffs(); // recalculer A,B si nécessaire
              Serial.printf("Te -> %.0f ms\n", v);
            }
          }
          else if (cmd == "Tau")
          {
            if (xSemaphoreTake(mutex, (TickType_t)10) == pdTRUE)
            {
              Tau = v;
              xSemaphoreGive(mutex);
            }
            computeFilterCoeffs();
            Serial.printf("Tau -> %.0f ms\n", v);
          }
          else if (cmd == "Set")
          {
            if (xSemaphoreTake(mutex, (TickType_t)10) == pdTRUE)
            {
              theta_set = v;
              xSemaphoreGive(mutex);
            }
            Serial.printf("Setpoint -> %.2f °\n", v);
          }
          else
          {
            Serial.printf("Commande inconnue: %s\n", line.c_str());
          }
        }
        line = ""; // reset de la ligne
      }
      else
      {
        line += c; // accumulation des caractères
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // petite attente pour laisser le CPU à d'autres tâches
  }
}