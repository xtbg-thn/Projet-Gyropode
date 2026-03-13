#include <Arduino.h>
#include <String.h>
#include <Wire.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// --- Variables d'Asservissement ---
char FlagCalcul = 0;
float Ve, Vs = 0;
float Te = 10;    // période d'échantillonage en ms
float Tau = 1000; // constante de temps du filtre en ms
float Kp = 15.0;      
float Kd = 1.0;       /*
float Alpha = 0.2;    */// Nouveau : Lissage de l'accélération
float Oeq = 0.0;      
float angle_filtre = 0.0;
float angle_offset = 0.0;
float erreur_precedente = 0.0;
unsigned long temps_precedent = 0;
float gyro_rate_shared = 0.0;

// --- Pins Mini L298N ---
const int IN1 = 19; 
const int IN2 = 18; 
const int IN3 = 17; 
const int IN4 = 16;

// PWM (LEDC) configuration for ESP32
const int PWM_FREQ = 5000;
const int PWM_RES = 8; // 8-bit resolution (0-255)
const int PWM_CH1 = 0;
const int PWM_CH2 = 1;
const int PWM_CH3 = 2;
const int PWM_CH4 = 3;

// coefficient du filtre
float A, B;

void sensorTask(void* pvParameters);
void controle(void *parameters);
//void Vin(void *parameters);
void reception(char ch);

Adafruit_MPU6050 mpu;
SemaphoreHandle_t dataMutex = xSemaphoreCreateMutex();

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(21, 22);
  Serial.printf("Bonjour \n\r");

  if (!mpu.begin()) {
    Serial.println("MPU non détecté");
  } else {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    angle_offset = atan2(a.acceleration.y, a.acceleration.x) * 180.0 / PI;
  }

  // Setup motor pins and LEDC channels
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES); ledcAttachPin(IN1, PWM_CH1);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES); ledcAttachPin(IN2, PWM_CH2);
  ledcSetup(PWM_CH3, PWM_FREQ, PWM_RES); ledcAttachPin(IN3, PWM_CH3);
  ledcSetup(PWM_CH4, PWM_FREQ, PWM_RES); ledcAttachPin(IN4, PWM_CH4);

  // initialize timing
  temps_precedent = millis();

  xTaskCreate(sensorTask, "sensorTask", 4096, NULL, 2, NULL);
  xTaskCreate(controle, "controle", 10000, NULL, 10, NULL);
  //xTaskCreate(Vin, "Vin", 4096, NULL, 1, NULL);

  // calcul coeff filtre
  A = 1 / (1 + Tau / Te);
  B = Tau / Te * A;
}

void sensorTask(void* pvParameters) {
  sensors_event_t a, g, t;
  for(;;) {
    mpu.getEvent(&a, &g, &t);
    unsigned long now = millis();
    float dt = (now - temps_precedent) / 1000.0;
    if (dt <= 0) dt = 0.001;
    float angle_acc = atan2(a.acceleration.y, a.acceleration.x) * 180.0 / PI;
    float gyro_rate = g.gyro.x * 180.0 / PI;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      // Complementary filter: trust gyro for fast changes, accelerometer for drift correction
      float angle_acc_corrected = angle_acc - angle_offset;
      angle_filtre = 0.98 * (angle_filtre + gyro_rate * dt) + 0.02 * angle_acc_corrected;
      gyro_rate_shared = gyro_rate;
      temps_precedent = now;
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(5)); // ~200 Hz sensor update
  }
}

void controle(void *parameters)
{
  static float ec = 0;
  float local_angle, local_Kp, local_Oeq;

  for(;;) {
    // 1. Récupération sécurisée des données partagées
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      local_angle = angle_filtre;
      local_Kp = Kp;
      local_Oeq = Oeq;
      xSemaphoreGive(dataMutex);
    } else {
      // Si le mutex est occupé, on attend un peu et on recommence la boucle
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    // 2. Calcul simplifié selon ta logique
    static float ec;
    float erreur;
    erreur = angle_filtre - Oeq;
    ec = erreur*local_Kp;

    // 3. Conversion pour le PWM (vitesse)
    // On utilise abs(ec) pour la valeur de puissance et on contraint entre 0 et 255
    int vitesse = constrain((int)abs(ec), 0, 255);

    // 4. Sorties moteurs via LEDC
    if (ec > 0) {
      ledcWrite(PWM_CH1, ec); ledcWrite(PWM_CH2, 0);
      ledcWrite(PWM_CH3, 0);       ledcWrite(PWM_CH4, ec);
    } else {
      ledcWrite(PWM_CH1, 0);       ledcWrite(PWM_CH2, ec);
      ledcWrite(PWM_CH3, ec); ledcWrite(PWM_CH4, 0);
    }

    // Fréquence de la boucle (100 Hz)
    vTaskDelay(pdMS_TO_TICKS(10));
  }
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
    Serial.printf("Ve:%lf Vs:%lf \n", Ve, Vs);

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