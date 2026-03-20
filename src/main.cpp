#include <Arduino.h>
#include <String.h>
#include <Wire.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- Variables d'Asservissement ---
char FlagCalcul = 0;
float Ve, Vs = 0;
float Te = 5;               // période d'échantillonage en ms
float Tau = 500;            // constante de temps du filtre en ms
float Kp = 15.0;
float Kd = 1.0;

float ax, ay, gz;
float gyroz;
float angleAcc;
float E_filtrer;
float angle_filtre = 0;
float Oeq = 0;
float angle_offset;
float ec_final,Ec,erreur;
float Ec_offset;
int pwm;


float erreur_precedente = 0.0;
unsigned long t_precedent = 0;

// --- Pins Mini L298N ---
const int IN1 = 19;
const int IN2 = 18;
const int IN3 = 17;
const int IN4 = 16;

// PWM (LEDC) configuration for ESP32
const int PWM_FREQ = 20000;
const int PWM_RES = 10;       // 10-bit resolution (0-1023)
const int PWM_CH1 = 0;
const int PWM_CH2 = 1;
const int PWM_CH3 = 2;
const int PWM_CH4 = 3;

// coefficient du filtre
float A, B;

float angle();
void initgyro();
void controle(void *parameters);

Adafruit_MPU6050 mpu;
SemaphoreHandle_t dataMutex = xSemaphoreCreateMutex();

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  //  Wire.begin(21, 22);

  // Setup motor pins and LEDC channels
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH3, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH4, PWM_FREQ, PWM_RES);
  ledcAttachPin(IN1, PWM_CH1);
  ledcAttachPin(IN2, PWM_CH2);
  ledcAttachPin(IN3, PWM_CH3);
  ledcAttachPin(IN4, PWM_CH4);

  // initialize timing
  t_precedent = millis();

  // calcul coeff filtre
  A = 1 / (1 + Tau / Te);
  B = Tau / Te * A;

  initgyro();

  Serial.println("Setup terminé, lancement de la tâche de contrôle...");

  xTaskCreate(
      controle,
      "Controle",
      16384,
      NULL,
      1,
      NULL);
}

void initgyro()
{
  if (!mpu.begin())
  {
    Serial.println("MPU non détecté");
    while (1)
    {
      delay(10);
    }                                                       // Boucle infinie pour indiquer l'erreur
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
}

float angle()
{
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  ay = a.acceleration.y;
  ax = a.acceleration.x;
  gz = g.gyro.z;

  angleAcc = atan2(ay, ax) * 180.0 / PI;

  gyroz = -(gz * Tau / 1000.0) * 180.0 / PI;                // Convertir en degrés et appliquer le filtre pour compenser l'offset

  E_filtrer = gyroz + angleAcc;                             // Combinaison du gyroscope et de l'accéléromètre pour obtenir une estimation plus stable de l'angle
  angle_filtre = A * E_filtrer + B * angle_filtre;          // Application du filtre pour lisser l'estimation de l'angle

  return angle_filtre;
}

void controle(void *parameters)
{
  static float Ec = 0;
  TickType_t t_precedent;

  t_precedent = xTaskGetTickCount();                        // Initialiser le temps précédent pour vTaskDelayUntil

  while (1)
  {

    erreur = Oeq - angle();
    Ec = erreur * Kp - Kd * (gz * (180/PI));                // Test avec un Kp = 29.0 -- pas mal
    ec_final = Ec;

    if (Ec>0) Ec += Ec_offset;                              // compensation de couple de forttement sec  +
    if (Ec<0) Ec -= Ec_offset;                              //                    "                      -

    int pwm = constrain((int)abs(Ec), 0, 1023);             // On utilise abs(pwm) pour la valeur de puissance et on contraint entre 0 et 1023

    if (Ec > 0)
    {
      ledcWrite(PWM_CH1, pwm);
      ledcWrite(PWM_CH2, 0);
      ledcWrite(PWM_CH3, 0);
      ledcWrite(PWM_CH4, pwm);
    }
    else
    {
      ledcWrite(PWM_CH1, 0);
      ledcWrite(PWM_CH2, pwm);
      ledcWrite(PWM_CH3, pwm);
      ledcWrite(PWM_CH4, 0);
    }
    FlagCalcul = 1;
    vTaskDelayUntil(&t_precedent, pdMS_TO_TICKS(Te));       // Attendre jusqu'à la prochaine période d'échantillonnage
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

    if (commande == "Kp")   Kp = valeur.toFloat();
    if (commande == "Kd")   Kd = valeur.toFloat();
    if (commande == "Ec_offset")   Ec_offset = valeur.toFloat();
    if (commande == "pwm")   pwm = valeur.toInt();
    
    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}


void serialEvent()
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    reception(Serial.read());
  }
}


void loop()
{
  if (FlagCalcul == 1)
  {
    Serial.printf("%f %f %f   %f %f\n", angleAcc, gyroz, angle_filtre, pwm, Ec_offset);
    FlagCalcul = 0;
  }
}
