// Ce code est un programme de contrôle pour un gyropode utilisant un ESP32. 
// Il utilise des capteurs MPU6050 pour mesurer l'angle et la vitesse, 
// ainsi que des encodeurs pour mesurer la position des roues. 
// Le programme implémente un contrôle en boucle fermée pour maintenir le gyropode en équilibre. 
// Les paramètres de contrôle peuvent être ajustés via la communication série.

#include <Arduino.h>
#include <String.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Encoder.h>

// --- Variables d'Asservissement ---
char FlagCalcul = 0;
float Te = 5;               // période d'échantillonage en ms
float Tau = 500;            // constante de temps du filtre en ms
float Kp = 20.0;
float Kd = 0.6;
float Vcons = 0;
float Kpv;
float Kdv;
float Oeq = -1.20;     // -3.5499573 angle trouver  avec le cable
int C0g = 585, C0d = 589;
int incr = 748;             // nombre d'incrément par tour de roue
float R = 0.035;            // rayon de la roue en m

float ax, ay, gz;
float gyroz;
float angleAcc;
float E_filtrer;
float angle_filtre;
float Vmes;
float ec_final, erreur, Ec, Ecd, Ecg;
volatile int pwmg, pwmd;
float encd = 0, encg = 0;
float posg=0, posd=0, pos_precg = 0, pos_precd = 0;
float vd=0, vg=0;

float eVin, ePVit, dVit, eVit;
float vLinF=0;
float Ocons,Teta, eTeta, posd_ref, posg_ref;

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

ESP32Encoder encoderD;
ESP32Encoder encoderG;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  //  Wire.begin(21, 22);

  // use pin 33 and 32 for the first encoder
	encoderG.attachHalfQuad(33, 32);
	// use pin 34 and 35 for the second encoder
	encoderD.attachHalfQuad(35, 34);
  
  encoderG.setCount(0);
  encoderD.setCount(0);

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
  TickType_t t_precedent;

  t_precedent = xTaskGetTickCount();                        // Initialiser le temps précédent pour vTaskDelayUntil

  while (1)
  {
    // Calcul de la vitesse de déplacement du gyropode
    encd = encoderD.getCount();
    encg = encoderG.getCount();

    posd = encd* (360 * R) / incr;
    posg = encg* (360 * R) / incr;

    vd = (posd - pos_precd) / (Te*1000); // en m/s
    vg = (posg - pos_precg) / (Te*1000); // en m/s
    
   
    // Vitesse linéaire calculée à partir des vitesses des roues
    vLinF = (vd + vg) / 2; 

    // Calcul des variables de Asservissement de Vitesse
    eVit = Vcons - vLinF;     // Erreur de la Vitesse
    dVit = eVit - ePVit;      // Dérivé du kdVitesse
    
    // Mise à jour des positions précédentes pour le prochain calcul de vitesse
    pos_precd = posd;
    pos_precg = posg;

    // Mise à jour de ePVit pour le prochain calcul de dVit
    ePVit = eVit;

    // Asservissement Vitesse
    Ocons = Kpv * eVit + Kdv * dVit;
    //Ocons = constrain(Ocons, -2.0 / 180 * PI, 2.0 / 180 * PI);
    eTeta = Ocons - Teta; // Erreur de Position

    erreur = Ocons + Oeq - angle();
    Ec = erreur * Kp + Kd * (gz * (180/PI));                
    Ecd = Ec;
    Ecg = Ec;

    if (Ec>0) {Ecd += C0d; Ecg += C0g;}                      // compensation de couple de forttement sec  +
    if (Ec<0) {Ecd -= C0d; Ecg -= C0g;}                      //                    "                      -

    pwmd = constrain((int)abs(Ecd), 0, 1023);                // On utilise abs(Ec) pour la valeur de puissance et on contraint entre 0 et 1023
    pwmg = constrain((int)abs(Ecg), 0, 1023);

    if (Ecd > 0)
    {
      ledcWrite(PWM_CH1, pwmd);
      ledcWrite(PWM_CH2, 0);
      ledcWrite(PWM_CH3, 0);
      ledcWrite(PWM_CH4, pwmg);
    }
    else
    {
      ledcWrite(PWM_CH1, 0);
      ledcWrite(PWM_CH2, pwmd);
      ledcWrite(PWM_CH3, pwmg);
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
    if (commande == "Kpv")   Kpv = valeur.toFloat();
    if (commande == "Kdv")   Kdv = valeur.toFloat();

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
   // Serial.printf("%f %f %f %f\n", angle_filtre, Ec, Ocons, vLinF);
   Serial.printf("%f %f %f %f\n",posd, posg, vLinF, Ocons);
    FlagCalcul = 0;
  }
}
// motg 585 motd 589