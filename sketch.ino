#include "config.h"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <string.h>
#include <math.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>

// ===================== PINS SELON diagram.json =====================
#define DHT_PIN        4
#define PIR_PIN        14
#define NOISE_PIN      34

#define LED_GREEN_PIN  25
#define LED_BLUE_PIN   27
#define LED_RED_PIN    26
#define BUZZER_PIN     12

#define OLED_SDA       21
#define OLED_SCL       22

// ===================== BLYNK / WIFI =====================
const char WIFI_SSID[] = WIFI_NETWORK_NAME;
const char WIFI_PASS[] = WIFI_NETWORK_PASSWORD;

// ===================== CAPTEUR DHT22 =====================
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// ===================== OLED SSD1306 =====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===================== MPU6050 =====================
Adafruit_MPU6050 mpu;

// ===================== SEUILS DE DECISION =====================
// Température recommandée dans le sujet : environ 18°C à 20°C.
const float TEMP_MIN = 18.0;
const float TEMP_MAX = 20.0;

// Bruit simulé avec potentiomètre : 0 à 100%.
const int NOISE_THRESHOLD = 60;

// Une acceleration normale au repos est proche de 9,81 m/s2.
const float FALL_FREE_FALL_THRESHOLD = 3.0;
const float FALL_IMPACT_THRESHOLD = 20.0;

// ===================== TIMERS millis() =====================
const unsigned long SENSOR_INTERVAL_MS = 500;    // Lecture capteurs toutes les 0,5s pour une latence < 1s
const unsigned long DISPLAY_SWITCH_MS  = 3000;   // Changement page OLED toutes les 3s
const unsigned long DISPLAY_REFRESH_MS = 500;    // Rafraîchissement OLED
const unsigned long SERIAL_DEBUG_MS    = 1000;   // Affichage Serial
const unsigned long BLYNK_SEND_MS      = 1000;   // Envoi des jauges Blynk toutes les 1s
const unsigned long BLYNK_CONNECT_MS   = 5000;   // Tentative de reconnexion Blynk
const unsigned long SUMMARY_INTERVAL_MS = 30000; // 2 heures (30s pour test rapide dans Wokwi)
const unsigned long MPU_INTERVAL_MS    = 100;    // Detection chute en moins de 1s
const unsigned long FALL_ALERT_HOLD_MS = 1000;   // Maintien visible de l'alerte

unsigned long lastSensorRead = 0;
unsigned long lastDisplaySwitch = 0;
unsigned long lastDisplayRefresh = 0;
unsigned long lastSerialDebug = 0;
unsigned long lastBlynkSend = 0;
unsigned long lastBlynkConnect = 0;
unsigned long lastSummaryMs = 0;
unsigned long lastMpuRead = 0;
unsigned long lastFallDetectedMs = 0;
bool lastWifiConnected = false;
bool lastBlynkConnected = false;
unsigned long lastSensorValuesMs = 0;
unsigned long lastAlertLatencyMs = 0;

// ===================== ETATS DU SYSTEME =====================
enum BabyState {
  CALM_SLEEP,
  AMBIENT_NOISE,
  AWAKE,
  AGITATION,
  THERMAL_ALERT,
  FALL_ALERT
};

BabyState currentState = CALM_SLEEP;
BabyState lastBuzzerState = CALM_SLEEP;

// ===================== VARIABLES CAPTEURS =====================
float temperature = 19.0;
float humidity = 50.0;
bool motionDetected = false;
int noiseRaw = 0;
int noisePercent = 0;

// ===================== VARIABLES MPU6050 =====================
bool mpuAvailable = false;
bool fallDetected = false;
float accelerationX = 0.0;
float accelerationY = 0.0;
float accelerationZ = 9.81;
float accelerationMagnitude = 9.81;
unsigned long lastFallAlertLatencyMs = 0;

// ===================== RESUME SUR 2 HEURES =====================
float periodTemperatureMin = 0.0;
float periodTemperatureMax = 0.0;
unsigned long periodWakeCount = 0;
bool periodTemperatureInitialized = false;
bool previousMotionDetected = false;

float lastSummaryTemperatureMin = 0.0;
float lastSummaryTemperatureMax = 0.0;
unsigned long lastSummaryWakeCount = 0;
bool lastSummaryAvailable = false;

// ===================== VARIABLES OLED =====================
bool displayPage = false; // false = page capteurs, true = page état

// ===================== VARIABLES BUZZER NON-BLOQUANT =====================
unsigned long buzzerTimer = 0;
bool buzzerActive = false;
bool buzzerPatternStarted = false;
int buzzerStep = 0;
bool buzzerEnabled = true;

// ===================== VARIABLES BLYNK =====================
BabyState lastBlynkNotifiedState = CALM_SLEEP;
int lastBlynkThermalType = 0; // -1 = froid, 0 = normal, 1 = chaud

// ===================== PROTOTYPES =====================
void readSensors();
void setupMpu();
void readMpu(unsigned long now);
void updateState();
void updateLeds();
void updateBuzzer();
void updateDisplay();
void drawNoiseBar(int x, int y, int width, int height, int percent);
void setupBlynk();
void updateBlynk(unsigned long now);
void sendBlynkData();
void sendBlynkNotification();
bool isBlynkConfigured();
String blynkStateText();
void updateTwoHourSummary(unsigned long now);
void printDebug();

void resetBuzzerPattern();
void handleAwakeBeeps();
void handleAgitationBeeps();

String stateToText(BabyState state);
String motionToText(bool motion);

BLYNK_WRITE(V3) {
  buzzerEnabled = (param.asInt() == 1);

  if (!buzzerEnabled) {
    resetBuzzerPattern();
  }
}

// =============================================================
// SETUP
// =============================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIR_PIN, INPUT);
  pinMode(NOISE_PIN, INPUT);

  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  noTone(BUZZER_PIN);

  dht.begin();

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("Erreur : OLED SSD1306 non detecte !");
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Babyphone IoT");
    display.println("Initialisation...");
    display.display();
  }

  setupMpu();
  readMpu(millis());
  setupBlynk();
  unsigned long setupSensorStart = millis();
  readSensors();
  updateState();
  updateLeds();
  lastAlertLatencyMs = millis() - setupSensorStart;
  lastSummaryMs = millis();
  resetBuzzerPattern();
}

// =============================================================
// LOOP PRINCIPALE - AUCUN delay()
// =============================================================
void loop() {
  unsigned long now = millis();

  updateBlynk(now);

  if (now - lastMpuRead >= MPU_INTERVAL_MS) {
    lastMpuRead = now;
    bool previousFallDetected = fallDetected;
    unsigned long fallCycleStart = millis();

    readMpu(now);

    if (fallDetected != previousFallDetected) {
      updateState();
      updateLeds();
      lastFallAlertLatencyMs = millis() - fallCycleStart;
    }
  }

  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;
    unsigned long sensorCycleStart = millis();
    readSensors();
    updateState();
    updateLeds();
    lastAlertLatencyMs = millis() - sensorCycleStart;
  }

  updateTwoHourSummary(now);

  updateBuzzer();

  if (now - lastDisplaySwitch >= DISPLAY_SWITCH_MS) {
    lastDisplaySwitch = now;
    displayPage = !displayPage;
  }

  if (now - lastDisplayRefresh >= DISPLAY_REFRESH_MS) {
    lastDisplayRefresh = now;
    updateDisplay();
  }

  if (now - lastSerialDebug >= SERIAL_DEBUG_MS) {
    lastSerialDebug = now;
    printDebug();
  }
}

// =============================================================
// MPU6050 - DETECTION DE CHUTE
// =============================================================
void setupMpu() {
  if (!mpu.begin()) {
    Serial.println("Erreur : MPU6050 non detecte !");
    mpuAvailable = false;
    return;
  }

  mpuAvailable = true;
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 connecte.");
}

void readMpu(unsigned long now) {
  if (!mpuAvailable) {
    return;
  }

  sensors_event_t accelerationEvent;
  sensors_event_t gyroEvent;
  sensors_event_t temperatureEvent;
  mpu.getEvent(&accelerationEvent, &gyroEvent, &temperatureEvent);

  accelerationX = accelerationEvent.acceleration.x;
  accelerationY = accelerationEvent.acceleration.y;
  accelerationZ = accelerationEvent.acceleration.z;
  accelerationMagnitude = sqrt(
    accelerationX * accelerationX +
    accelerationY * accelerationY +
    accelerationZ * accelerationZ
  );

  bool abnormalAcceleration =
    accelerationMagnitude < FALL_FREE_FALL_THRESHOLD ||
    accelerationMagnitude > FALL_IMPACT_THRESHOLD;

  if (abnormalAcceleration) {
    fallDetected = true;
    lastFallDetectedMs = now;
  } else if (fallDetected && now - lastFallDetectedMs >= FALL_ALERT_HOLD_MS) {
    fallDetected = false;
  }
}

// =============================================================
// LECTURE DES CAPTEURS
// =============================================================
void readSensors() {
  // DHT22 : température et humidité
  float newTemp = dht.readTemperature(false, true);
  float newHum = dht.readHumidity();

  // Si la lecture DHT échoue, on garde les dernières valeurs valides.
  if (!isnan(newTemp)) {
    temperature = newTemp;

    if (!periodTemperatureInitialized) {
      periodTemperatureMin = newTemp;
      periodTemperatureMax = newTemp;
      periodTemperatureInitialized = true;
    } else {
      periodTemperatureMin = min(periodTemperatureMin, newTemp);
      periodTemperatureMax = max(periodTemperatureMax, newTemp);
    }
  }

  if (!isnan(newHum)) {
    humidity = newHum;
  }

  // PIR : HIGH = mouvement detecté
  motionDetected = (digitalRead(PIR_PIN) == HIGH);

  // Un front montant du PIR correspond a un nouvel eveil detecte.
  if (motionDetected && !previousMotionDetected) {
    periodWakeCount++;
  }
  previousMotionDetected = motionDetected;

  // Potentiomètre : bruit simulé de 0 à 100%
  noiseRaw = analogRead(NOISE_PIN);
  noisePercent = map(noiseRaw, 0, 4095, 0, 100);
  noisePercent = constrain(noisePercent, 0, 100);

  lastSensorValuesMs = millis();
}

// =============================================================
// LOGIQUE DE DECISION AVEC PRIORITES
// =============================================================
void updateState() {
  BabyState oldState = currentState;

  bool thermalProblem = (temperature < TEMP_MIN || temperature > TEMP_MAX);
  bool loudNoise = (noisePercent >= NOISE_THRESHOLD);

  // Priorite 1 : chute detectee
  if (fallDetected) {
    currentState = FALL_ALERT;
  }
  // Priorite 2 : alerte thermique
  else if (thermalProblem) {
    currentState = THERMAL_ALERT;
  }
  // Priorité 2 : agitation = mouvement + bruit élevé
  else if (motionDetected && loudNoise) {
    currentState = AGITATION;
  }
  // Priorité 3 : éveil = mouvement seulement
  else if (motionDetected) {
    currentState = AWAKE;
  }
  // Priorité 4 : bruit seul = bruit ambiant
  else if (loudNoise) {
    currentState = AMBIENT_NOISE;
  }
  // Priorité 5 : sommeil calme
  else {
    currentState = CALM_SLEEP;
  }

  if (currentState != oldState) {
    resetBuzzerPattern();
  }
}

// =============================================================
// GESTION DES LEDS
// =============================================================
void updateLeds() {
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);

  switch (currentState) {
    case CALM_SLEEP:
      digitalWrite(LED_GREEN_PIN, HIGH);
      break;

    case AMBIENT_NOISE:
      digitalWrite(LED_GREEN_PIN, HIGH);
      break;

    case AWAKE:
      digitalWrite(LED_BLUE_PIN, HIGH);
      break;

    case AGITATION:
      digitalWrite(LED_BLUE_PIN, HIGH);
      digitalWrite(LED_RED_PIN, HIGH);
      break;

    case THERMAL_ALERT:
      digitalWrite(LED_RED_PIN, HIGH);
      break;

    case FALL_ALERT:
      digitalWrite(LED_RED_PIN, HIGH);
      break;
  }
}

// =============================================================
// GESTION DU BUZZER SANS delay()
// =============================================================
void updateBuzzer() {
  if (!buzzerEnabled && currentState != FALL_ALERT) {
    noTone(BUZZER_PIN);
    buzzerActive = false;
    buzzerPatternStarted = false;
    buzzerStep = 0;
    lastBuzzerState = currentState;
    return;
  }

  if (currentState != lastBuzzerState) {
    resetBuzzerPattern();
    lastBuzzerState = currentState;
  }

  switch (currentState) {
    case CALM_SLEEP:
    case AMBIENT_NOISE:
      noTone(BUZZER_PIN);
      break;

    case AWAKE:
      // 2 bips doux, répétés de temps en temps
      handleAwakeBeeps();
      break;

    case AGITATION:
      // Bips plus rapides tant que l'agitation continue
      handleAgitationBeeps();
      break;

    case THERMAL_ALERT:
      // Alerte prioritaire : bip continu
      tone(BUZZER_PIN, 1200);
      break;

    case FALL_ALERT:
      // Une chute reste prioritaire meme en mode nuit.
      tone(BUZZER_PIN, 1800);
      break;
  }
}

void resetBuzzerPattern() {
  noTone(BUZZER_PIN);
  buzzerTimer = millis();
  buzzerActive = false;
  buzzerPatternStarted = false;
  buzzerStep = 0;
}

// Etat AWAKE : deux bips courts puis pause
void handleAwakeBeeps() {
  unsigned long now = millis();

  if (!buzzerPatternStarted) {
    tone(BUZZER_PIN, 700);
    buzzerActive = true;
    buzzerPatternStarted = true;
    buzzerStep = 0;
    buzzerTimer = now;
    return;
  }

  switch (buzzerStep) {
    case 0: // Premier bip ON
      if (now - buzzerTimer >= 150) {
        noTone(BUZZER_PIN);
        buzzerActive = false;
        buzzerStep = 1;
        buzzerTimer = now;
      }
      break;

    case 1: // Petite pause entre les deux bips
      if (now - buzzerTimer >= 150) {
        tone(BUZZER_PIN, 700);
        buzzerActive = true;
        buzzerStep = 2;
        buzzerTimer = now;
      }
      break;

    case 2: // Deuxième bip ON
      if (now - buzzerTimer >= 150) {
        noTone(BUZZER_PIN);
        buzzerActive = false;
        buzzerStep = 3;
        buzzerTimer = now;
      }
      break;

    case 3: // Pause longue avant de répéter les 2 bips
      if (now - buzzerTimer >= 5000) {
        buzzerPatternStarted = false;
        buzzerStep = 0;
      }
      break;
  }
}

// Etat AGITATION : bips rapides ON/OFF
void handleAgitationBeeps() {
  unsigned long now = millis();

  if (!buzzerPatternStarted) {
    tone(BUZZER_PIN, 950);
    buzzerActive = true;
    buzzerPatternStarted = true;
    buzzerTimer = now;
    return;
  }

  unsigned long duration = buzzerActive ? 200 : 250;

  if (now - buzzerTimer >= duration) {
    buzzerTimer = now;
    buzzerActive = !buzzerActive;

    if (buzzerActive) {
      tone(BUZZER_PIN, 950);
    } else {
      noTone(BUZZER_PIN);
    }
  }
}

// =============================================================
// AFFICHAGE OLED : 2 PAGES ALTERNÉES TOUTES LES 3s
// =============================================================
void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (!displayPage) {
    // Page 1 : valeurs des capteurs
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Babyphone IoT");
    display.println("----------------");

    display.print("T:");
    display.print(temperature, 1);
    display.print("C  H:");
    display.print(humidity, 1);
    display.println(" %");

    display.print("Bruit: ");
    display.print(noisePercent);
    display.println(" %");
    drawNoiseBar(0, 37, 100, 8, noisePercent);

    display.setCursor(0, 52);
    display.print("PIR:");
    display.print(motionToText(motionDetected));
    display.print(" A:");
    display.print(accelerationMagnitude, 1);
  } else {
    // Page 2 : état et alertes
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Etat systeme");
    display.println("----------------");

    display.setTextSize(1);
    display.print("Etat: ");
    display.println(stateToText(currentState));

    display.println();

    if (currentState == FALL_ALERT) {
      display.println("CHUTE DETECTEE!");
      display.print("Accel: ");
      display.print(accelerationMagnitude, 1);
      display.println(" m/s2");
      display.println("Intervention!");
    } else if (currentState == THERMAL_ALERT) {
      display.println("ALERTE THERMIQUE!");
      if (temperature < TEMP_MIN) {
        display.println("Chambre trop froide");
      } else {
        display.println("Chambre trop chaude");
      }
    } else if (currentState == AGITATION) {
      display.println("Mouvement + bruit");
      display.println("Bebe agite");
    } else if (currentState == AWAKE) {
      display.println("Mouvement detecte");
      display.println("Bebe reveille");
    } else if (currentState == AMBIENT_NOISE) {
      display.println("Bruit sans mouvement");
      display.println("Bruit ambiant");
    } else {
      display.println("Situation normale");
      display.println("Sommeil calme");
    }
  }

  display.display();
}

void drawNoiseBar(int x, int y, int width, int height, int percent) {
  int fillWidth = map(percent, 0, 100, 0, width - 2);
  fillWidth = constrain(fillWidth, 0, width - 2);

  display.drawRect(x, y, width, height, SSD1306_WHITE);
  display.fillRect(x + 1, y + 1, fillWidth, height - 2, SSD1306_WHITE);
  display.setCursor(x + width + 4, y);
  display.print("max");
}

// =============================================================
// RESUME PERIODIQUE SUR 2 HEURES
// Pour un test rapide dans Wokwi, mettre SUMMARY_INTERVAL_MS a 30000.
// =============================================================
void updateTwoHourSummary(unsigned long now) {
  if (now - lastSummaryMs < SUMMARY_INTERVAL_MS) {
    return;
  }

  lastSummaryMs = now;
  lastSummaryWakeCount = periodWakeCount;
  lastSummaryAvailable = periodTemperatureInitialized;

  Serial.println();
  Serial.println("========== RESUME 2H ==========");

  if (periodTemperatureInitialized) {
    lastSummaryTemperatureMin = periodTemperatureMin;
    lastSummaryTemperatureMax = periodTemperatureMax;

    Serial.print("Temperature min: ");
    Serial.print(lastSummaryTemperatureMin, 1);
    Serial.println(" C");
    Serial.print("Temperature max: ");
    Serial.print(lastSummaryTemperatureMax, 1);
    Serial.println(" C");
  } else {
    Serial.println("Temperature min/max: aucune mesure valide");
  }

  Serial.print("Nombre d'eveils: ");
  Serial.println(lastSummaryWakeCount);
  Serial.println("===============================");
  Serial.println();

  periodTemperatureInitialized = false;
  periodWakeCount = 0;
}

// =============================================================
// INTEGRATION BLYNK
// V0 = temperature, V1 = humidite, V2 = bruit, V3 = switch buzzer
// V5 = temperature min 2h, V6 = temperature max 2h, V7 = eveils 2h
// =============================================================
bool isBlynkConfigured() {
  return strcmp(BLYNK_AUTH_TOKEN, "YOUR_BLYNK_AUTH_TOKEN") != 0;
}

String blynkStateText() {
  if (currentState == THERMAL_ALERT) {
    if (temperature < TEMP_MIN) {
      return "Alerte thermique - Chambre trop froide";
    }

    return "Alerte thermique - Chambre trop chaude";
  }

  return stateToText(currentState);
}

void setupBlynk() {
  if (!isBlynkConfigured()) {
    Serial.println("Blynk non configure : remplacer YOUR_BLYNK_AUTH_TOKEN.");
    return;
  }

  Serial.println("Connexion WiFi Wokwi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS, 6);
  Blynk.config(BLYNK_AUTH_TOKEN, "blynk.cloud", 80);
}

void updateBlynk(unsigned long now) {
  if (!isBlynkConfigured()) {
    return;
  }

  bool wifiConnected = (WiFi.status() == WL_CONNECTED);

  if (wifiConnected != lastWifiConnected) {
    lastWifiConnected = wifiConnected;
    Serial.println(wifiConnected ? "WiFi connecte." : "WiFi deconnecte.");
  }

  if (wifiConnected) {
    if (!Blynk.connected() && now - lastBlynkConnect >= BLYNK_CONNECT_MS) {
      lastBlynkConnect = now;
      Serial.println("Connexion Blynk...");
      Blynk.connect(3000);
    }

    if (Blynk.connected()) {
      if (!lastBlynkConnected) {
        lastBlynkConnected = true;
        Serial.println("Blynk connecte.");
      }

      Blynk.run();

      if (now - lastBlynkSend >= BLYNK_SEND_MS) {
        lastBlynkSend = now;
        sendBlynkData();
        sendBlynkNotification();
      }
    } else if (lastBlynkConnected) {
      lastBlynkConnected = false;
      Serial.println("Blynk deconnecte.");
    }
  }
}

void sendBlynkData() {
  Blynk.virtualWrite(V0, temperature);
  Blynk.virtualWrite(V1, humidity);
  Blynk.virtualWrite(V2, noisePercent);
  Blynk.virtualWrite(V4, blynkStateText());

  if (lastSummaryAvailable) {
    Blynk.virtualWrite(V5, lastSummaryTemperatureMin);
    Blynk.virtualWrite(V6, lastSummaryTemperatureMax);
    Blynk.virtualWrite(V7, lastSummaryWakeCount);
  }
}

void sendBlynkNotification() {
  int thermalType = 0;

  if (currentState == THERMAL_ALERT) {
    thermalType = (temperature < TEMP_MIN) ? -1 : 1;
  }

  bool stateChanged = (currentState != lastBlynkNotifiedState);
  bool thermalChanged = (currentState == THERMAL_ALERT && thermalType != lastBlynkThermalType);

  if (!stateChanged && !thermalChanged) {
    return;
  }

  switch (currentState) {
    case FALL_ALERT:
      Blynk.logEvent("chute_detectee", "Chute detectee - intervention immediate");
      break;

    case AWAKE:
      Blynk.logEvent("bebe_reveille", "Bebe reveille");
      break;

    case AGITATION:
      Blynk.logEvent("bebe_agite", "Bebe agite");
      break;

    case THERMAL_ALERT:
      if (thermalType < 0) {
        Blynk.logEvent("temperature_basse", "Temperature chambre trop basse");
      } else {
        Blynk.logEvent("temperature_haute", "Temperature chambre trop haute");
      }
      break;

    default:
      break;
  }

  lastBlynkNotifiedState = currentState;
  lastBlynkThermalType = thermalType;
}

// =============================================================
// DEBUG SERIAL
// =============================================================
void printDebug() {
  Serial.print("Temp=");
  Serial.print(temperature, 1);
  Serial.print("C | Hum=");
  Serial.print(humidity, 1);
  Serial.print("% | PIR=");
  Serial.print(motionToText(motionDetected));
  Serial.print(" | Bruit=");
  Serial.print(noisePercent);
  Serial.print("% | Chute=");
  Serial.print(fallDetected ? "Oui" : "Non");
  Serial.print(" | Etat=");
  Serial.print(stateToText(currentState));

  if (currentState == FALL_ALERT) {
    Serial.print(" | Detail=Acceleration anormale");
    Serial.print(" | LatenceChute=");
    Serial.print(lastFallAlertLatencyMs);
    Serial.print("ms");
  } else if (currentState == THERMAL_ALERT) {
    Serial.print(" | Detail=");
    if (temperature < TEMP_MIN) {
      Serial.print("Chambre trop froide");
    } else {
      Serial.print("Chambre trop chaude");
    }
  }

  Serial.print(" | LatenceMaxDetection=");
  Serial.print(SENSOR_INTERVAL_MS);
  Serial.print("ms | EveilsPeriode=");
  Serial.print(periodWakeCount);

  Serial.println();
}

// =============================================================
// FONCTIONS UTILITAIRES
// =============================================================
String stateToText(BabyState state) {
  switch (state) {
    case CALM_SLEEP:
      return "Sommeil calme";
    case AMBIENT_NOISE:
      return "Bruit ambiant";
    case AWAKE:
      return "Eveil";
    case AGITATION:
      return "Agitation";
    case THERMAL_ALERT:
      return "Alerte thermique";
    case FALL_ALERT:
      return "Chute detectee";
    default:
      return "Inconnu";
  }
}

String motionToText(bool motion) {
  return motion ? "Oui" : "Non";
}
