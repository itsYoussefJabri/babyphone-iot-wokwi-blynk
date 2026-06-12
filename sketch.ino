#include "config.h"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <string.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_GFX.h>
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

// ===================== SEUILS DE DECISION =====================
// Température recommandée dans le sujet : environ 18°C à 20°C.
const float TEMP_MIN = 18.0;
const float TEMP_MAX = 20.0;

// Bruit simulé avec potentiomètre : 0 à 100%.
const int NOISE_THRESHOLD = 60;

// ===================== TIMERS millis() =====================
const unsigned long SENSOR_INTERVAL_MS = 500;    // Lecture capteurs toutes les 0,5s pour une latence < 1s
const unsigned long DISPLAY_SWITCH_MS  = 3000;   // Changement page OLED toutes les 3s
const unsigned long DISPLAY_REFRESH_MS = 500;    // Rafraîchissement OLED
const unsigned long SERIAL_DEBUG_MS    = 1000;   // Affichage Serial
const unsigned long BLYNK_SEND_MS      = 1000;   // Envoi des jauges Blynk toutes les 1s
const unsigned long BLYNK_CONNECT_MS   = 5000;   // Tentative de reconnexion Blynk

unsigned long lastSensorRead = 0;
unsigned long lastDisplaySwitch = 0;
unsigned long lastDisplayRefresh = 0;
unsigned long lastSerialDebug = 0;
unsigned long lastBlynkSend = 0;
unsigned long lastBlynkConnect = 0;
bool lastWifiConnected = false;
bool lastBlynkConnected = false;

// ===================== ETATS DU SYSTEME =====================
enum BabyState {
  CALM_SLEEP,
  AMBIENT_NOISE,
  AWAKE,
  AGITATION,
  THERMAL_ALERT
};

BabyState currentState = CALM_SLEEP;
BabyState lastBuzzerState = CALM_SLEEP;

// ===================== VARIABLES CAPTEURS =====================
float temperature = 19.0;
float humidity = 50.0;
bool motionDetected = false;
int noiseRaw = 0;
int noisePercent = 0;

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

  setupBlynk();
  readSensors();
  updateState();
  updateLeds();
  resetBuzzerPattern();
}

// =============================================================
// LOOP PRINCIPALE - AUCUN delay()
// =============================================================
void loop() {
  unsigned long now = millis();

  updateBlynk(now);

  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;
    readSensors();
    updateState();
    updateLeds();
  }

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
// LECTURE DES CAPTEURS
// =============================================================
void readSensors() {
  // DHT22 : température et humidité
  float newTemp = dht.readTemperature(false, true);
  float newHum = dht.readHumidity();

  // Si la lecture DHT échoue, on garde les dernières valeurs valides.
  if (!isnan(newTemp)) {
    temperature = newTemp;
  }

  if (!isnan(newHum)) {
    humidity = newHum;
  }

  // PIR : HIGH = mouvement detecté
  motionDetected = (digitalRead(PIR_PIN) == HIGH);

  // Potentiomètre : bruit simulé de 0 à 100%
  noiseRaw = analogRead(NOISE_PIN);
  noisePercent = map(noiseRaw, 0, 4095, 0, 100);
  noisePercent = constrain(noisePercent, 0, 100);
}

// =============================================================
// LOGIQUE DE DECISION AVEC PRIORITES
// =============================================================
void updateState() {
  BabyState oldState = currentState;

  bool thermalProblem = (temperature < TEMP_MIN || temperature > TEMP_MAX);
  bool loudNoise = (noisePercent >= NOISE_THRESHOLD);

  // Priorité 1 : alerte thermique
  if (thermalProblem) {
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
  }
}

// =============================================================
// GESTION DU BUZZER SANS delay()
// =============================================================
void updateBuzzer() {
  if (!buzzerEnabled) {
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
    display.print("PIR  : ");
    display.println(motionToText(motionDetected));
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

    if (currentState == THERMAL_ALERT) {
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
// INTEGRATION BLYNK
// V0 = temperature, V1 = humidite, V2 = bruit, V3 = switch buzzer
// =============================================================
bool isBlynkConfigured() {
  return strcmp(BLYNK_AUTH_TOKEN, "YOUR_BLYNK_AUTH_TOKEN") != 0;
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
  Blynk.virtualWrite(V4, stateToText(currentState));
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
  Serial.print("% | Etat=");
  Serial.println(stateToText(currentState));
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
    default:
      return "Inconnu";
  }
}

String motionToText(bool motion) {
  return motion ? "Oui" : "Non";
}
