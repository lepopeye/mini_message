#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager
#include <MD_MAX72xx.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESPUI.h>
#include <WiFi.h>     // Ajout pour ESP32
#include <ESPmDNS.h>  // Bibliothèque pour mDNS sur ESP32

// Update these with values suitable for your network.
#define MQTT_SERVER "192.168.1.100"
#define MQTT_PORT 1883
#define MQTT_USER ""
#define MQTT_PASSWORD ""

#define MQTT_TOPIC_SUB "monpetitmessage/elo/message"
#define MQTT_TOPIC_SUB_CONFIRM "monpetitmessage/elo/state"

#define MQTT_TOPIC_PUB "monpetitmessage/wouam/message"
#define MQTT_TOPIC_PUB_CONFIRM "monpetitmessage/wouam/state"

#define PIN_BUTTON 27

#define ESP_NAME "Mon petit message pour W"
#define ESP_MDNS "elle"

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define MAX_INTENSITY 2
#define CLK_PIN 18      // or SCK
#define DATA_PIN 19     // or MOSI
#define CS_PIN 5        // or SS
#define DELAYTIME 100   // in milliseconds
#define CHAR_SPACING 1  // pixels between characters
#define BUF_SIZE 75     // Global message buffers shared by Serial and Scrolling functions
#define COL_SIZE 8      // Définit la taille de la colonne pour le module LED

MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

// Debug statements
#define DEBUG 1
#if DEBUG
#define PRINT(s, x) \
  { \
    Serial.print(F(s)); \
    Serial.print(x); \
  }
#define PRINTS(x) Serial.print(F(x))
#define PRINTD(x) Serial.println(x, DEC)
#else
#define PRINT(s, x)
#define PRINTS(x)
#define PRINTD(x)
#endif

WiFiClient espClient;
PubSubClient client(espClient);

String receivedMessage;                   // Variable pour stocker le message reçu
String lastMessage;                       // Variable pour stocker le dernier message
String lastStatus = "";                   // État par défaut
unsigned long lastButtonPress = 0;        // Pour le debounce
const unsigned long debounceDelay = 200;  // Délai de debounce en millisecondes

uint16_t textInput;  // Ajout de la déclaration manquante
uint16_t lblStatus;  // Ajout d'une variable pour le label d'état
uint16_t buttonId;
uint16_t lblLastMessage;

void textCallback(Control* sender, int type) {
  String message = sender->value;
  Serial.println("Sending message: " + message);

  // Publier le message sur MQTT
  client.publish(MQTT_TOPIC_PUB, message.c_str());
  client.publish(MQTT_TOPIC_PUB_CONFIRM, "wait");
}

void buttonCallback(Control* sender, int type) {
  Serial.println("Button clicked");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  PRINTS("\n[MD_MAX72XX Test & Demo]");
  if (!mx.begin()) {
    PRINTS("\nMD_MAX72XX initialization failed");
  }
  mx.control(MD_MAX72XX::INTENSITY, MAX_INTENSITY);

  // WIFI
  printTextNoAnim("wifi ...", 1);
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setMinimumSignalQuality(10);
  wm.setConfigPortalTimeout(30);
  wm.setScanDispPerc(true);

  bool res = wm.autoConnect(ESP_NAME);
  if (!res) {
    Serial.println("Failed to connect to WiFi");
    printTextNoAnim("wifi ko !", 1);
    delay(4000);
  } else {
    Serial.println("connected...yeey :)");
    printTextNoAnim("wifi ok", 2);
    delay(1000);

    // Configuration de mDNS pour rendre l'ESP accessible via http://monesp.local
    if (MDNS.begin(ESP_MDNS)) {  // Remplacez "monesp" par le nom que vous préférez
      Serial.println("mDNS démarré. Accédez à http://monpetitmessage.local");
    } else {
      Serial.println("Échec du démarrage mDNS");
    }
  }
  printTextNoAnim("", 0);

  // Initialize MQTT
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setKeepAlive(60);  // Définit un timeout de 60 secondes
  client.setCallback(callback);

  // Connect to MQTT
  connectToMQTT();

  // Initialize ESPUI
  ESPUI.begin(ESP_NAME);

  // Add a Text Input for the message without a callback
  textInput = ESPUI.addControl(ControlType::Text, "Message (200 car. max)", "", ControlColor::Turquoise);
  ESPUI.addControl(ControlType::Max, "", "200", ControlColor::None, textInput);
  ESPUI.getControl(textInput)->callback = textCallback;

  // Add a Button to send the message without a callback
  buttonId = ESPUI.addControl(ControlType::Button, "Envoyer", "Envoyer", ControlColor::Peterriver);
  ESPUI.getControl(buttonId)->callback = buttonCallback;

  lblLastMessage = ESPUI.addControl(ControlType::Label, "Dernier message", lastStatus, ControlColor::Dark);
  lblStatus = ESPUI.addControl(ControlType::Label, "État du Message", lastStatus, ControlColor::Dark);
}

// Mettre à jour le texte dans l'interface web
void updateWebUI(const String& message) {
  //ESPUI.getControl(textInput)->value = message;  // Mettre à jour la valeur du contrôle de message
  //ESPUI.updateControl(textInput);                // Mettre à jour l'affichage

  ESPUI.getControl(lblStatus)->value = message;  // Mettre à jour la valeur du contrôle d'état
  ESPUI.updateControl(lblStatus);                // Mettre à jour l'affichage
}

// Mettre à jour le texte dans l'interface web
void updateLastMessageUI(const String& message) {
  //ESPUI.getControl(textInput)->value = message;  // Mettre à jour la valeur du contrôle de message
  //ESPUI.updateControl(textInput);                // Mettre à jour l'affichage

  ESPUI.getControl(lblLastMessage)->value = message;  // Mettre à jour la valeur du contrôle d'état
  ESPUI.updateControl(lblLastMessage);                // Mettre à jour l'affichage
}

// Mettre à jour le texte dans l'interface web
void updateInputUI(const String& message) {

  ESPUI.getControl(textInput)->value = message;  // Mettre à jour la valeur du contrôle de message
  ESPUI.updateControl(textInput);                // Mettre à jour l'affichage
}


void connectToMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT... ");
    if (client.connect(ESP_MDNS, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("Connected");
      client.subscribe(MQTT_TOPIC_SUB);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  // Récupérer le message
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';             // Terminer la chaîne de caractères
  Serial.println(message);            // Afficher le message sur le moniteur série
  receivedMessage = String(message);  // Stocker le message reçu

  // Vérifiez si le message provient du bon topic
  if (strcmp(topic, MQTT_TOPIC_SUB) == 0) {
    // Publier une confirmation de lecture
    String confirmationMessage = "received";
    client.publish(MQTT_TOPIC_SUB_CONFIRM, confirmationMessage.c_str());

    // dernier message
    updateLastMessageUI(message);  // Mettre à jour l'état sur l'interface web

    // champs input reset
    updateInputUI("");  // Mettre à jour l'état sur l'interface web

    lastStatus = "Message reçu (non lu)";  // Mettre à jour l'état
    updateWebUI(lastStatus);               // Mettre à jour l'état sur l'interface web
  }
}

void publishTestMessage() {
  StaticJsonDocument<200> doc;
  //doc["message"] = "Hello, world!";
  doc["timestamp"] = millis();
  char jsonBuffer[512];
  size_t n = serializeJson(doc, jsonBuffer);
  client.publish(MQTT_TOPIC_PUB, jsonBuffer, n);
}

void scrollText(const char* p, int intensity) {
  mx.control(MD_MAX72XX::INTENSITY, intensity);
  int numSpaces = 11;
  uint8_t charWidth;
  uint8_t cBuf[8];
  char buffer[100];
  snprintf(buffer, sizeof(buffer), "%s%*s", p, numSpaces, "");
  mx.clear();
  const char* msg = buffer;
  while (*msg != '\0') {
    charWidth = mx.getChar(*msg++, sizeof(cBuf) / sizeof(cBuf[0]), cBuf);
    for (uint8_t i = 0; i <= charWidth; i++) {
      mx.transform(MD_MAX72XX::TSL);
      mx.setColumn(0, (i < charWidth) ? cBuf[i] : 0);
      delay(DELAYTIME);
    }
  }
}

void printTextNoAnim(const char* message, int intensity) {
  mx.control(MD_MAX72XX::INTENSITY, intensity);
  char buffer[BUF_SIZE];
  snprintf(buffer, BUF_SIZE, "%s", message);
  printText(0, MAX_DEVICES - 1, buffer);
}

void printText(uint8_t modStart, uint8_t modEnd, const char* pMsg) {
  uint8_t state = 0;
  uint8_t curLen;
  uint16_t showLen;
  uint8_t cBuf[8];
  int16_t col = ((modEnd + 1) * COL_SIZE) - 1;

  mx.control(modStart, modEnd, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);

  do {
    switch (state) {
      case 0:
        if (*pMsg == '\0') {
          showLen = col - (modEnd * COL_SIZE);
          state = 2;
          break;
        }
        showLen = mx.getChar(*pMsg++, sizeof(cBuf) / sizeof(cBuf[0]), cBuf);
        curLen = 0;
        state++;
      case 1:
        mx.setColumn(col--, cBuf[curLen++]);
        if (curLen == showLen) {
          showLen = CHAR_SPACING;
          state = 2;
        }
        break;
      case 2:
        curLen = 0;
        state++;
      case 3:
        mx.setColumn(col--, 0);
        curLen++;
        if (curLen == showLen)
          state = 0;
        break;
      default:
        col = -1;
    }
  } while (col >= (modStart * COL_SIZE));

  mx.control(modStart, modEnd, MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}

void loop() {
  if (!client.connected()) {
    connectToMQTT();
  }
  client.loop();

  // Lire l'état du bouton
  if (digitalRead(PIN_BUTTON) == LOW) {  // Vérifiez si le bouton est pressé
    unsigned long currentTime = millis();
    if (currentTime - lastButtonPress > debounceDelay) {  // Vérifiez le debounce
      lastButtonPress = currentTime;                      // Mettre à jour le temps de dernier appui
      if (receivedMessage.length() > 0) {                 // Vérifiez s'il y a un message à afficher
        String confirmationMessage = "reading";
        client.publish(MQTT_TOPIC_SUB_CONFIRM, confirmationMessage.c_str());  // Envoie la confirmation
        lastStatus = "Lecture";                                               // Mettre à jour l'état
        updateWebUI(lastStatus);                                              // Mettre à jour l'état sur l'interface web
        scrollText(receivedMessage.c_str(), 5);                               // Affichez le message sur l'écran
        receivedMessage = "";                                                 // Réinitialiser le message après l'affichage
        confirmationMessage = "read";
        client.publish(MQTT_TOPIC_SUB_CONFIRM, confirmationMessage.c_str());  // Envoie la confirmation
        lastStatus = "Message lu";                                            // Mettre à jour l'état
        updateWebUI(lastStatus);                                              // Mettre à jour l'état sur l'interface web
      } else {
        scrollText("...", 5);
        String confirmationMessage = "no message";
        client.publish(MQTT_TOPIC_SUB_CONFIRM, confirmationMessage.c_str());  // Envoie la confirmation
      }
    }
  }
}
