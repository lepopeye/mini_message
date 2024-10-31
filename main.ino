
#include <MD_MAX72xx.h>

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW 
#define MAX_DEVICES 4
#define MAX_INTENSITY 0x9
#define CLK_PIN   18  // or SCK
#define DATA_PIN  19  // or MOSI
#define CS_PIN    5  // or SS

MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

#define  DELAYTIME  100  // in milliseconds

// Turn on debug statements to the serial output
#define  DEBUG  1

#if  DEBUG
#define PRINT(s, x) { Serial.print(F(s)); Serial.print(x); }
#define PRINTS(x) Serial.print(F(x))
#define PRINTD(x) Serial.println(x, DEC)

#else
#define PRINT(s, x)
#define PRINTS(x)
#define PRINTD(x)

#endif

void setup() {
  // put your setup code here, to run once:
#if  DEBUG
  Serial.begin(57600);
#endif
  PRINTS("\n[MD_MAX72XX Test & Demo]");

  if (!mx.begin())
    PRINTS("\nMD_MAX72XX initialization failed");
mx.control(MD_MAX72XX::INTENSITY, 6);
}

void loop() {
  // Nombre d'espaces ajoutés à la fin du message
  int numSpaces = 10;
  scrollText("je fais un essai", numSpaces);
}

void scrollText(const char *p, int numSpaces) {
  uint8_t charWidth;
  uint8_t cBuf[8];  // this should be ok for all built-in fonts
  char buffer[100]; // Assurez-vous que la taille du buffer est suffisante
  
  // Copier le message initial dans le buffer et ajouter les espaces
  snprintf(buffer, sizeof(buffer), "%s%*s", p, numSpaces, "");
  
  mx.clear();
  const char* msg = buffer;
  
  while (*msg != '\0') {
    charWidth = mx.getChar(*msg++, sizeof(cBuf) / sizeof(cBuf[0]), cBuf);
    for (uint8_t i = 0; i <= charWidth; i++) // allow space between characters
    {
      mx.transform(MD_MAX72XX::TSL);
      mx.setColumn(0, (i < charWidth) ? cBuf[i] : 0);
      delay(DELAYTIME);
 
    }
  }
}
