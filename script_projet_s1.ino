#include <Arduino.h>
#include <RTClib.h>
#include <MD_Parola.h>
#include <ss_oled.h>
#include <EEPROM.h>

#define VALIDE_MENU A1

#define PM_AM_SWITCH 13
#define LED_SECONDE 12
#define REVEIL_BUZZER 11

#define TRIGPIN 10
#define ECHOPIN 9

#define PINA_ENCODEUR 8
#define PINB_ENCODEUR 7

#define DATA_PIN 6
#define CS_PIN 5
#define CLK_PIN 4
 
#define BOUTON_MENU 3
#define REVEIL_BUTTON 2

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

unsigned long previousMinuteEdit = 0;

volatile byte reveil = 1;
byte alarmePasse = 0;
unsigned long previousReveilEdit = 0;
unsigned long previousReveilStart = 0;

unsigned long alarmPreviousMillis = 0;
byte alarmState = 0;

int etatPrecedentLigneCLK;

volatile byte flag = 0;

SSOLED ssoled;
MD_Parola myDisplay = MD_Parola(MD_MAX72XX::FC16_HW, DATA_PIN, CLK_PIN, CS_PIN, 4);
RTC_DS1307 RTC;
DateTime reveilTime;

void clsOled() {
  oledFill(&ssoled, 0, 1);
}

void flagMenu() {
  flag = 1;
}

int detectionMain() {
  //digitalWrite(TRIGPIN, HIGH);
  PORTD = PORTD | (1 << TRIGPIN);
  delayMicroseconds(10);
  //digitalWrite(TRIGPIN, LOW);
  PORTD = PORTD & !(1 << TRIGPIN);

  // Mesure la durée de l'impulsion reçue sur la broche Echo
  int Duree = pulseIn(ECHOPIN, HIGH);
  //int Duree = 0;
  // Calcule la distance en cm
  int a = Duree * 0.034 / 2;
  if (a >= 10 || a < 1) {
    return 0;
  } else {
    return 1;
  }
}

void updateLedMatrix(DateTime time) {
  char heureAffichable[6];
  short heures = time.hour();
  short heureFormat;
  char heuresStr[3];
  short minutes = time.minute();
  char minutesStr[3];


  if (digitalRead(PM_AM_SWITCH) == HIGH && heures > 12) {
      heureFormat = heures - 12;
  } else {
    heureFormat = heures;
  }

  itoa(heureFormat, heuresStr, 10);

  if (heureFormat < 10) {
    heureAffichable[0] = '0';
    heureAffichable[1] = heuresStr[0];
  } else {
    heureAffichable[0] = heuresStr[0];
    heureAffichable[1] = heuresStr[1];
  }

  heureAffichable[2] = ':';

  itoa(minutes, minutesStr, 10);

  if (minutes < 10) {
    heureAffichable[3] = '0';
    heureAffichable[4] = minutesStr[0];
  } else {
    heureAffichable[3] = minutesStr[0];
    heureAffichable[4] = minutesStr[1];
  }

  heureAffichable[5] = '\0';
  
  myDisplay.print(heureAffichable);
  // Serial.println(heureAffichable);
}

void updateOled(DateTime time) {
  clsOled();
  if (digitalRead(PM_AM_SWITCH) == HIGH) {
    if (time.hour() >= 12) {
      oledWriteString(&ssoled, 0, 0, 0, "PM", FONT_SMALL, 0, 1);
    } else {
      oledWriteString(&ssoled, 0, 0, 0, "AM", FONT_SMALL, 0, 1);
    }
  }
  if (reveil) {
    oledWriteString(&ssoled, 0, (int) OLED_WIDTH/2, 0, "ALARME ON", FONT_SMALL, 0, 1);
  } else {
    oledWriteString(&ssoled, 0, (int) OLED_WIDTH/2, 0, "ALARME OFF", FONT_SMALL, 0, 1);
  }

  char heureStr[3];
  itoa(RTC.now().hour(), heureStr, 10);
  char minuteStr[3];
  itoa(RTC.now().minute(), minuteStr, 10);
  oledWriteString(&ssoled, 0, 0, 2, heureStr, FONT_SMALL, 0, 1);
  oledWriteString(&ssoled, 0, -1, -1, ":", FONT_SMALL, 0, 1);
  oledWriteString(&ssoled, 0, -1, -1, minuteStr, FONT_SMALL, 0, 1);
}

void alarm(DateTime now) {
  if (reveil && !alarmePasse && !alarmState && now.hour() == reveilTime.hour() && now.minute() == reveilTime.minute()) {
    previousReveilStart = millis();
    //digitalWrite(REVEIL_BUZZER, HIGH);
    PORTD = PORTD | (1 << REVEIL_BUZZER);
    alarmState = 1;
  }

  if (alarmState && millis() - alarmPreviousMillis >= 100) {
    digitalWrite(REVEIL_BUZZER, !digitalRead(REVEIL_BUZZER));
    alarmPreviousMillis = millis();
  }

  // Éteindre l'alarme après 20 secondes
  byte detect = detectionMain();
  if ((alarmState && millis() - previousReveilStart >= 20000) || (detect && alarmState)) {
    if (detect) {
      EEPROM.put(0, millis() - previousReveilStart);
    }
    // digitalWrite(REVEIL_BUZZER, LOW);
    PORTD = PORTD & !(1 << REVEIL_BUZZER);
    alarmState = 0;
    alarmePasse = 1;
  }

  if (millis() - previousReveilStart >= 120000) {
    alarmePasse = 0;
  }
}

void toggleReveil() {
  reveil = !reveil;
  // Serial.println((reveil ? F("Réveil allumé") : F("Réveil éteint")));
}

void menu() {
  if (flag == 1) {
    // Serial.println(F("Menu !"));
    byte fini = 0;
    short etape = 0;
    short temps = 24;
    short heures = 0;
    short compteur = 0;
    byte boutonAppuie = 0;
    clsOled();
    if (etape < 2) {
      oledWriteString(&ssoled, 0, 0, 0, "Reveil", FONT_SMALL, 0, 1);
    } else {
      oledWriteString(&ssoled, 0, 0, 0, "Heure", FONT_SMALL, 0, 1);
    }
    if (etape == 0 || etape == 2) {
      char minuteStr[3];
      itoa(compteur, minuteStr, 10);
      oledWriteString(&ssoled, 0, 0, 2, minuteStr, FONT_SMALL, 0, 1);
      oledWriteString(&ssoled, 0, -1, -1, ":00", FONT_SMALL, 0, 1);
    } else {
      char heureStr[3];
      itoa(heures, heureStr, 10);
      char minuteStr[3];
      itoa(compteur, minuteStr, 10);
      oledWriteString(&ssoled, 0, 0, 2, heureStr, FONT_SMALL, 0, 1);
      oledWriteString(&ssoled, 0, -1, -1, ":", FONT_SMALL, 0, 1);
      oledWriteString(&ssoled, 0, -1, -1, minuteStr, FONT_SMALL, 0, 1);
    }
    while(!fini) {
      short etatActuelDeLaLigneCLK = digitalRead(PINB_ENCODEUR);

      if (!digitalRead(VALIDE_MENU) && !boutonAppuie) {
        boutonAppuie = 1;
        // Serial.print(temps);
        // Serial.print(F(" : "));
        // Serial.println(compteur);
        if (etape == 0 || etape == 2) {
          heures = compteur;
        }
        if (etape == 1 || etape == 3) {
          char heureFinale[9];
          char heureStr[2];
          itoa(heures, heureStr, 10);
          if (heures < 10) {
            heureFinale[0] = '0';
            heureFinale[1] = heureStr[0];
          } else {
            heureFinale[0] = heureStr[0];
            heureFinale[1] = heureStr[1];
          }

          heureFinale[2] = ':';

          char minuteStr[2];
          itoa(compteur, minuteStr, 10);
          if (compteur < 10) {
            heureFinale[3] = '0';
            heureFinale[4] = minuteStr[0];
          } else {
            heureFinale[3] = minuteStr[0];
            heureFinale[4] = minuteStr[1];
          }
          heureFinale[5] = ':';
          heureFinale[6] = '0';
          heureFinale[7] = '0';
          heureFinale[8] = '\0';

          if (etape == 1) {
            reveilTime = DateTime(__DATE__, heureFinale);
          } else {
            RTC.adjust(DateTime(__DATE__, heureFinale));
          }
        }

        if (etape + 1 == 4) {
          fini = 1;
          // Serial.println(F("Bye :)"));
          flag = 0;
          break;
        } else {
          etape++;
          if (temps == 24) {
            temps = 60;
          } else {
            temps = 24;
          }
          compteur = 0;
        }
      }

      if (!digitalRead(VALIDE_MENU) == LOW) {
        boutonAppuie = 0;
      }

      if(etatActuelDeLaLigneCLK != etatPrecedentLigneCLK) {
        etatPrecedentLigneCLK = etatActuelDeLaLigneCLK;
        if(etatActuelDeLaLigneCLK == LOW) {
          if(etatActuelDeLaLigneCLK != digitalRead(PINA_ENCODEUR)) {
            if (compteur + 1 == temps) {
              compteur = 0;
            } else {
              compteur++;
            }
          } else {
            if (compteur == 0) {
              compteur = temps - 1;
            } else {
              compteur--;
            }
          }
          clsOled();
          if (etape < 2) {
            oledWriteString(&ssoled, 0, 0, 0, "Reveil", FONT_SMALL, 0, 1);
          } else {
            oledWriteString(&ssoled, 0, 0, 0, "Heure", FONT_SMALL, 0, 1);
          }
          if (etape == 0 || etape == 2) {
            char minuteStr[3];
            itoa(compteur, minuteStr, 10);
            oledWriteString(&ssoled, 0, 0, 2, minuteStr, FONT_SMALL, 0, 1);
            oledWriteString(&ssoled, 0, -1, -1, ":00", FONT_SMALL, 0, 1);
          } else {
            char heureStr[3];
            itoa(heures, heureStr, 10);
            char minuteStr[3];
            itoa(compteur, minuteStr, 10);
            oledWriteString(&ssoled, 0, 0, 2, heureStr, FONT_SMALL, 0, 1);
            oledWriteString(&ssoled, 0, -1, -1, ":", FONT_SMALL, 0, 1);
            oledWriteString(&ssoled, 0, -1, -1, minuteStr, FONT_SMALL, 0, 1);
          }
        }
      }
      delay(1);
    }
  }
}

void setup () {
  // Serial.begin(9600);
  Wire.begin();
  RTC.begin();
  
  //       oledInit(SSOLED*, type,        oled_addr, rotate180, invert, bWire, SDA_PIN, SCL_PIN, RESET_PIN, speed)
  short rc = oledInit(&ssoled, OLED_128x64, 0x3C,      0,         0,      1,     A4,      A5,      -1,        400000L); // use standard I2C bus at 400Khz
  if (rc == -1) {
  }

  if (!RTC.isrunning()) {
    // Serial.println(F("RTC is NOT running!"));
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }

  previousMinuteEdit = millis();

  pinMode(LED_SECONDE, OUTPUT);
  pinMode(PM_AM_SWITCH, INPUT);
  pinMode(REVEIL_BUTTON, INPUT);
  pinMode(REVEIL_BUZZER, OUTPUT);
  pinMode(PINA_ENCODEUR, INPUT);
  pinMode(PINB_ENCODEUR, INPUT);
  pinMode(BOUTON_MENU, INPUT);
  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);
  pinMode(VALIDE_MENU, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(REVEIL_BUTTON), toggleReveil, FALLING);
  attachInterrupt(digitalPinToInterrupt(BOUTON_MENU), flagMenu, RISING);

  etatPrecedentLigneCLK = digitalRead(PINB_ENCODEUR);

  cli();
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS10); // horloge interne divisée par 1024
  TIMSK1 = _BV(OCIE1A); // it sur comparateur
  TCNT1 = 0;
  OCR1A = (F_CPU / 1000 / 1024) * 500; // 500ms est une demi-période des secondes
  sei();

  myDisplay.begin();
  myDisplay.displayClear();
  myDisplay.setTextAlignment(PA_CENTER);

  updateLedMatrix(RTC.now());
  updateOled(RTC.now());

  delay(200);
}

ISR(TIMER1_COMPA_vect) {
  // fait clignoter la led à 1/2 periode de 1s
  digitalWrite(LED_SECONDE, !digitalRead(LED_SECONDE));
}

void loop () {
  DateTime now = RTC.now();

  updateOled(now);
  if (millis() - previousMinuteEdit >= 3000) {
    updateLedMatrix(now);
    previousMinuteEdit = millis();
  }
  
  menu();
  alarm(now);
}
