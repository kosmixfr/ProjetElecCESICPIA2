#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include <ChainableLED.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <RTClib.h>
#include <string.h>
#include <EEPROM.h>
#include "main.h"
 
#define SIX_SEVEN_SIX 6
#define SIX_SEVEN_SEVEN 7
#define CLK_LED 4
#define DATA_LED 5
#define LED_COUNT 1
#define VERSION 1
#define NUMERO_LOT 1
#define CAPTEUR_TIMEOUT 1000
#define BOUTON_ROUGE 2
#define BOUTON_VERT 3
#define EEPROM_MAGIC 0xACAB
#define EEPROM_VERSION 2
#define EEPROM_LOT 3
#define EEPROM_PARAMS 4
// 1 : un lot de 5 enregistrements ne tient pas dans la RAM libre (~100 o)
#define BATCH_SIZE 1
#define ANTI_REBOND_MS 30
#define DUREE_APPUI_LONG 5000
 
TinyGPSPlus gps;
// latitude gps / longitude gps
// Pile, size :2
float LesKordonnesGps[2];
const int MaxFileSize = 1000;
const int SD_CS = 4;
// library for the BME280 sensor
Adafruit_BME280 capteur;
// clock for the RTC DS3231j
RTC_DS3231 rtc_clock;
 
int SumCapteurError = 0;

enum ModeFonctionnement
{
  MODE_CONFIG,
  MODE_STANDARD,
  MODE_ECO,
  MODE_MAINTENANCE
};

ModeFonctionnement modeActuel = MODE_STANDARD;
 
 
ChainableLED leds(CLK_LED, DATA_LED, LED_COUNT); // CLK=3, DATA=4
 
SoftwareSerial gpsSerial(SIX_SEVEN_SIX, SIX_SEVEN_SEVEN); // RX, TX
 
void SaveEepromParameters()
{
  int adresse = EEPROM_PARAMS;
  for (unsigned int i = 0; i < sizeof(losParametros) / sizeof(Parameters); i++)
  {
    EEPROM.put(adresse, losParametros[i].Value);
    adresse += sizeof(long);
  }
}

void InitLed()
{
  leds.setColorRGB(0, 0, 0, 0);
}

void InitEeprom()
{
  uint16_t magic = 0;
  EEPROM.get(0, magic);

  if (magic != EEPROM_MAGIC)
  {
    EEPROM.put(0, (uint16_t)EEPROM_MAGIC);
    EEPROM.put(2, (uint8_t)VERSION);
    EEPROM.put(3, (uint8_t)NUMERO_LOT);
    SaveEepromParameters();
    return;
  }

  EEPROM.put(2, (uint8_t)VERSION);
  EEPROM.put(3, (uint8_t)NUMERO_LOT);

  int adresse = EEPROM_PARAMS;
  for (unsigned int i = 0; i < sizeof(losParametros) / sizeof(Parameters); i++)
  {
    long valeur = losParametros[i].Value;
    EEPROM.get(adresse, valeur);
    if (valeur >= losParametros[i].MinValue && valeur <= losParametros[i].MaxValue)
    {
      losParametros[i].Value = valeur;
    }
    adresse += sizeof(long);
  }
}

void InitRtc()
{
  if (!rtc_clock.begin())
  {
    OnImprimeLerreurViaLed(1);
  }
}

void InitSd()
{
  if (!SD.begin(SD_CS))
  {
    OnImprimeLerreurViaLed(5);
  }
}

void InitCapteur()
{
  if (!capteur.begin(0x76))
  {
    if (!capteur.begin(0x77))
    {
      Serial.println(F("Erreur : BME280 introuvable !"));
      OnImprimeLerreurViaLed(3);
    }
  }
}

struct EtatBouton
{
  uint8_t pin;
  bool enfonce;
  bool appui;
  bool appuiLong;
  bool longSignale; // appui long deja declenche pour cette pression
  unsigned long derniereModif;
  unsigned long debut;
};

volatile EtatBouton boutonRouge;
volatile EtatBouton boutonVert;

static inline void ScannerBouton(volatile EtatBouton &bouton, unsigned long maintenant)
{
  bool niveau = digitalRead(bouton.pin) == LOW;

  if (niveau != bouton.enfonce && (maintenant - bouton.derniereModif) >= ANTI_REBOND_MS)
  {
    bouton.enfonce = niveau;
    bouton.derniereModif = maintenant;
    if (niveau)
    {
      bouton.appui = true;
      bouton.debut = maintenant;
      bouton.appuiLong = false;
      bouton.longSignale = false;
    }
  }

  // longSignale evite de re-armer appuiLong tant que le bouton reste enfonce
  if (bouton.enfonce && !bouton.longSignale && (maintenant - bouton.debut) >= DUREE_APPUI_LONG)
  {
    bouton.appuiLong = true;
    bouton.longSignale = true;
  }
}

ISR(TIMER1_COMPA_vect)
{
  unsigned long maintenant = millis();
  ScannerBouton(boutonRouge, maintenant);
  ScannerBouton(boutonVert, maintenant);
}

static bool PrendreEvenement(volatile bool &drapeau)
{
  noInterrupts();
  bool valeur = drapeau;
  drapeau = false;
  interrupts();
  return valeur;
}

void InitBoutons()
{
  pinMode(BOUTON_ROUGE, INPUT_PULLUP);
  pinMode(BOUTON_VERT, INPUT_PULLUP);

  boutonRouge.pin = BOUTON_ROUGE;
  boutonVert.pin = BOUTON_VERT;
  boutonRouge.enfonce = digitalRead(BOUTON_ROUGE) == LOW;
  boutonVert.enfonce = digitalRead(BOUTON_VERT) == LOW;
  boutonRouge.appui = boutonRouge.enfonce;
  boutonVert.appui = boutonVert.enfonce;
  boutonRouge.debut = millis();
  boutonVert.debut = millis();
  boutonRouge.derniereModif = millis();
  boutonVert.derniereModif = millis();

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 249;
  TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10);
  TIMSK1 = _BV(OCIE1A);
  interrupts();
}

bool BoutonRougePresse()
{
  return boutonRouge.enfonce;
}

bool BoutonVertPresse()
{
  return boutonVert.enfonce;
}

bool BoutonRougeAppui()
{
  return PrendreEvenement(boutonRouge.appui);
}

bool BoutonVertAppui()
{
  return PrendreEvenement(boutonVert.appui);
}

bool BoutonRougeAppuiLong()
{
  return PrendreEvenement(boutonRouge.appuiLong);
}

bool BoutonVertAppuiLong()
{
  return PrendreEvenement(boutonVert.appuiLong);
}

void setup()
{
  Serial.begin(9600);
  Serial.println(F("Lancement"));
  InitGps();
  InitBoutons();

  Wire.setWireTimeout(CAPTEUR_TIMEOUT);
  Wire.begin();

  InitLed();
  InitRtc();
  InitSd();
  InitEeprom();
  InitCapteur();

  modeActuel = (BoutonRougePresse() || BoutonRougeAppui()) ? MODE_CONFIG : MODE_STANDARD;
}

// Ecrit v avec 'decimales' decimales dans out, sans printf flottant (economie de flash).
// Renvoie un pointeur vers la fin de la chaine ecrite.
char *FormatDecimal(float v, uint8_t decimales, char *out)
{
  char *p = out;
  if (v < 0)
  {
    *p++ = '-';
    v = -v;
  }
  long facteur = 1;
  for (uint8_t i = 0; i < decimales; i++)
  {
    facteur *= 10;
  }
  long valeur = (long)(v * facteur + 0.5f);
  p += sprintf_P(p, PSTR("%ld."), valeur / facteur);
  long fraction = valeur % facteur;
  for (long diviseur = facteur / 10; diviseur > 0; diviseur /= 10)
  {
    *p++ = '0' + (fraction / diviseur) % 10;
  }
  *p = '\0';
  return p;
}

// Ouvre le fichier AAMMJJ_i.LOG du jour (i de 0 a 9) dont la taille est sous FILE_MAX_SIZE.
// Nom au format 8.3 : la bibliotheque SD ne gere pas les noms longs.
// false=erreur
bool GetFile(DateTime *date, File &fichier)
{
  long max_size = GetParam(F("FILE_MAX_SIZE"))->Value;
  char nomFichier[13];
  for (int i = 0; i < 10; i++)
  {
    sprintf_P(nomFichier, PSTR("%02d%02d%02d_%d.LOG"),
            date->year() % 100, date->month(), date->day(), i);
    fichier = SD.open(nomFichier, FILE_WRITE);
    if (!fichier)
    {
      fichier.close();
      OnImprimeLerreurViaLed(5);
      return false;
    }
    if ((long)fichier.size() < max_size)
    {
      return true;
    }
    fichier.close();
  }
  OnImprimeLerreurViaLed(6);
  return false;
}
 

// Lit les octets recus du GPS. A appeler souvent : le buffer SoftwareSerial ne fait que 64 octets.
void PompeGps()
{
  while (gpsSerial.available())
  {
    gps.encode(gpsSerial.read());
  }
}

// Initialise le GPS et attend le fix satellite (bloquant).
// A appeler au debut du setup : ne revient que quand la position est valide.
// Sans messages serie pour economiser la flash (Uno presque pleine).
void InitGps()
{
  gpsSerial.begin(9600);
  while (!gps.location.isValid())
  {
    PompeGps();
  }
  LesKordonnesGps[0] = gps.location.lat();
  LesKordonnesGps[1] = gps.location.lng();
}

Record *GetNewRecord(int withGps)
{
  // Position : fix GPS frais si demandé, sinon dernière position connue
  if (withGps)
  {
    PompeGps();
    if (!gps.location.isValid())
    {
      OnImprimeLerreurViaLed(2);
      return NULL;
    }
    LesKordonnesGps[0] = gps.location.lat();
    LesKordonnesGps[1] = gps.location.lng();
  }
  // utilisation impossible au 0,0 du globe (aucune position connue)
  if (LesKordonnesGps[0] == 0 && LesKordonnesGps[1] == 0)
  {
    OnImprimeLerreurViaLed(2);
    return NULL;
  }

  // Capteur BME280 : pression convertie de Pa en hPa
  unsigned long timeout = GetParam(F("TIMEOUT"))->Value;
  unsigned long debut = millis();
  float temperature = capteur.readTemperature();
  float pression = capteur.readPressure() / 100.0f;
  float humidity = capteur.readHumidity();
  unsigned long duree = millis() - debut;

  if (isnan(temperature) || isnan(pression) || isnan(humidity) || duree > timeout)
  {
    SumCapteurError++;
    if (SumCapteurError >= 2)
    {
      OnImprimeLerreurViaLed(3);
    }
    return NULL;
  }
  SumCapteurError = 0;

  if ((ValidCapteurData(F("MIN_TEMP_AIR"), F("MAX_TEMP_AIR"), temperature) + ValidCapteurData(F("HYGR_MINT"), F("HYGR_MAXT"), humidity) + ValidCapteurData(F("PRESSURE_MIN"), F("PRESSURE_MAX"), pression)) > 0)
  {
    return NULL;
  }

  DateTime maintenant = rtc_clock.now();

  Record *record_ptr = (Record *)malloc(sizeof(Record));
  if (record_ptr == NULL)
  {
    return NULL;
  }
  record_ptr->Latitude = LesKordonnesGps[0];
  record_ptr->Longitude = LesKordonnesGps[1];
  record_ptr->Date = maintenant;
  record_ptr->Temperature = temperature;
  record_ptr->Humidity = humidity;
  record_ptr->Pressure = pression;
 
  return record_ptr;
}
 

const __FlashStringHelper *ProcessCommande(char commande[], char value[])
{
  Parameters *param = GetParam(commande);
  if (param == NULL)
  {
    return F("Commande inconnu");
  }
  // Parsing manuel d'un entier signe (remplace strtol, ~500 o de flash).
  // value doit etre termine par '\0'.
  char *p = value;
  bool negatif = false;
  if (*p == '-')
  {
    negatif = true;
    p++;
  }
  if (*p == '\0')
  {
    return F("Nombre invalide");
  }
  long nombre = 0;
  for (; *p != '\0'; p++)
  {
    if (*p < '0' || *p > '9')
    {
      return F("Nombre invalide");
    }
    if (nombre > 100000000L)
    {
      return F("Valeur out of bound !!");
    }
    nombre = nombre * 10 + (*p - '0');
  }
  if (negatif)
  {
    nombre = -nombre;
  }
  if (nombre < param->MinValue || nombre > param->MaxValue)
  {
    return F("Valeur out of bound !!");
  }
  param->Value = nombre;
  return NULL;
}
 
// Recherche par nom saisi en RAM (commande recue par la liaison serie)
Parameters *GetParam(const char *name)
{
  for (int i = 0; i < sizeof(losParametros) / sizeof(Parameters); i++)
  {
    if (strcmp_P(name, losParametros[i].Name) == 0)
    {
      return &losParametros[i];
    }
  }
  // printf("Paramétros non trouvé gringo ! ");
  return NULL;
}

// Recherche par nom en flash (F("...")) : le nom est copie dans un tampon RAM.
// Les noms font au plus 13 caracteres.
Parameters *GetParam(const __FlashStringHelper *name)
{
  char tampon[16];
  strncpy_P(tampon, (PGM_P)name, sizeof(tampon) - 1);
  tampon[sizeof(tampon) - 1] = '\0';
  return GetParam(tampon);
}
 
// 1=error,0=continue
// nomMin / nomMax : noms des parametres bornes (en flash)
int ValidCapteurData(const __FlashStringHelper *nomMin, const __FlashStringHelper *nomMax, float value)
{
  int minVal = GetParam(nomMin)->Value;
  int maxVal = GetParam(nomMax)->Value;
  if (value < minVal || value > maxVal)
  {
    OnImprimeLerreurViaLed(4);
    return 1;
  }
  return 0;
}
 
struct Record *batch[BATCH_SIZE];
int SavedBatch = 0;
 
// Libere les enregistrements du lot et le vide
void LibererLot()
{
  for (int i = 0; i < SavedBatch; i++)
  {
    free(batch[i]);
  }
  SavedBatch = 0;
}

// Ajoute record au lot ; AppendSdCard en prend possession (libere apres ecriture).
// Le lot est ecrit sur la SD quand il est plein.
// 1=error,0=success
int AppendSdCard(Record *record)
{
  batch[SavedBatch] = record;
  SavedBatch++;
  if (SavedBatch < BATCH_SIZE)
  {
    return 0;
  }

  File fichier;
  if (!GetFile(&batch[SavedBatch - 1]->Date, fichier))
  {
    LibererLot();
    return 1;
  }
  for (int i = 0; i < SavedBatch; i++)
  {
    char buffer[64];
    char *p = buffer;
    p = FormatDecimal(batch[i]->Latitude, 6, p);
    *p++ = ';';
    p = FormatDecimal(batch[i]->Longitude, 6, p);
    *p++ = ';';
    p = FormatDecimal(batch[i]->Temperature, 2, p);
    *p++ = ';';
    p = FormatDecimal(batch[i]->Humidity, 2, p);
    *p++ = ';';
    p = FormatDecimal(batch[i]->Pressure, 2, p);
    *p++ = '\n';
    *p = '\0';
    size_t longueur = p - buffer;
    if (fichier.write((const uint8_t *)buffer, longueur) != longueur)
    {
      OnImprimeLerreurViaLed(6);
    }
  }
  fichier.close();
  LibererLot();
  return 0;
}
 

// Affiche une ligne du tableau. libelle est en flash (F("...")).
void AfficheLigne(const __FlashStringHelper *libelle, const char *valeur)
{
  char ligne[64];
  snprintf_P(ligne, sizeof(ligne), PSTR("| %-12S : %-19s |"), (PGM_P)libelle, valeur);
  Serial.println(ligne);
}

void PrintRecord(Record *record)
{
  Serial.println();
  // Serial.println(F("RELEVE DES CAPTEURS"));
 
  if (record == NULL)
  {
    // Serial.println(F("Aucun enregistrement disponible"));
    Serial.println();
    return;
  }
 
  char date[20];
  snprintf_P(date, sizeof(date), PSTR("%02d/%02d/%04d %02d:%02d:%02d"),
           record->Date.day(), record->Date.month(), record->Date.year(),
           record->Date.hour(), record->Date.minute(), record->Date.second());
 
  char latitude[16];
  char longitude[16];
  char temperature[16];
  char humidite[16];
  char pression[16];
 
  FormatDecimal(record->Latitude, 5, latitude);
  FormatDecimal(record->Longitude, 5, longitude);
 
  // Ajout manuel de l'unite (evite strcat_P pour economiser la flash).
  // FormatDecimal renvoie la fin de chaine, on s'en sert directement.
  char *fin;
  fin = FormatDecimal(record->Temperature, 2, temperature);
  *fin++ = ' ';
  *fin++ = 'C';
  *fin = '\0';

  fin = FormatDecimal(record->Humidity, 2, humidite);
  *fin++ = ' ';
  *fin++ = '%';
  *fin = '\0';

  fin = FormatDecimal(record->Pressure, 2, pression);
  *fin++ = ' ';
  *fin++ = 'h';
  *fin++ = 'P';
  *fin++ = 'a';
  *fin = '\0';

  AfficheLigne(F("Date"), date);
  AfficheLigne(F("Latitude"), latitude);
  AfficheLigne(F("Longitude"), longitude);
  AfficheLigne(F("Temperature"), temperature);
  AfficheLigne(F("Humidite"), humidite);
  AfficheLigne(F("Pression"), pression);
 
  Serial.println(F("+--------------------------------------+"));
  Serial.println();
}
 
void ModeMaintenance()
{
  while (1)
  {
    if (BoutonRougeAppuiLong())
    {
      Serial.println(F("Retour en mode standard"));
      modeActuel = MODE_STANDARD;
      return;
    }

    Record *record = GetNewRecord(true);
    PrintRecord(record);
    free(record);
    for (int i = 0; i < 100; i++)
    {
      PompeGps();
      delay(10);
    }
  }
}

void ModeNormalOrEco(bool ecoMode)
{
  if(ecoMode){
    Serial.println(F("ModeEco"));
  }else{
    Serial.println(F("ModeNormal"));
  }
  int shouldDoGPS = true;
  unsigned long intervalle = (unsigned long)GetParam(F("LOG_INTERVAL"))->Value * 1000 * (ecoMode ? 2 : 1);
 
  while (1)
  {
    // AppendSdCard prend possession du record : ne pas le liberer ici
    Record *record_ptr = GetNewRecord(shouldDoGPS);
    if (record_ptr != NULL && AppendSdCard(record_ptr) > 0)
    {
      modeActuel = MODE_STANDARD;
      return;
    }

    unsigned long debut = millis();
    while (millis() - debut < intervalle)
    {
      if (BoutonVertAppuiLong() && !ecoMode)
      {
        Serial.println(F("Passage en mode economique"));
        modeActuel = MODE_ECO;
        return;
      }

      if (BoutonRougeAppuiLong())
      {
        if (ecoMode)
        {
          modeActuel = MODE_STANDARD;
        }
        else
        {
          Serial.println(F("Passage en mode maintenance"));
          modeActuel = MODE_MAINTENANCE;
        }
        return;
      }

      PompeGps();
      delay(10);
    }

    if (ecoMode == 1)
    {
      shouldDoGPS = !shouldDoGPS;
    }
  }
}
void ModeConfig()
{
  // Serial.println("=== Mode configuration ===");
  // Serial.print("Imprimer la commande :");
  char commande[40];
  int currentCommandeIndex = 0;
  char valeur[40];
  int currentValeurIndex = 0;
  int haveEqual = false;
  while (Serial.available())
  {
    char caractere = Serial.read();
 
    if (caractere == '\r' || caractere == '\n')
    {
      commande[currentCommandeIndex] = '\0';
      valeur[currentValeurIndex] = '\0';
      (ProcessCommande(commande, valeur));
      currentCommandeIndex = 0;
      currentValeurIndex = 0;
      haveEqual = false;
      continue;
    }
    
    
    if (caractere == '=')
    {
      haveEqual = true;
    }
    
    
    else
    {
      // Les buffers font 40 o (39 caracteres + '\0') : on ignore le surplus
      if (haveEqual)
      {
        if (currentValeurIndex < 39)
        {
          valeur[currentValeurIndex] = caractere;
          currentValeurIndex++;
        }
      }
      else
      {
        if (currentCommandeIndex < 39)
        {
          commande[currentCommandeIndex] = caractere;
          currentCommandeIndex++;
        }
      }
    }
  }
}
 
// Affiche le code d'erreur sur la liaison serie (LED pas encore cablee)
void OnImprimeLerreurViaLed(int mode)
{
  Serial.print(F("Erreur "));
  Serial.print(mode);
  switch (mode)
  {
  case 1:
    Serial.println(F(" : horloge RTC"));
    break;
  case 2:
    Serial.println(F(" : GPS sans fix"));
    break;
  case 3:
    Serial.println(F(" : capteur BME280"));
    break;
  case 4:
    Serial.println(F(" : donnee capteur incoherente"));
    break;
  case 5:
    Serial.println(F(" : acces carte SD"));
    break;
  case 6:
    Serial.println(F(" : carte SD pleine"));
    break;
  default:
    Serial.println();
    break;
  }
}
 
 
void loop()
{
  switch (modeActuel)
  {
  case MODE_CONFIG:
    ModeConfig();
    modeActuel = MODE_STANDARD;
    break;

  case MODE_STANDARD:
    ModeNormalOrEco(false);
    break;

  case MODE_ECO:
    ModeNormalOrEco(true);
    break;

  case MODE_MAINTENANCE:
    ModeMaintenance();
    break;
  }
}
 
