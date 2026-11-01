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
#include <EEPROM.h>
#include <stdlib.h> // ltoa (FormatDecimal)
#include <string.h> // strcmp_P / strncpy_P
#include "main.h"
 
#define SIX_SEVEN_SIX 6
#define SIX_SEVEN_SEVEN 7
#define CLK_LED 8
#define DATA_LED 9
#define LED_COUNT 1
#define VERSION 2 // 2 : plan EEPROM a 15 parametres (ajout de PRESSURE)
#define NUMERO_LOT 1
#define CAPTEUR_TIMEOUT 25000
#define BOUTON_ROUGE 2
#define BOUTON_VERT 3
#define RTC_RETRY 5
#define CAPTEUR_RETRY 5
#define EEPROM_MAGIC 0xACAB
#define EEPROM_VERSION 2
#define EEPROM_LOT 3
#define EEPROM_PARAMS 4
#define ANTI_REBOND_MS 30
#define DUREE_APPUI_LONG 5000
#define CONFIG_INACTIVITE_MS 1800000UL // 30 min sans saisie -> retour en standard
 
TinyGPSPlus gps;
// latitude gps / longitude gps
// Pile, size :2
float LesKordonnesGps[2];
// CS de la carte SD : D4 (cablage materiel MOSI=D11 MISO=D12 SCK=D13).
// La LED a ete deplacee sur D8/D9 pour liberer D4 (et D3 = bouton vert !).
const int SD_CS = 4;
// library for the BME280 sensor
Adafruit_BME280 capteur;
// Grove RTC DS1307 (adresse I2C 0x68, alimenter en 5V !)
// Absent si compilation avec -DSANS_RTC (env uno_sans_rtc).
#ifndef SANS_RTC
RTC_DS1307 rtc_clock;
#endif
 
int SumCapteurError = 0;
// Maintenance en interieur : autorise les mesures sans position GPS
// (position affichee 0.00000). Les modes normal/eco exigent toujours le GPS.
bool sansPositionOk = false;
// Dernier code d'echec de GetNewRecord (0 = ok) : la maintenance l'affiche
// pour dire QUEL sous-systeme bloque (1=RTC, 2=GPS, 3=BME280, 4=donnee).
// Le Record est statique, jamais alloue : le tas (~300 o libres) est trop
// petit pour malloc (SD + pile) et malloc refusait (ex-Echec 7).
uint8_t codeEchec = 0;
// SD prete ou non : InitSd le pose, GetFile le repose a false sur echec
// et retente SD.begin (carte reinseree). Evite les SD.open repetes sur
// un bus mort (pics de courant -> brown-out + "Lancement" en boucle).
bool carteSdOk = false;

enum ModeFonctionnement
{
  MODE_CONFIG,
  MODE_STANDARD,
  MODE_ECO,
  MODE_MAINTENANCE
};

ModeFonctionnement modeActuel = MODE_STANDARD;
// Instant (millis) de la derniere saisie en configuration : sans saisie
// pendant CONFIG_INACTIVITE_MS, retour automatique en mode standard.
unsigned long derniereActiviteConfig = 0;
 
 
ChainableLED leds(CLK_LED, DATA_LED, LED_COUNT);

// --- Signalisation LED (voir spec) : couleur continue par mode, clignotant 1 Hz par erreur ---
#define COUL_NOIR 0
#define COUL_VERT 1
#define COUL_JAUNE 2
#define COUL_BLEU 3
#define COUL_ORANGE 4
#define COUL_ROUGE 5
#define COUL_BLANC 6

static const uint8_t COULEURS_LED[][3] PROGMEM = {
    {0, 0, 0},       // noir : eteint
    {0, 255, 0},     // vert : mode standard
    {255, 255, 0},   // jaune : mode configuration
    {0, 0, 255},     // bleu : mode economique
    {255, 165, 0},   // orange : mode maintenance
    {255, 0, 0},     // rouge : erreurs
    {255, 255, 255}, // blanc : erreurs carte SD
};

// noinline+noclone : garde un seul exemplaire generique de setColorRGB
// (une specialisation par constantes couterait ~250 o de flash chacune).
__attribute__((noinline, noclone)) static void AppliqueCouleur(uint8_t idx)
{
  leds.setColorRGB(0,
                   pgm_read_byte(&COULEURS_LED[idx][0]),
                   pgm_read_byte(&COULEURS_LED[idx][1]),
                   pgm_read_byte(&COULEURS_LED[idx][2]));
}

// Couleur continue de chaque mode (index = valeur de ModeFonctionnement).
static const uint8_t COULEUR_PAR_MODE[4] PROGMEM = {
    COUL_JAUNE,  // MODE_CONFIG
    COUL_VERT,   // MODE_STANDARD
    COUL_BLEU,   // MODE_ECO
    COUL_ORANGE, // MODE_MAINTENANCE
};

__attribute__((noinline, noclone)) static void AfficheCouleurMode()
{
  AppliqueCouleur(pgm_read_byte(&COULEUR_PAR_MODE[(uint8_t)modeActuel]));
}

// Alterne 2 couleurs (durees en ms, 1 Hz => 500/500 ; 2x plus long => 333/667)
// 4 cycles puis retour a la couleur du mode en cours.
__attribute__((noinline, noclone)) static void ClignoteLed(uint8_t c1, uint8_t c2, int d1_ms, int d2_ms)
{
  for (uint8_t i = 0; i < 4; i++)
  {
    AppliqueCouleur(c1);
    delay(d1_ms);
    AppliqueCouleur(c2);
    delay(d2_ms);
  }
  AfficheCouleurMode();
}
 
SoftwareSerial gpsSerial(SIX_SEVEN_SIX, SIX_SEVEN_SEVEN); // RX, TX

// Bornes lues en PROGMEM (voir LesBornes dans main.h)
static long BorneMin(int i)
{
  return (long)pgm_read_dword(&LesBornes[i].Min);
}

static long BorneMax(int i)
{
  return (long)pgm_read_dword(&LesBornes[i].Max);
}

void SaveEepromParameters()
{
  int adresse = EEPROM_PARAMS;
  for (uint8_t i = 0; i < NB_PARAMS; i++)
  {
    EEPROM.put(adresse, losParametros[i].Value);
    adresse += sizeof(long);
  }
}

void InitLed()
{
  AppliqueCouleur(COUL_NOIR);
}

void InitEeprom()
{
  uint16_t magic = 0;
  uint8_t version = 0;
  EEPROM.get(0, magic);
  EEPROM.get(EEPROM_VERSION, version);

  // Premier boot, ou EEPROM ecrite par une version au plan different :
  // on repart des valeurs par defaut (sinon les 15 longs seraient mal relus).
  if (magic != EEPROM_MAGIC || version != (uint8_t)VERSION)
  {
    EEPROM.put(0, (uint16_t)EEPROM_MAGIC);
    EEPROM.put(EEPROM_VERSION, (uint8_t)VERSION);
    EEPROM.put(EEPROM_LOT, (uint8_t)NUMERO_LOT);
    SaveEepromParameters();
    return;
  }

  EEPROM.put(EEPROM_VERSION, (uint8_t)VERSION);
  EEPROM.put(EEPROM_LOT, (uint8_t)NUMERO_LOT);

  int adresse = EEPROM_PARAMS;
  for (uint8_t i = 0; i < NB_PARAMS; i++)
  {
    long valeur = losParametros[i].Value;
    EEPROM.get(adresse, valeur);
    if (valeur >= BorneMin(i) && valeur <= BorneMax(i))
    {
      losParametros[i].Value = valeur;
    }
    adresse += sizeof(long);
  }
}

void InitRtc()
{
#ifdef SANS_RTC
  Serial.println(F("Sans RTC"));
  return;
#else
  for (int essai = 0; essai < RTC_RETRY; essai++)
  {
    Wire.clearWireTimeoutFlag();
    if (rtc_clock.begin())
    {
      Wire.clearWireTimeoutFlag();
      return;
    }
    Wire.clearWireTimeoutFlag();
    delay(100);
  }
  OnImprimeLerreurViaLed(1);
#endif
}

#ifdef SANS_RTC
// Horloge fictive quand la RTC est absente : 01/01/2026 00:00:00 UTC
// + secondes ecoulees depuis le boot. Monotone donc triable et compatible
// avec la rotation des fichiers journaliers ; repart de zero a chaque reset.
// (Une date vraiment aleatoire casserait l'ordre des mesures et des fichiers.)
#define FAUSSE_DATE_BASE 1767225600UL // 01/01/2026 00:00:00 UTC
DateTime GenererDateSansRtc()
{
  return DateTime(FAUSSE_DATE_BASE + millis() / 1000UL);
}
#endif

void InitSd()
{
  // En mode maitre SPI, SS (D10 sur Uno) doit rester en sortie
  // meme quand le CS de la SD est ailleurs (ici D4).
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  // CS explicite a HIGH avant begin : libere MISO, puis delai
  // de demarrage carte (certaines cartes exigent ~200-300 ms
  // apres mise sous tension, sinon SD.begin echoue -> Erreur 5).
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  delay(300);
  // 3 essais : la 1re init echoue souvent a froid (alim/carte lente).
  for (uint8_t i = 0; i < 3; i++)
  {
    if (SD.begin(SD_CS))
    {
      carteSdOk = true;
      return;
    }
    SD.end();
    delay(500);
  }
  OnImprimeLerreurViaLed(5);
  // Aide au cablage (1 ligne flash) : cause n°1 = CS, format, contact.
  Serial.println(F("Verif CS=4 FAT32 carte enfoncee"));
}

void InitCapteur()
{
  for (uint8_t essai = 0; essai < CAPTEUR_RETRY; essai++)
  {
    Wire.clearWireTimeoutFlag();
    if (capteur.begin(0x76) || capteur.begin(0x77))
    {
      Wire.clearWireTimeoutFlag();
      return;
    }
    Wire.clearWireTimeoutFlag();
    delay(100);
  }
  // Message detaille inutile : OnImprimeLerreurViaLed(3) signale deja
  // "Erreur 3 : BME280" + LED (flash a ~100 %, chaque octet compte).
  OnImprimeLerreurViaLed(3);
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
  volatile bool evenement; // pose par l'ISR externe, traite par ActualiserBouton
};

// Seul 'evenement' est partage avec les ISR : pas de volatile global,
// pas de section critique pour les autres champs (contexte boucle uniquement).
EtatBouton boutonRouge;
EtatBouton boutonVert;

// ISR minimales : un simple drapeau. Tout le traitement (anti-rebond,
// appui, appui long) est fait en contexte boucle par ActualiserBouton,
// donc sans contrainte de temps d'ISR et sans millis() en interruption.
static void InterruptionBoutonRouge()
{
  boutonRouge.evenement = true;
}

static void InterruptionBoutonVert()
{
  boutonVert.evenement = true;
}

// Traite les fronts signales par attachInterrupt + scrute l'appui long
// (aucun front ne se produit pendant un maintien : d'ou la scrutation).
// noinline : une seule copie partagee (flash presque pleine).
// Appelee par chaque accesseur Bouton* et par PompeGps (qui tourne dans
// toutes les attentes : pas d'angle mort, meme pendant InitGps).
__attribute__((noinline)) static void ActualiserBouton(EtatBouton &bouton)
{
  // Prise atomique du drapeau : un front ISR pendant la lecture est
  // soit vu (avant), soit conserve pour le prochain appel (apres).
  noInterrupts();
  bool evenement = bouton.evenement;
  bouton.evenement = false;
  interrupts();

  unsigned long maintenant = millis();

  if (evenement && (maintenant - bouton.derniereModif) >= ANTI_REBOND_MS)
  {
    bool niveau = digitalRead(bouton.pin) == LOW;
    if (niveau != bouton.enfonce)
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
  }

  // longSignale evite de re-armer appuiLong tant que le bouton reste enfonce
  if (bouton.enfonce && !bouton.longSignale && (maintenant - bouton.debut) >= DUREE_APPUI_LONG)
  {
    bouton.appuiLong = true;
    bouton.longSignale = true;
  }
}

static bool PrendreEvenement(bool &drapeau)
{
  // Pas de section critique : en contexte boucle uniquement, l'ISR
  // ne touche plus qu'a 'evenement' (bool, acces atomique sur AVR).
  bool valeur = drapeau;
  drapeau = false;
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
  // appuiLong/longSignale/evenement : deja a false via l'init .bss.

  // Interruptions externes sur fronts montants ET descendants (CHANGE) :
  // D2 = INT0, D3 = INT1 sur Uno. L'ISR ne pose qu'un drapeau ;
  // anti-rebond + appui long dans ActualiserBouton (boucle).
  attachInterrupt(digitalPinToInterrupt(BOUTON_ROUGE), InterruptionBoutonRouge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BOUTON_VERT), InterruptionBoutonVert, CHANGE);
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
  ActualiserBouton(boutonRouge);
  return PrendreEvenement(boutonRouge.appui);
}

bool BoutonVertAppui()
{
  ActualiserBouton(boutonVert);
  return PrendreEvenement(boutonVert.appui);
}

bool BoutonRougeAppuiLong()
{
  ActualiserBouton(boutonRouge);
  return PrendreEvenement(boutonRouge.appuiLong);
}

bool BoutonVertAppuiLong()
{
  ActualiserBouton(boutonVert);
  return PrendreEvenement(boutonVert.appuiLong);
}

void setup()
{
  Serial.begin(9600);
  Serial.println(F("Lancement"));

  // I2C en premier : le timeout court + SoftwareSerial actif perturbaient
  // les init RTC/BME (timeout 1 ms, flag jamais efface).
  Wire.begin();
  // 25 ms + reset auto du bus sur timeout (recommandation Wire AVR).
  Wire.setWireTimeout(CAPTEUR_TIMEOUT, true);
  Wire.clearWireTimeoutFlag();

  InitBoutons();
  // Memorise la demande de config des maintenant : les inits suivantes
  // (RTC, capteur, SD, GPS jusqu'a 120 s) sont bloquantes et l'utilisateur
  // peut relacher le bouton avant la fin du setup.
  bool demandeConfig = BoutonRougePresse() || BoutonRougeAppui();
  InitLed();
  // Pas de scan I2C ici : 242 o de flash economises (projet a ~99.9 %).
  // Un probleme I2C remonte de toute facon via les erreurs 1 (RTC) et 3 (BME280).
  InitRtc();
  InitCapteur();
  InitSd();
  InitEeprom();

  // GPS en dernier : SoftwareSerial bavard fausse les timings I2C,
  // et l'ancien InitGps bloquait tout sans fix.
  InitGps();

  modeActuel = (demandeConfig || BoutonRougePresse() || BoutonRougeAppui()) ? MODE_CONFIG : MODE_STANDARD;
  if (modeActuel == MODE_CONFIG)
  {
    derniereActiviteConfig = millis();
    Serial.println(F("=CONFIG="));
    Serial.print(F("> "));
    // L'entree en config exigeait ROUGE tenu au reset, mais les inits
    // suivantes bloquent (GPS jusqu'a 120 s sans fix) : a l'arrivee ici
    // un bouton encore tenu depasse deja les 5 s et provoquait une sortie
    // immediate vers STANDARD. On neutralise l'appui long en cours
    // (longSignale arme : ignore jusqu'au prochain front) : la sortie
    // exigera un NOUVEL appui long de 5 s. Idem pour VERT (ECO intempestif).
    // Ecritures 1 octet donc atomiques sur AVR : pas de section critique.
    // (Les drapeaux d'appui court ne sont plus lus apres le setup.)
    boutonRouge.appuiLong = false;
    boutonRouge.longSignale = true;
    boutonVert.appuiLong = false;
    boutonVert.longSignale = true;
  }
}

// Ecrit v avec 'decimales' decimales dans out, sans printf (flash pleine :
// sprintf_P tirerait vfprintf ~950 o). Partie entiere via ltoa.
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
  char tmp[12];
  ltoa(valeur / facteur, tmp, 10);
  for (char *q = tmp; *q; q++)
  {
    *p++ = *q;
  }
  *p++ = '.';
  long fraction = valeur % facteur;
  for (long diviseur = facteur / 10; diviseur > 0; diviseur /= 10)
  {
    *p++ = '0' + (fraction / diviseur) % 10;
  }
  *p = '\0';
  return p;
}

// Ecrit deux chiffres (00-99) et renvoie la fin de la chaine.
static char *DeuxChiffres(char *p, uint8_t valeur)
{
  *p++ = '0' + valeur / 10;
  *p++ = '0' + valeur % 10;
  return p;
}

// Chiffre base 36 (0-9 puis A-Z) : 2 caracteres = 1296 valeurs.
static char ChiffreBase36(uint8_t valeur)
{
  return valeur < 10 ? '0' + valeur : 'A' + valeur - 10;
}

// Comme FormatDecimal, mais ecrit "NA" si la valeur est invalide (isnan) :
// c'est ce qu'exige le cahier des charges pour un capteur muet ou incoherent.
static char *FormateChamp(float valeur, uint8_t decimales, char *out)
{
  if (isnan(valeur))
  {
    out[0] = 'N';
    out[1] = 'A';
    out[2] = '\0';
    return out + 2;
  }
  return FormatDecimal(valeur, decimales, out);
}

// Ouvre le fichier AAMMJJrr.LOG du jour (rr = revision base 36, 1296 valeurs)
// dont la taille est sous FILE_MAX_SIZE.
// Nom au format 8.3 : la bibliotheque SD ne gere pas les noms longs.
// 3 chiffres de revision + la date complete depasseraient les 8 caracteres
// autorises, d'ou la base 36 sur 2 caracteres (00..ZZ = 1296 fichiers/jour).
// false=erreur
bool GetFile(DateTime *date, File &fichier)
{
  // Carte tombee ou reinseree apres le boot : une seule retentative
  // (pas de boucle serree -> pics de courant -> reset "Lancement").
  if (!carteSdOk)
  {
    digitalWrite(SD_CS, HIGH);
    if (!SD.begin(SD_CS))
    {
      OnImprimeLerreurViaLed(5);
      return false;
    }
    carteSdOk = true;
  }
  long max_size = GetParam(F("FILE_MAX_SIZE"))->Value;
  // Nom 8.3 manuel AAMMJJrr.LOG (sans sprintf_P -> vfprintf ~950 o economises).
  char nomFichier[13];
  char *p = DeuxChiffres(nomFichier, date->year() % 100);
  p = DeuxChiffres(p, date->month());
  p = DeuxChiffres(p, date->day());
  *p++ = '0'; // numero de revision, remplace dans la boucle
  *p++ = '0';
  *p++ = '.';
  *p++ = 'L';
  *p++ = 'O';
  *p++ = 'G';
  *p = '\0';
  for (uint16_t i = 0; i < 1296; i++)
  {
    nomFichier[6] = ChiffreBase36(i / 36); // revision 00..ZZ
    nomFichier[7] = ChiffreBase36(i % 36);
    fichier = SD.open(nomFichier, FILE_WRITE);
    if (!fichier)
    {
      // Ne pas close() un File invalide. Marque le bus mort pour
      // retenter SD.begin au prochain appel (reinsertion possible).
      carteSdOk = false;
      SD.end();
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
// En profite pour actualiser les boutons : tourne dans toutes les attentes
// (InitGps, modes normal/eco/maintenance), donc pas d'angle mort malgre les phases bloquantes.
void PompeGps()
{
  ActualiserBouton(boutonRouge);
  ActualiserBouton(boutonVert);
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
  // Timeout anti-blocage : sans fix au bout de 120 s on continue,
  // GetNewRecord() signalera l'erreur 2 quand il faudra un fix frais.
  const unsigned long TIMEOUT_GPS_MS = 120000UL;
  unsigned long debut = millis();
#ifdef SANS_RTC
  unsigned long dernierPoint = 0;
#endif
  while (!gps.location.isValid())
  {
    PompeGps();
#ifdef SANS_RTC
    if (millis() - dernierPoint >= 5000UL)
    {
      dernierPoint = millis();
      Serial.write('.');
    }
#endif
    if (millis() - debut >= TIMEOUT_GPS_MS)
    {
      OnImprimeLerreurViaLed(2);
      return;
    }
  }
  LesKordonnesGps[0] = gps.location.lat();
  LesKordonnesGps[1] = gps.location.lng();
}

Record *GetNewRecord(int withGps)
{
  codeEchec = 0;
  // Position : fix GPS frais si demandé, sinon dernière position connue.
  // En maintenance interieure (sansPositionOk), on mesure meme sans fix.
  if (withGps)
  {
    // Fenetre d'ecoute ~1,5 s : une phrase NMEA fait ~80 o (~80 ms a 9600 baud).
    // Un seul PompeGps() instantane ne drainait que le contenu actuel du buffer
    // 64 o et ratait quasi a coup sur la trame complete -> Erreur 2 abusive.
    // PompeGps() actualise aussi les boutons : pas d'angle mort pendant l'ecoute.
    // Sortie anticipee des que le fix est valide.
    for (uint8_t i = 0; i < 150 && !gps.location.isValid(); i++)
    {
      PompeGps();
      delay(10);
    }
    if (gps.location.isValid())
    {
      LesKordonnesGps[0] = gps.location.lat();
      LesKordonnesGps[1] = gps.location.lng();
    }
    else if (!sansPositionOk)
    {
      OnImprimeLerreurViaLed(2);
      return NULL;
    }
  }
  // utilisation impossible au 0,0 du globe (aucune position connue)
  if (!sansPositionOk && LesKordonnesGps[0] == 0 && LesKordonnesGps[1] == 0)
  {
    OnImprimeLerreurViaLed(2);
    return NULL;
  }

  // Capteur BME280 : pression convertie de Pa en hPa
  // TIMEOUT est en secondes (cf. losParametros), duree est en ms.
  unsigned long timeout = (unsigned long)GetParam(F("TIMEOUT"))->Value * 1000UL;
  unsigned long debut = millis();
  Wire.clearWireTimeoutFlag();
  float temperature = capteur.readTemperature();
  float pression = capteur.readPressure() / 100.0f;
  float humidity = capteur.readHumidity();
  bool timeoutI2C = Wire.getWireTimeoutFlag();
  Wire.clearWireTimeoutFlag();
  unsigned long duree = millis() - debut;

  if (isnan(temperature) || isnan(pression) || isnan(humidity) || timeoutI2C || duree > timeout)
  {
    SumCapteurError++;
    codeEchec = 3;
    if (SumCapteurError >= 2)
    {
      OnImprimeLerreurViaLed(3);
    }
    // Une mesure ratee n'annule plus l'enregistrement : le cahier des charges
    // demande d'ecrire "NA" pour la donnee du capteur qui n'a pas repondu.
    temperature = NAN;
    pression = NAN;
    humidity = NAN;
  }
  else
  {
    SumCapteurError = 0;
    // Valeur hors bornes : signalee (erreur 4) puis remplacee par "NA".
    if (ValidCapteurData(F("MIN_TEMP_AIR"), F("MAX_TEMP_AIR"), temperature))
      temperature = NAN;
    if (ValidCapteurData(F("HYGR_MINT"), F("HYGR_MAXT"), humidity))
      humidity = NAN;
    if (ValidCapteurData(F("PRESSURE_MIN"), F("PRESSURE_MAX"), pression))
      pression = NAN;
    // Capteur desactive par configuration (TEMP_AIR / HYGR / PRESSURE = 0) : "NA".
    if (GetParam(F("TEMP_AIR"))->Value == 0)
      temperature = NAN;
    if (GetParam(F("HYGR"))->Value == 0)
      humidity = NAN;
    if (GetParam(F("PRESSURE"))->Value == 0)
      pression = NAN;
  }

  DateTime maintenant;
#ifdef SANS_RTC
  maintenant = GenererDateSansRtc();
#else
  {
    Wire.clearWireTimeoutFlag();
    maintenant = rtc_clock.now();
    bool rtcTimeout = Wire.getWireTimeoutFlag();
    Wire.clearWireTimeoutFlag();
    if (rtcTimeout)
    {
      OnImprimeLerreurViaLed(1);
      return NULL;
    }
  }
#endif

  // Record statique : un seul enregistrement vivant a la fois
  // (ecrit sur SD avant la mesure suivante). malloc echouait
  // systematiquement : tas plein (85 % de RAM statique + pile).
  static Record leRecord;
  leRecord.Latitude = LesKordonnesGps[0];
  leRecord.Longitude = LesKordonnesGps[1];
  leRecord.Date = maintenant;
  leRecord.Temperature = temperature;
  leRecord.Humidity = humidity;
  leRecord.Pressure = pression;

  return &leRecord;
}
 

// Commandes du cahier des charges qui ne sont pas des parametres numeriques.
// Renvoie un message d'erreur, ou NULL quand la commande est acceptee
// (l'appelant sauvegarde alors l'EEPROM et repond "OK").
// CLOCK / DATE / DAY ne sont pas implementes : la conversion date <-> horodatage
// de RTClib coute ~830 o de flash et il n'en reste que ~420 de libre (cf. doc).
static const __FlashStringHelper *CommandeSpeciale(char commande[])
{
  if (!strcmp_P(commande, PSTR("RESET")))
  {
    for (uint8_t i = 0; i < NB_PARAMS; i++)
    {
      losParametros[i].Value = (long)pgm_read_dword(&LesDefauts[i]);
    }
    return NULL;
  }
  if (!strcmp_P(commande, PSTR("VERSION")))
  {
    // Version et lot tiennent sur un chiffre (codes 1..9).
    Serial.print(F("VERSION "));
    Serial.write('0' + VERSION);
    Serial.print(F(" lot "));
    Serial.write('0' + NUMERO_LOT);
    Serial.println();
    return NULL;
  }
  return F("Commande inconnu");
}

const __FlashStringHelper *ProcessCommande(char commande[], char value[])
{
  Parameters *param = GetParam(commande);
  if (param == NULL)
  {
    // RESET / VERSION (syntaxe differente des parametres numeriques)
    return CommandeSpeciale(commande);
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
  int idx = param - losParametros;
  if (nombre < BorneMin(idx) || nombre > BorneMax(idx))
  {
    return F("Valeur out of bound !!");
  }
  param->Value = nombre;
  return NULL;
}
 
// Recherche par nom saisi en RAM (commande recue par la liaison serie)
Parameters *GetParam(const char *name)
{
  for (uint8_t i = 0; i < NB_PARAMS; i++)
  {
    if (strcmp_P(name, losParametros[i].Name) == 0)
    {
      return &losParametros[i];
    }
  }
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
  // Debug Erreur 4 : affiche la valeur fautive + LA borne violée.
  // Testé en deux temps (MIN d'abord) : un seul pointeur de nom survit
  // aux appels dans chaque chemin (garder les deux coûtait ~94 o).
  int minVal = GetParam(nomMin)->Value;
  if (value < minVal)
  {
    char texte[16];
    FormatDecimal(value, 2, texte);
    AfficheLigne(nomMin, texte);
    OnImprimeLerreurViaLed(4);
    return 1;
  }
  int maxVal = GetParam(nomMax)->Value;
  if (value > maxVal)
  {
    char texte[16];
    FormatDecimal(value, 2, texte);
    AfficheLigne(nomMax, texte);
    OnImprimeLerreurViaLed(4);
    return 1;
  }
  return 0;
}
 
// Ecrit un champ (valeur formatee + separateur) sur la carte SD.
// Le tampon vient de l'appelant : la pile ne grandit pas d'un cadre.
// champ[12] suffit : plus grand champ "-180.123456" = 11 + '\0' = 12.
static bool EcritChamp(File &fichier, char *champ, float valeur, uint8_t decimales, uint8_t separateur)
{
  char *fin = FormateChamp(valeur, decimales, champ);
  size_t taille = (size_t)(fin - champ);
  // Deux ecritures distinctes (pas de &&) : le separateur est ecrit meme
  // si la valeur a echoue, comme dans l'ancien enchainement.
  bool okValeur = (fichier.write((const uint8_t *)champ, taille) == taille);
  bool okSeparateur = (fichier.write(separateur) == 1);
  return okValeur && okSeparateur;
}

// Ecrit "AA-MM-JJ HH:MM:SS;" en tete de chaque ligne de log.
// Meme convention d'annee sur 2 chiffres que le nom de fichier AAMMJJrr.LOG.
static bool EcritDateHeure(File &fichier, DateTime *date)
{
  char tampon[19];
  char *p = DeuxChiffres(tampon, date->year() % 100);
  *p++ = '-';
  p = DeuxChiffres(p, date->month());
  *p++ = '-';
  p = DeuxChiffres(p, date->day());
  *p++ = ' ';
  p = DeuxChiffres(p, date->hour());
  *p++ = ':';
  p = DeuxChiffres(p, date->minute());
  *p++ = ':';
  p = DeuxChiffres(p, date->second());
  size_t taille = (size_t)(p - tampon);
  bool okDate = (fichier.write((const uint8_t *)tampon, taille) == taille);
  bool okSeparateur = (fichier.write(';') == 1);
  return okDate && okSeparateur;
}

// Ecrit record sur la SD (record statique, valide pendant l'ecriture).
// Ecriture immediate : un lot ne tient pas en RAM (~100 o libres).
// 1=error,0=success
int AppendSdCard(Record *record)
{
  File fichier;
  if (!GetFile(&record->Date, fichier))
  {
    return 1;
  }
  // Ecriture morceau par morceau avec un seul champ[12] : buffer[48/64]
  // + File + SD depassait la pile (~270 o libres) -> "Mo?" + reset.
  char champ[12];
  bool ok = EcritDateHeure(fichier, &record->Date);
  ok &= EcritChamp(fichier, champ, record->Latitude, 6, ';');
  ok &= EcritChamp(fichier, champ, record->Longitude, 6, ';');
  ok &= EcritChamp(fichier, champ, record->Temperature, 2, ';');
  ok &= EcritChamp(fichier, champ, record->Humidity, 2, ';');
  ok &= EcritChamp(fichier, champ, record->Pressure, 2, '\n');
  if (!ok)
  {
    OnImprimeLerreurViaLed(6);
  }
  fichier.close();
  return 0;
}
 

// Affiche une ligne du tableau. libelle est en flash (F("...")).
// Ecriture directe sans tampon ligne[40] : avec ~270 o de pile libre,
// ligne[40] + PrintRecord + ValidCapteurData + SD corrompaient la RAM
// ("Mo?" + reset). Format manuel, sans snprintf_P (vfprintf economise).
void AfficheLigne(const __FlashStringHelper *libelle, const char *valeur)
{
  Serial.write('|');
  Serial.write(' ');
  PGM_P f = (PGM_P)libelle;
  uint8_t n = 0;
  char c;
  while (n < 12 && (c = (char)pgm_read_byte(f++)))
  {
    Serial.write(c);
    n++;
  }
  while (n < 12)
  {
    Serial.write(' ');
    n++;
  }
  Serial.write(' ');
  Serial.write(':');
  Serial.write(' ');
  n = 0;
  while (n < 19 && valeur[n])
  {
    Serial.write(valeur[n]);
    n++;
  }
  while (n < 19)
  {
    Serial.write(' ');
    n++;
  }
  Serial.write(' ');
  Serial.write('|');
  Serial.println();
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
 
  // UN SEUL tampon reutilise (date=19, autres<=12) : date[] + val[]
  // + AfficheLigne imbrique depassaient la pile libre (~270 o) et
  // corrompaient la RAM (flot d'octets illisibles "Mo?" sur serie).
  // Date manuelle (sans snprintf_P -> vfprintf economise).
  char val[20];
  {
    char *q = val;
    DateTime d = record->Date;
    uint16_t aa = d.year();
    q = DeuxChiffres(q, d.day());
    *q++ = '/';
    q = DeuxChiffres(q, d.month());
    *q++ = '/';
    q = DeuxChiffres(q, aa / 100);
    q = DeuxChiffres(q, aa % 100);
    *q++ = ' ';
    q = DeuxChiffres(q, d.hour());
    *q++ = ':';
    q = DeuxChiffres(q, d.minute());
    *q++ = ':';
    q = DeuxChiffres(q, d.second());
    *q = '\0';
  }

  AfficheLigne(F("Date"), val);

  FormatDecimal(record->Latitude, 5, val);
  AfficheLigne(F("Latitude"), val);
  FormatDecimal(record->Longitude, 5, val);
  AfficheLigne(F("Longitude"), val);

  // Ajout manuel de l'unite (evite strcat_P pour economiser la flash).
  // FormateChamp renvoie la fin de chaine ("NA" si la valeur est invalide).
  char *fin;
  fin = FormateChamp(record->Temperature, 2, val);
  *fin++ = ' ';
  *fin++ = 'C';
  *fin = '\0';
  AfficheLigne(F("Temperature"), val);

  fin = FormateChamp(record->Humidity, 2, val);
  *fin++ = ' ';
  *fin++ = '%';
  *fin = '\0';
  AfficheLigne(F("Humidite"), val);

  fin = FormateChamp(record->Pressure, 2, val);
  *fin++ = ' ';
  *fin++ = 'h';
  *fin++ = 'P';
  *fin++ = 'a';
  *fin = '\0';
  AfficheLigne(F("Pression"), val);

  Serial.println();
}
 
void ModeMaintenance()
{
  sansPositionOk = true;
  // Carte SD liberee : elle peut etre retiree ou changee sans risque.
  // carteSdOk=false force un SD.begin() avant le prochain enregistrement
  // (GetFile), pour ne jamais utiliser le volume FAT en cache sur une autre carte.
  SD.end();
  carteSdOk = false;
  while (1)
  {
    if (BoutonRougeAppuiLong())
    {
      Serial.println(F("Mode standard"));
      modeActuel = MODE_STANDARD;
      sansPositionOk = false;
      return;
    }

    // NULL seulement pour une horloge (1) ou un GPS (2) indisponible : un capteur
    // muet ou incoherent (3/4) donne maintenant un enregistrement avec "NA".
    // On affiche quand meme un message, sinon silence total entre deux erreurs.
    Record *record = GetNewRecord(true);
    if (record != NULL)
    {
      PrintRecord(record);
    }
    else
    {
      // Precise QUEL sous-systeme bloque (1=RTC, 2=GPS). Chiffre via
      // Serial.write('0'+code) car print(int) tirerait ~200 o de division.
      Serial.print(F("Echec "));
      Serial.write('0' + codeEchec);
      Serial.println();
    }
    for (int i = 0; i < 100; i++)
    {
      PompeGps();
      delay(10);
    }
  }
}

void ModeNormalOrEco(bool ecoMode)
{
  sansPositionOk = false;
  Serial.println(ecoMode ? F("ModeEco") : F("ModeNormal"));
  // Flush avant GPS/BME/SD (gros appels de courant) : sans lui, un
  // brown-out pendant SD.write tronquait le texte en "ModeNo?" sur
  // le moniteur. Ici le texte part complet avant la mesure.
  Serial.flush();
  int shouldDoGPS = true;
  // LOG_INTERVAL est en minutes (cahier des charges) : 10 min par defaut.
  unsigned long intervalle = (unsigned long)GetParam(F("LOG_INTERVAL"))->Value * 60000UL * (ecoMode ? 2 : 1);
 
  while (1)
  {
    // Record statique : rien a liberer ici
    Record *record_ptr = GetNewRecord(shouldDoGPS);
    // SD en echec (5=acces, 6=pleine) : on NE revient PAS aussitot
    // (l'ancien return relancait ModeNormal en boucle serree -> spam
    // "Erreur 5" + pics de courant -> reset "Lancement"). On tombe
    // dans l'attente intervalle ci-dessous puis on reessaye : la carte
    // peut etre reinseree (GetFile retente SD.begin via carteSdOk).
    if (record_ptr != NULL)
    {
      AppendSdCard(record_ptr);
    }

    unsigned long debut = millis();
    while (millis() - debut < intervalle)
    {
      if (BoutonVertAppuiLong() && !ecoMode)
      {
        Serial.println(F("Mode eco"));
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
          Serial.println(F("Mode maintenance"));
          modeActuel = MODE_MAINTENANCE;
        }
        return;
      }

      PompeGps();
      delay(10);
    }

    if (ecoMode)
    {
      shouldDoGPS = !shouldDoGPS;
    }
  }
}
void ModeConfig()
{
  // Buffers STATIC : loop() rappelle ModeConfig() en boucle (non bloquant).
  // A 9600 baud les caracteres arrivent ~1/ms : une ligne arrive en plusieurs
  // appels. En variables locales, tout etait perdu a chaque retour -> Entree
  // semblait ne rien faire. En static, la ligne se reconstitue entre appels.
  // 16 o suffisent : noms <= 13 car. + '\0', valeurs <= 6 chiffres + '\0'.
  static char commande[16];
  static uint8_t currentCommandeIndex = 0;
  static char valeur[16];
  static uint8_t currentValeurIndex = 0;
  static bool haveEqual = false;
  while (Serial.available())
  {
    derniereActiviteConfig = millis(); // activite : repousse le timeout 30 min
    char caractere = Serial.read();

    if (caractere == '\r' || caractere == '\n' || caractere == '\0')
    {
      // Ignore les lignes vides (les terminaux envoient "\r\n" : le 2e
      // caractere arrive sur des buffers deja vides). haveEqual garde les
      // saisies type "=5" pour repondre "Commande inconnu" au lieu du silence.
      if (currentCommandeIndex > 0 || haveEqual)
      {
        commande[currentCommandeIndex] = '\0';
        valeur[currentValeurIndex] = '\0';
        const __FlashStringHelper *reponse = ProcessCommande(commande, valeur);
        if (reponse)
        {
          Serial.println(reponse);
        }
        else
        {
          SaveEepromParameters();
          Serial.println(F("OK"));
        }
        Serial.print(F("> "));
      }
      currentCommandeIndex = 0;
      currentValeurIndex = 0;
      haveEqual = false;
      continue;
    }

    // Caractere de controle (ESC des fleches / Suppr, etc.) : ignore,
    // sans echo, pour ne pas perturber l'affichage du terminal.
    if (caractere < ' ')
    {
      continue;
    }

    // Echo : le moniteur PlatformIO n'affiche pas les caracteres tapes
    // (pas d'echo local), la carte les renvoie donc au fur et a mesure.
    Serial.write(caractere);

    if (caractere == '=')
    {
      haveEqual = true;
    }
    // Les buffers font 16 o (15 caracteres + '\0') : on ignore le surplus
    else if (haveEqual)
    {
      if (currentValeurIndex < 15)
      {
        valeur[currentValeurIndex++] = caractere;
      }
    }
    else if (currentCommandeIndex < 15)
    {
      commande[currentCommandeIndex++] = caractere;
    }
  }
}
 
// Signale le code d'erreur : message serie + LED (cf. spec).
// Clignotants a 1 Hz : durees egales => 500/500, "2x plus long" => 333/667.
// Factorise en un seul Serial.println + un seul ClignoteLed (flash pleine).
void OnImprimeLerreurViaLed(int mode)
{
  codeEchec = (uint8_t)mode;
  Serial.print(F("Erreur "));
  // write plutot que print(int) : ~200 o de division logicielle en moins
  // (Uno a 99,5 % de flash). Codes 1..6 = un seul chiffre.
  Serial.write((char)('0' + mode));
  const __FlashStringHelper *msg = NULL;
  uint8_t c2 = COUL_BLANC;
  int d1 = 500, d2 = 500;
  switch (mode)
  {
  case 1:
    msg = F(" : horloge RTC");
    c2 = COUL_BLEU;
    break;
  case 2:
    msg = F(" : GPS sans fix");
    c2 = COUL_JAUNE;
    break;
  case 3:
    msg = F(" : BME280");
    c2 = COUL_VERT;
    break;
  case 4:
    msg = F(" : incoherent");
    c2 = COUL_VERT;
    d1 = 333;
    d2 = 667;
    break;
  case 5:
    msg = F(" : SD acces");
    d1 = 333;
    d2 = 667;
    break;
  case 6:
    msg = F(" : SD pleine");
    break;
  default:
    break;
  }
  if (msg)
  {
    Serial.println(msg);
    ClignoteLed(COUL_ROUGE, c2, d1, d2);
  }
  else
  {
    Serial.println();
  }
}
 
 
void loop()
{
  // Couleur continue du mode en cours (uniquement lors d'un changement).
  static ModeFonctionnement dernierModeAffiche = MODE_MAINTENANCE;
  if (modeActuel != dernierModeAffiche)
  {
    dernierModeAffiche = modeActuel;
    AfficheCouleurMode();
  }
  switch (modeActuel)
  {
  case MODE_CONFIG:
    ModeConfig();
    // Sortie : appui long rouge (5 s) ou 30 min sans saisie. L'appui long
    // est lu en premier pour toujours consommer son evenement.
    if (BoutonRougeAppuiLong() || (millis() - derniereActiviteConfig >= CONFIG_INACTIVITE_MS))
    {
      modeActuel = MODE_STANDARD;
    }
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
 
