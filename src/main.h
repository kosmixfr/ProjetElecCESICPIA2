#pragma once

#include <avr/pgmspace.h>
#include <RTClib.h>

void setup();

void loop();

void InitLed();
void InitGps();
void PompeGps();
void InitEeprom();
void InitRtc();
void InitSd();
void InitCapteur();
void InitBoutons();
bool BoutonRougePresse();
bool BoutonVertPresse();
bool BoutonRougeAppui();
bool BoutonVertAppui();
bool BoutonRougeAppuiLong();
bool BoutonVertAppuiLong();
void SaveEepromParameters();

struct Record
{
  double Latitude;
  double Longitude;
  DateTime Date;
  float Temperature;
  float Humidity;
  float Pressure;
};

struct Parameters
{
  const char *Name; // chaine en PROGMEM : a lire avec strcmp_P
  long Value;
};

// Bornes min/max (meme ordre que losParametros) : en PROGMEM, pas en RAM.
// Economise 112 o de RAM : la pile deborde sinon (cf. diagnostic).
struct Bornes
{
  long Min;
  long Max;
};

// 2=erreruaccesauxdonnesdugps
// 1=erreurd'accesàl'horlogertc
// 3=erreuraccesauxdonnéesd'uncapteur
// 4=donnesincohérented'uncapteur
// 5=Erreur d’accès ou d’écriture sur la carte SD
// 6=Carte SD pleine
void OnImprimeLerreurViaLed(int);

void ModeNormal(bool);
void ModeNormalOrEco(bool ecoMode);
void ModeConfig();
void ModeMaintenance();
Parameters *GetParam(const char *);
Parameters *GetParam(const __FlashStringHelper *);
int ValidCapteurData(const __FlashStringHelper *nomMin, const __FlashStringHelper *nomMax, float value);
char *FormatDecimal(float value, uint8_t decimales, char *out);
void AfficheLigne(const __FlashStringHelper *libelle, const char *valeur);

// Noms des parametres : stockes en flash (PROGMEM) pour ne pas occuper la RAM
static const char NOM_LOG_INTERVAL[] PROGMEM = "LOG_INTERVAL";
static const char NOM_TIMEOUT[] PROGMEM = "TIMEOUT";
static const char NOM_FILE_MAX_SIZE[] PROGMEM = "FILE_MAX_SIZE";
static const char NOM_LUMIN[] PROGMEM = "LUMIN";
static const char NOM_LUMIN_LOW[] PROGMEM = "LUMIN_LOW";
static const char NOM_LUMIN_HIGH[] PROGMEM = "LUMIN_HIGH";
static const char NOM_TEMP_AIR[] PROGMEM = "TEMP_AIR";
static const char NOM_MIN_TEMP_AIR[] PROGMEM = "MIN_TEMP_AIR";
static const char NOM_MAX_TEMP_AIR[] PROGMEM = "MAX_TEMP_AIR";
static const char NOM_HYGR[] PROGMEM = "HYGR";
static const char NOM_HYGR_MINT[] PROGMEM = "HYGR_MINT";
static const char NOM_HYGR_MAXT[] PROGMEM = "HYGR_MAXT";
static const char NOM_PRESSURE_MIN[] PROGMEM = "PRESSURE_MIN";
static const char NOM_PRESSURE_MAX[] PROGMEM = "PRESSURE_MAX";

Parameters losParametros[14] = {
    {NOM_LOG_INTERVAL, 10},
    {NOM_TIMEOUT, 30},
    {NOM_FILE_MAX_SIZE, 2048},
    {NOM_LUMIN, 1},
    {NOM_LUMIN_LOW, 255},
    {NOM_LUMIN_HIGH, 768},
    {NOM_TEMP_AIR, 1},
    {NOM_MIN_TEMP_AIR, -10},
    {NOM_MAX_TEMP_AIR, 60},
    {NOM_HYGR, 1},
    {NOM_HYGR_MINT, 0},
    {NOM_HYGR_MAXT, 50},
    {NOM_PRESSURE_MIN, 800},
    {NOM_PRESSURE_MAX, 1100}};

static const Bornes LesBornes[14] PROGMEM = {
    {1, 1440},   // LOG_INTERVAL
    {1, 255},    // TIMEOUT
    {1, 65535},  // FILE_MAX_SIZE
    {0, 1},      // LUMIN
    {0, 1023},   // LUMIN_LOW
    {0, 1023},   // LUMIN_HIGH
    {0, 1},      // TEMP_AIR
    {-40, 85},   // MIN_TEMP_AIR
    {-40, 85},   // MAX_TEMP_AIR
    {0, 1},      // HYGR
    {-40, 85},   // HYGR_MINT
    {-40, 85},   // HYGR_MAXT
    {300, 1100}, // PRESSURE_MIN
    {300, 1100}  // PRESSURE_MAX
};
