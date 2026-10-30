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
  long MinValue;
  long MaxValue;
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
    {NOM_LOG_INTERVAL, 10, 1, 1440},
    {NOM_TIMEOUT, 30, 1, 255},
    {NOM_FILE_MAX_SIZE, 2048, 1, 65535},
    {NOM_LUMIN, 1, 0, 1},
    {NOM_LUMIN_LOW, 255, 0, 1023},
    {NOM_LUMIN_HIGH, 768, 0, 1023},
    {NOM_TEMP_AIR, 1, 0, 1},
    {NOM_MIN_TEMP_AIR, -10, -40, 85},
    {NOM_MAX_TEMP_AIR, 60, -40, 85},
    {NOM_HYGR, 1, 0, 1},
    {NOM_HYGR_MINT, 0, -40, 85},
    {NOM_HYGR_MAXT, 50, -40, 85},
    {NOM_PRESSURE_MIN, 800, 300, 1100},
    {NOM_PRESSURE_MAX, 1100, 300, 1100}};
