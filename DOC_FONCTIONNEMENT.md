# ACAB — Documentation de fonctionnement

Station météo embarquée sur **Arduino Uno** (PlatformIO).
Mesure température / humidité / pression (BME280), horodate (RTC DS1307),
localise (GPS), stocke sur carte SD, signale par LED RGB et 2 boutons,
configurable par liaison série.

Fichiers : `src/main.cpp` (1033 lignes), `src/main.h` (paramètres + `Record`),
`platformio.ini` (2 environnements).

---

## 1. Matériel et câblage

| Élément | Branchement | Lib / remarque |
|---|---|---|
| BME280 | I2C `SDA/SCL`, adresses `0x76` puis `0x77` | `Adafruit_BME280` |
| RTC DS1307 (Grove) | I2C `0x68`, alimenter en **5V** | `RTClib`, `RTC_DS1307 rtc_clock` |
| GPS | `SoftwareSerial RX=6, TX=7` à 9600 baud | `TinyGPSPlus`, objet `gps` + `gpsSerial` |
| Carte SD | SPI `MOSI=11 MISO=12 SCK=13`, `CS=4`, `D10` forcé en `OUTPUT` (maître SPI) | `SD.h` |
| LED Chainable RGB | `CLK=8, DATA=9`, `LED_COUNT=1` | `ChainableLED leds` |
| Bouton rouge | `D2`, `INPUT_PULLUP` (actif à `LOW`) | interruption Timer1 |
| Bouton vert | `D3`, `INPUT_PULLUP` | interruption Timer1 |

> La LED a été déplacée sur D8/D9 pour libérer D4 (CS SD) — D3 est le bouton vert.

Contraintes : Uno ~32 Ko flash, projet à ~99,9 %. D'où optimisations radicales
(see §9).

---

## 2. Environnements de compilation (`platformio.ini`)

| Env | Flags | Usage |
|---|---|---|
| `uno` | `-flto -Os -mcall-prologues` | nominal, avec RTC |
| `uno_sans_rtc` | idem + `-DSANS_RTC` | sans horloge RTC |

`uno_sans_rtc` active une horloge fictive monotone :
`FAUSSE_DATE_BASE (01/01/2026 00:00:00 UTC) + millis()/1000`
via `GenererDateSansRtc()`. Triable et compatible rotation journalière,
repart à zéro à chaque reset. Compiler/injecter :
`pio run -e uno_sans_rtc -t upload`.

`monitor_speed = 9600`, ports auto-détectés.

---

## 3. Modes de fonctionnement

```cpp
enum ModeFonctionnement { MODE_CONFIG, MODE_STANDARD, MODE_ECO, MODE_MAINTENANCE };
ModeFonctionnement modeActuel = MODE_STANDARD;
```

| Mode | Couleur LED continue | Rôle |
|---|---|---|
| STANDARD | vert | mesure périodique + SD, GPS à chaque mesure |
| CONFIG | jaune | réglage paramètres par série |
| ECO | bleu | comme STANDARD mais `LOG_INTERVAL x2`, GPS 1 mesure sur 2 |
| MAINTENANCE | orange | affichage direct sur série, sans écrire sur SD |

### Transitions (gérées dans `loop()` + `ModeNormalOrEco()` + `ModeMaintenance()`)

```
Au boot : [ROUGE tenu au démarrage] → CONFIG
          sinon → STANDARD

CONFIG ──(rouge appui long 5s)──> STANDARD
STANDARD ──(vert appui long)──> ECO
STANDARD ──(rouge appui long)──> MAINTENANCE
ECO ──(rouge appui long)──> STANDARD  (retour direct, pas de maintenance)
MAINTENANCE ──(rouge appui long)──> STANDARD
```

`loop()` ne fait que dispatcher + rafraîchir la couleur LED lors d'un
changement (`dernierModeAffiche` statique → `AfficheCouleurMode()`).

---

## 4. Démarrage `setup()`

Ordre volontaire (commenté dans le code) :

1. `Serial.begin(9600)` + `« Lancement »`.
2. `Wire.begin()`, `Wire.setWireTimeout(25000, true)` (25 ms + reset auto bus),
   `clearWireTimeoutFlag()`. **I2C avant tout** : SoftwareSerial bavard faussait les timings.
3. `InitBoutons()` — configure pull-ups, états initiaux, Timer1 en CTC à 1 kHz
   (voir §7). Mémorise immédiatement `demandeConfig = BoutonRougePresse() || BoutonRougeAppui()`
   car les inits suivantes (RTC, SD, GPS jusqu'à 120 s) sont bloquantes et
   l'utilisateur peut relâcher avant la fin.
4. `InitLed()` → LED éteinte (noir).
5. `InitRtc()` : 5 essais `rtc_clock.begin()`, sinon **erreur 1**. En `SANS_RTC` : message seul.
6. `InitCapteur()` : 5 essais `capteur.begin(0x76)` puis `0x77`, sinon **erreur 3**.
7. `InitSd()` : `pinMode(10, OUTPUT)` puis `SD.begin(4)`, sinon **erreur 5**.
8. `InitEeprom()` : voir §6.
9. `InitGps()` en dernier : `gpsSerial.begin(9600)`, attend `gps.location.isValid()`
   avec timeout 120 s → **erreur 2** puis continue. En `SANS_RTC` affiche un `.` toutes les 5 s.
10. Choix mode : `demandeConfig || bouton rouge (pressé/appui)` → `MODE_CONFIG`
    (`=CONFIG=` + prompt `> `), sinon `MODE_STANDARD`.

Pas de scan I2C au boot (242 o économisés) : un problème I2C remonte via erreurs 1/3.

---

## 5. Boucle principale par mode

### 5.1 `ModeConfig()` — non bloquant

Appelé en boucle par `loop()`. Draine `Serial` sans attendre :
parse `COMMANDE=VALEUR` (buffers `static` 16 o, 15 car. + `\0`, surplus ignoré —
`static` car `loop()` rappelle en boucle : en locaux la ligne lente était perdue
et Entrée semblait sans effet).
`\r`, `\n`, `\0` valident la ligne (lignes vides ignorées pour `\r\n`, sauf
saisie type `=5` qui répond `Commande inconnu`).
Contrôles `< ' '` ignorés sans echo (flèches ESC, etc.). Echo manuel par
`Serial.write()` car le moniteur PlatformIO n'a pas d'echo local.
`=` bascule de `commande` vers `valeur`.

- `ProcessCommande(commande, valeur)` :
  cherche le paramètre (`GetParam`), parse entier signé manuel (remplace `strtol`,
  ~500 o), vérifie `[MinValue, MaxValue]` → `« Commande inconnu »`,
  `« Nombre invalide »`, `« Valeur out of bound !! »` ou `NULL` = succès.
  En succès : `SaveEepromParameters()` + `« OK »`. Affiche `> ` après chaque ligne.
- Sortie testée par `loop()` : `BoutonRougeAppuiLong()` → `MODE_STANDARD`.
  À l'entrée en config, l'appui long en cours est neutralisé (`longSignale`
  armé : un bouton encore tenu après les inits bloquantes — GPS jusqu'à 120 s —
  ne provoque pas de sortie immédiate). La sortie exige un NOUVEL appui long
  de 5 s (un front descendant ré-arme). Idem pour VERT (pas de ECO intempestif).

### 5.2 `ModeNormalOrEco(bool ecoMode)` — bloquant avec attente active

`ecoMode=false` = STANDARD (`« ModeNormal »`), `true` = ECO (`« ModeEco »`).
`sansPositionOk = false` (GPS exigé).

- `intervalle = LOG_INTERVAL(s) * 1000 * (ecoMode ? 2 : 1)`.
- `shouldDoGPS = true` ; en ECO on alterne (`shouldDoGPS = !shouldDoGPS`) → GPS frais
  1 mesure sur 2 pour économiser l'énergie.
- Boucle infinie :
  1. `record = GetNewRecord(shouldDoGPS)` ; si non-NULL, `AppendSdCard(record)`
     **prend possession** (ne pas `free` ici). Si `AppendSdCard > 0` (erreur fichier)
     → retour `MODE_STANDARD` (en pratique on reste, `loop()` relance).
  2. Attente `intervalle` par tranches de 10 ms avec `PompeGps()` (drain buffer GPS 64 o)
     et scrutation boutons :
     - vert long + non-eco → `MODE_ECO`, return.
     - rouge long → si eco `MODE_STANDARD`, sinon `MODE_MAINTENANCE` (+ message série), return.

### 5.3 `ModeMaintenance()` — boucle affichage

`sansPositionOk = true` → mesures intérieures sans fix GPS autorisées (position 0.00000).
Boucle infinie :

- rouge long → `« Retour en mode standard »`, `modeActuel=STANDARD`, `sansPositionOk=false`, return.
- `record = GetNewRecord(true)` ; si non-NULL → `PrintRecord(record)` + `free(record)`.
  Si NULL (erreur déjà signalée), on n'imprime rien.
- `100 x { PompeGps(); delay(10); }` ≈ 1 s entre affichages tout en nourrissant le GPS.

`PrintRecord()` : tableau série `| Date : ... |`, `Latitude`, `Longitude`,
`Temperature (.. C)`, `Humidite (.. %)`, `Pression (.. hPa)` via `FormatDecimal`
+ `AfficheLigne()`. `NULL` → lignes vides (non utilisé en maintenance).

---

## 6. Paramètres et EEPROM (`main.h`, `InitEeprom`, `SaveEepromParameters`)

14 paramètres en RAM (`Parameters losParametros[14]`), noms en flash `PROGMEM`
(`NOM_*`, 13 car. max) lus par `strcmp_P`.

| Param | Défaut | Min–Max | Rôle |
|---|---|---|---|
| `LOG_INTERVAL` | 10 | 1–1440 | période mesure (s, x2 en ECO) |
| `TIMEOUT` | 30 | 1–255 | durée max lecture BME (interprétée en s dans le code : `*1000`) |
| `FILE_MAX_SIZE` | 2048 | 1–65535 | taille max fichier LOG (octets) |
| `LUMIN` | 1 | 0–1 | (réservé, non exploité) |
| `LUMIN_LOW` | 255 | 0–1023 | (réservé) |
| `LUMIN_HIGH` | 768 | 0–1023 | (réservé) |
| `TEMP_AIR` | 1 | 0–1 | (activation, non exploité) |
| `MIN_TEMP_AIR` | -10 | -40–85 | borne basse température (°C) |
| `MAX_TEMP_AIR` | 60 | -40–85 | borne haute température |
| `HYGR` | 1 | 0–1 | (activation) |
| `HYGR_MINT` | 0 | -40–85 | borne basse humidité (%) |
| `HYGR_MAXT` | 50 | -40–85 | borne haute humidité |
| `PRESSURE_MIN` | 800 | 300–1100 | borne basse pression (hPa) |
| `PRESSURE_MAX` | 1100 | 300–1100 | borne haute pression |

Plan EEPROM : `0: magic uint16 0xACAB`, `2: VERSION (1)`, `3: NUMERO_LOT (1)`,
`4+: 14 x long` (params).

- Premier boot (`magic != 0xACAB`) : écrit magic + version + lot + `SaveEepromParameters()`.
- Sinon : réécrit version/lot, relit chaque `long` et n'adopte que si dans `[Min,Max]`.
- `GetParam(const char*)` : recherche `strcmp_P` ; surcharge
  `GetParam(const __FlashStringHelper*)` copie le nom flash en tampon RAM 16 o
  puis délègue. Noms `F("...")` à l'appel pour rester en flash.

---

## 7. Boutons poussoirs (anti-rebond + appui long)

Structure volatile `EtatBouton { pin, enfonce, appui, appuiLong, longSignale, derniereModif, debut }`
pour `boutonRouge` (D2) / `boutonVert` (D3).

- `InitBoutons()` : `INPUT_PULLUP`, capture états initiaux, `debut/derniereModif = millis()`,
  Timer1 CTC : `TCCR1A=0, TCCR1B=WGM12|CS11|CS10 (prescaler 64), OCR1A=249, TIMSK1=OCIE1A`
  → ISR à 1 kHz (16 MHz / 64 / 250).
- `ISR(TIMER1_COMPA_vect)` : `ScannerBouton()` sur les 2 boutons avec `millis()`.
- `ScannerBouton()` : `niveau = digitalRead == LOW` ; si changement et
  `maintenant-derniereModif >= ANTI_REBOND_MS (30)` → met à jour, sur front descendant
  `appui=true, debut=maintenant, appuiLong=false, longSignale=false`.
  Si `enfonce && !longSignale && maintenant-debut >= DUREE_APPUI_LONG (5000)` →
  `appuiLong=true, longSignale=true` (une seule fois par pression).
- Lecture : `BoutonRougePresse()/VertPresse()` = niveau instantané ;
  `Bouton*Rouge/Vert*Appui()/AppuiLong()` = `PrendreEvenement()` atomique
  (`noInterrupts/interrupts`, lit puis efface le drapeau).

Au `setup()`, `InitBoutons()` initialise `appui = enfonce` pour détecter un bouton
déjà tenu pendant le reset.

---

## 8. Acquisition `GetNewRecord(int withGps)` → `Record*` ou `NULL`

`Record { double Latitude, Longitude ; DateTime Date ; float Temperature, Humidity, Pressure }`,
alloué par `malloc(sizeof(Record))` (appelant propriétaire, `free` après usage
sauf via `AppendSdCard` qui prend possession).

1. **GPS** si `withGps` : `PompeGps()` (draine `gpsSerial` → `gps.encode()`),
   si `gps.location.isValid()` → maj `LesKordonnesGps[2]` (`float[2]`, initialisé à 0).
   Sinon si `!sansPositionOk` → **erreur 2**, `NULL`.
   Refus `0,0` (jamais de fix) si `!sansPositionOk` → **erreur 2**, `NULL`.
2. **BME280** : `debut=millis()`, `clearWireTimeoutFlag()`,
   `temperature=readTemperature()`, `pression=readPressure()/100 (Pa→hPa)`,
   `humidity=readHumidity()`, `timeoutI2C=getWireTimeoutFlag()`, `duree=millis()-debut`.
   Si `isnan(...) || timeoutI2C || duree > TIMEOUT*1000` → `SumCapteurError++`,
   si `>=2` → **erreur 3**, `NULL`. Sinon `SumCapteurError=0` (2 échecs consécutifs exigés).
3. **Cohérence** : `ValidCapteurData(MIN/MAX...)` sur T/H/P (lit bornes via `GetParam`) :
   somme > 0 → **erreur 4**, `NULL`.
4. **Horodatage** : `SANS_RTC ? GenererDateSansRtc() : rtc_clock.now()` avec
   contrôle timeout I2C → **erreur 1**, `NULL`.
5. `malloc`, remplit champs, retourne pointeur.

---

## 9. Stockage carte SD

- `GetFile(DateTime*, File&)` : cherche `AAMMJJ_i.LOG` (`i=0..9`, format 8.3 pour lib SD),
  `sprintf_P("%02d%02d%02d_%d.LOG", yy, mm, dd, i)`, ouvre `FILE_WRITE`,
  retourne le premier dont `size() < FILE_MAX_SIZE`. Échec open → **erreur 5**+`false` ;
  les 10 pleins → **erreur 6**+`false`.
- `AppendSdCard(Record*)` : `batch[SavedBatch++]=record` (`BATCH_SIZE=1`,
  lot de 1 car 5 enregistrements ne tiennent pas en RAM libre ~100 o).
  Quand plein : `GetFile(date du dernier)`, pour chaque entrée formate
  `lat(6);lon(6);temp(2);humi(2);press(2)\n` (buffer 64 o, `FormatDecimal`,
  `;` et `\n` manuels) puis `fichier.write()`. Échec partiel → **erreur 6**.
  `close()` puis `LibererLot()` (`free` chaque + `SavedBatch=0`). Retour `1=erreur, 0=ok`.
- `FormatDecimal(float v, uint8_t decimales, char *out)` : écrit sans `%f`
  (printf flottant désactivé pour la flash), arrondi `+0.5`, bourrage zéros
  fractionnaires, retourne fin de chaîne (pratique pour ajouter ` C`, ` %`, ` hPa`).

Exemple de ligne : `48.85661;2.35222;21.45;55.12;1013.25`

---

## 10. Signalisation LED et erreurs

Table `COULEURS_LED[][3]` en `PROGMEM` (noir, vert, jaune, bleu, orange, rouge, blanc),
`AppliqueCouleur(idx)` via `pgm_read_byte` + `setColorRGB(0,...)`.
`noinline,noclone` sur `AppliqueCouleur`, `AfficheCouleurMode`, `ClignoteLed`
pour éviter une spécialisation par constantes (~250 o chacune).

- `AfficheCouleurMode()` : jaune=CONFIG, bleu=ECO, orange=MAINTENANCE, vert=STANDARD (défaut).
- `ClignoteLed(c1,c2,d1,d2)` : 4 alternances `d1/d2` puis retour couleur mode.
  1 Hz → `500/500` ; « 2x plus long » → `333/667`.
- `OnImprimeLerreurViaLed(mode)` : `Serial.print("Erreur N : ...")` + clignotant :

| Code | Série | LED alternée |
|---|---|---|
| 1 | horloge RTC | rouge / bleu 500/500 |
| 2 | GPS sans fix | rouge / jaune 500/500 |
| 3 | capteur BME280 | rouge / vert 500/500 |
| 4 | donnée capteur incohérente | rouge / vert 333/667 |
| 5 | accès carte SD | rouge / blanc 333/667 |
| 6 | carte SD pleine | rouge / blanc 500/500 |

---

## 11. Optimisations mémoire (flash/RAM)

Flash saturée → chaque octet compte (`platformio.ini`, commentaires code) :
`-flto -Os -mcall-prologues`, `F()`/`PSTR`/`sprintf_P`/`snprintf_P`/`strcmp_P` partout,
noms paramètres en `PROGMEM`, `FormatDecimal` maison, parsing entier manuel
(au lieu de `strtol`), `noinline,noclone` LED, pas de `String` Arduino,
`Wire.setWireTimeout(25 ms, reset auto)`, buffers série statiques 16 o, `BATCH_SIZE=1`,
pas de scan I2C au boot, messages GPS réduits.

---

## 12. Utilisation type

1. Maintenir ROUGE au reset → mode CONFIG (LED jaune).
2. Terminal 9600 baud : `LOG_INTERVAL=5` → `OK`, etc. ; rouge long 5 s → STANDARD.
3. STANDARD (vert) : log toutes les `LOG_INTERVAL` s dans `AAMMJJ_0.LOG`,...
4. Vert long → ECO (bleu, période doublée, GPS 1/2) ; rouge long → MAINTENANCE (orange,
   tableau série chaque ~1 s) ; rouge long → STANDARD.
5. En `SANS_RTC`, dates fictives 01/01/2026 + uptime ; GPS timeout 120 s max au boot.

Limites connues : `TIMEOUT` documenté en secondes mais comparé à `millis()` en ms
(`*1000`), `ProcessCommande` refuse `>100000000` avant bornes, `Record.Latitude/Longitude`
en `double` mais stockés via `float LesKordonnesGps[2]`, `MaxFileSize` global inutilisé
(au profit du paramètre `FILE_MAX_SIZE`).
