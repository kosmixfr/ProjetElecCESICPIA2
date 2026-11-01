# ACAB — Documentation de fonctionnement

Station météo embarquée sur **Arduino Uno** (PlatformIO).
Mesure température / humidité / pression (BME280), horodate (RTC DS1307),
localise (GPS), stocke sur carte SD, signale par LED RGB et 2 boutons,
configurable par liaison série.

Fichiers : `src/main.cpp` (~1390 lignes), `src/main.h` (paramètres + `Record`),
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
| Bouton rouge | `D2` (INT0), `INPUT_PULLUP` (actif à `LOW`) | interruption externe `CHANGE` |
| Bouton vert | `D3` (INT1), `INPUT_PULLUP` | interruption externe `CHANGE` |

> La LED a été déplacée sur D8/D9 pour libérer D4 (CS SD) — D3 est le bouton vert.

Contraintes : Uno ~32 Ko flash, projet à ~99,9 %. D'où optimisations radicales
(see §9).

---

## 2. Environnements de compilation (`platformio.ini`)

| Env | Flags | Usage |
|---|---|---|
| `uno` | `-flto -Os -mcall-prologues -DSERIAL_RX_BUFFER_SIZE=32 -DSERIAL_TX_BUFFER_SIZE=16` | nominal, avec RTC |
| `uno_sans_rtc` | idem + `-DSANS_RTC` | sans horloge RTC |

Les tampons série (`SERIAL_*_BUFFER_SIZE`) sont réduits pour libérer de la RAM
(voir §13). Ne pas les ré-augmenter sans vérifier la marge de pile.

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

CONFIG ──(rouge appui long 5s, ou 30 min sans saisie série)──> STANDARD
STANDARD ──(vert appui long)──> ECO
STANDARD ──(rouge appui long)──> MAINTENANCE
ECO ──(rouge appui long)──> STANDARD  (retour direct, pas de maintenance : choix retenu, §14)
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
3. `InitBoutons()` — configure pull-ups, états initiaux, interruptions INT0/INT1
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
- Sortie testée par `loop()` : `BoutonRougeAppuiLong()` ou 30 min sans saisie
  (`millis() - derniereActiviteConfig >= CONFIG_INACTIVITE_MS`) → `MODE_STANDARD`.
  `derniereActiviteConfig` est armé à l'entrée en config (fin de `setup()`) et repoussé
  à chaque caractère reçu sur la série. Seule la saisie série compte comme activité.
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
      (record statique, écrit aussitôt). Si `AppendSdCard > 0` (erreur fichier)
     → retour `MODE_STANDARD` (en pratique on reste, `loop()` relance).
  2. Attente `intervalle` par tranches de 10 ms avec `PompeGps()` (drain buffer GPS 64 o)
     et scrutation boutons :
     - vert long + non-eco → `MODE_ECO`, return.
     - rouge long → si eco `MODE_STANDARD`, sinon `MODE_MAINTENANCE` (+ message série), return.

### 5.3 `ModeMaintenance()` — boucle affichage

`sansPositionOk = true` → mesures intérieures sans fix GPS autorisées (position 0.00000).
Entrée : `SD.end()` + `carteSdOk = false` → la carte peut être retirée ou changée ; le prochain
enregistrement (retour en standard) refait `SD.begin()` dans `GetFile` avant toute écriture,
pour ne jamais réutiliser le volume FAT en cache sur une autre carte.
Boucle infinie :

- rouge long → `« Retour en mode standard »`, `modeActuel=STANDARD`, `sansPositionOk=false`, return.
- `record = GetNewRecord(true)` ; si non-NULL → `PrintRecord(record)`.
  Si NULL, affiche `Echec N` où `N` = `codeEchec`, le sous-système en faute :
  `1`=RTC, `2`=GPS, `3`=BME280, `4`=donnée incohérente.
  (Le `Record` est statique : plus de `malloc`, qui refusait — ex-`Echec 7` —
  car le tas n'a pas ~30 o libres avec 85 %+ de RAM statique.)
- `100 x { PompeGps(); delay(10); }` ≈ 1 s entre affichages tout en nourrissant le GPS.

`PrintRecord()` : tableau série `| Date : ... |`, `Latitude`, `Longitude`,
`Temperature (.. C)`, `Humidite (.. %)`, `Pression (.. hPa)` via `FormatDecimal`
+ `AfficheLigne()`. `NULL` → lignes vides (non utilisé en maintenance).

---

## 6. Paramètres et EEPROM (`main.h`, `InitEeprom`, `SaveEepromParameters`)

14 paramètres en RAM (`Parameters losParametros[14]` : nom + valeur), noms en flash `PROGMEM`.
Bornes min/max dans `LesBornes[14]` (`main.h`), en `PROGMEM` : lues par `BorneMin(i)` /
`BorneMax(i)` (même ordre que `losParametros`). Ne pas remettre les bornes en RAM (§13).
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

Structure `EtatBouton { pin, enfonce, appui, appuiLong, longSignale, derniereModif, debut, evenement }`
pour `boutonRouge` (D2, INT0) / `boutonVert` (D3, INT1).

- `InitBoutons()` : `INPUT_PULLUP`, capture états initiaux, `debut/derniereModif = millis()`,
  `attachInterrupt(..., CHANGE)` sur D2 et D3. Pas de Timer1.
- ISR `InterruptionBoutonRouge/Vert()` : pose seulement `evenement = true`.
- `ActualiserBouton()` (appelée par les accesseurs `Bouton*` et par `PompeGps()`) :
  prise atomique de `evenement` ; si `maintenant-derniereModif >= ANTI_REBOND_MS (30)` et
  niveau modifié (`digitalRead == LOW`) → met à jour, sur front descendant
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
en `static` dans `GetNewRecord` (jamais `malloc`/`free` : le tas (~300 o libres
moins la pile) est trop petit — `malloc` refusait systématiquement).

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
5. Remplit le `Record` statique, retourne son pointeur (toujours valide).

---

## 9. Stockage carte SD

- `GetFile(DateTime*, File&)` : cherche `AAMMJJ_i.LOG` (`i=0..9`, format 8.3 pour lib SD),
  `sprintf_P("%02d%02d%02d_%d.LOG", yy, mm, dd, i)`, ouvre `FILE_WRITE`,
  retourne le premier dont `size() < FILE_MAX_SIZE`. Échec open → **erreur 5**+`false` ;
  les 10 pleins → **erreur 6**+`false`.
- `AppendSdCard(Record*)` : écriture immédiate (pas de lot : ~100 o de RAM
  libre interdisent un lot de 5). `GetFile(date du record)`, formate
  `lat(6);lon(6);temp(2);humi(2);press(2)\n` (buffer 64 o, `FormatDecimal`,
  `;` et `\n` manuels) puis `fichier.write()`. Échec partiel → **erreur 6**.
  `close()`. Retour `1=erreur, 0=ok`.
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
| 6 | SD pleine | rouge / blanc 500/500 |

`codeEchec` (RAM, `GetNewRecord` + `OnImprimeLerreurViaLed`) mémorise le dernier
code (1–4, 6 ci-dessus ; le `7` = `malloc` refusé a disparu avec le `Record`
statique) ; la maintenance affiche `Echec N` à chaque mesure ratée pour dire
quel sous-système bloque
(raccourci `6` en `SD pleine` : flash à ~100 %, chaque octet compte).

---

## 11. Optimisations mémoire (flash/RAM)

Flash saturée → chaque octet compte (`platformio.ini`, commentaires code) :
`-flto -Os -mcall-prologues`, `F()`/`PSTR`/`sprintf_P`/`snprintf_P`/`strcmp_P` partout,
noms paramètres en `PROGMEM`, `FormatDecimal` maison, parsing entier manuel
(au lieu de `strtol`), `noinline,noclone` LED, pas de `String` Arduino,
ni de `malloc`/`free` (record statique : le tas est trop petit),
`Wire.setWireTimeout(25 ms, reset auto)`, buffers série statiques 16 o,
écriture SD immédiate (pas de lot), pas de scan I2C au boot, messages GPS réduits.
`PrintRecord` réutilise un seul tampon valeur 20 o + `AfficheLigne` 40 o
(184 o imbriqués dépassaient les ~270 o de pile libre et corrompaient la RAM
— flot d'octets illisibles sur la série).
Budget RAM/pile/flash : voir §13 (avant tout ajout).

---

## 12. Utilisation type

1. Maintenir ROUGE au reset → mode CONFIG (LED jaune).
2. Terminal 9600 baud : `LOG_INTERVAL=5` → `OK`, etc. ; rouge long 5 s → STANDARD.
3. STANDARD (vert) : log toutes les `LOG_INTERVAL` s dans `AAMMJJ_0.LOG`,...
4. Vert long → ECO (bleu, période doublée, GPS 1/2) ; en ECO, rouge long → STANDARD ;
   rouge long en STANDARD → MAINTENANCE (orange, tableau série chaque ~1 s) ;
   rouge long en MAINTENANCE → STANDARD.
5. En `SANS_RTC`, dates fictives 01/01/2026 + uptime ; GPS timeout 120 s max au boot.

Limites connues : `TIMEOUT` documenté en secondes mais comparé à `millis()` en ms
(`*1000`), `ProcessCommande` refuse `>100000000` avant bornes, `Record.Latitude/Longitude`
en `double` mais stockés via `float LesKordonnesGps[2]`, `MaxFileSize` global inutilisé
(au profit du paramètre `FILE_MAX_SIZE`).

---

## 13. Budget mémoire — à lire avant tout ajout

### 13.1 Chiffres (build actuel)

| Ressource | Valeur | Remarque |
|---|---|---|
| RAM statique (.data + .bss + .noinit) | 1599 / 2048 o (78 %) | `uno` : 1601 o. 1787 o avant les corrections (§13.4) |
| Pile libre au boot | ~337 o | mesuré par le diagnostic (§13.3), non remesuré après §14 |
| Pile libre minimale (cycle de mesure) | **17 o** | pic pendant l'écriture SD : marge trop faible |
| Tas maximal | 44 o | BME280 (4 `new` dans `begin()`), SD (1 `malloc` par `SD.open`) |
| Flash | 99,3 % (~214 o libres) | `uno` : 99,6 % (~116 o libres) |

La pile et le tas partagent la même zone de RAM (entre la fin de la RAM statique et
`RAMEND`), sans garde : un débordement n'est pas signalé, il corrompt la mémoire
(variables, objets `gps`/SD, adresses de retour) et provoque des resets silencieux.

### 13.2 Règles pour un ajout

1. Pas de nouvelle variable statique/globale sans nécessité : chaque octet statique réduit la pile.
2. Textes et tables constants en `F()` ou `PROGMEM` (ex. `LesBornes`, §6).
3. Chemin de mesure (`GetNewRecord`, `AppendSdCard`, `GetFile`, `ModeNormalOrEco`) : pas de
   tableau local de plus de 16 o, pas de copie de `File` / `SdFile` ni de grosse structure.
4. Pas de `String`, pas de `new`/`malloc` pour nos propres objets, pas de `sprintf`/`printf` flottant.
5. Ne pas ré-augmenter les tampons série (`SERIAL_*_BUFFER_SIZE`, §2) sans mesurer la marge.
6. Après chaque ajout : compiler `uno_sans_rtc` et `uno`, reporter RAM et flash ci-dessus,
   puis mesurer la marge de pile (§13.3) sur un run de plusieurs minutes.
7. Seuil proposé (à valider) : au moins 50 o de pile libre mesurée. En dessous, ne pas
   ajouter de fonctionnalité sur le chemin de mesure avant d'avoir libéré de la RAM.

### 13.3 Diagnostic temporaire (à retirer)

- Code marqué « DIAGNOSTIC RESET » dans `src/main.cpp` : `diag` (en `.noinit`, conservé au reset),
  `DiagPeintPile` (peint la RAM libre en `0xAA` au boot), `DiagPileLibre` / `DiagTasUtilise`,
  `Etape(n)`, `DiagDemarrage`.
- Sortie série à chaque repère : `E<étape> <pile libre> <tas max>`.
  Étapes : `1` setup, `A` avant RTC, `B` avant capteur, `C` avant SD, `D` avant EEPROM,
  `2` avant GPS, `3` début de mesure, `4` BME280, `5` horodatage, `6` ouverture SD,
  `7` écriture SD, `8` attente entre mesures.
- Au reset, la ligne `Reset: etape N pile min M` donne la dernière étape et la pile minimale
  du run précédent. Si la RAM a été corrompue, elle affiche `diag absent` ou une valeur incohérente.
- La pile libre est mesurée depuis le sommet maximal du tas : les blocs libérés ne faussent pas la mesure.
- Coût : ~11 o de RAM, ~400 o de flash, et des sorties série à chaque repère (ralentissent la boucle).
  À retirer avant de figer une version.

### 13.4 Historique : boucle de reset « Lancement / ModeNormal »

- Symptôme : reset périodique après `ModeNormal`, sans message d'erreur. `ModeNormal` répété et
  `=CONFIG=` affiché au milieu de la boucle (retour de fonction corrompu).
- Cause : la pile disposait de trop peu de place (RAM statique trop élevée) ; le tas et la pile se
  chevauchent. Le débordement corrompt la mémoire, d'où les symptômes incohérents.
- Corrections :
  - bornes min/max des paramètres déplacées en `PROGMEM` (`LesBornes`) : −112 o de RAM ;
  - tampons série réduits à 32 o (RX) et 16 o (TX) : −82 o de RAM.
- Résultat : boucle non reproduite sur 200 s de fonctionnement. Marge de pile encore faible (17 o).

---

## 14. Conformité au cahier des charges (4 modes)

Conforme :
- Démarrage sans bouton → STANDARD ; bouton rouge tenu au démarrage → CONFIG.
- CONFIG : paramètres configurables en série, acquisition capteurs désactivée.
- CONFIG → STANDARD : appui long rouge 5 s, et **30 min sans saisie série** (ajouté).
- STANDARD → ECO : appui long vert 5 s. ECO → STANDARD : appui long rouge 5 s.
- STANDARD → MAINTENANCE : appui long rouge 5 s. MAINTENANCE → STANDARD : appui long rouge 5 s.
- MAINTENANCE : données capteurs sur la série, aucune écriture SD ; carte SD réinitialisée
  au retour en acquisition (ajouté, voir §5.3).

Écarts :
- **ECO → MAINTENANCE non implémenté (choix retenu).** Le cahier des charges dit que la
  maintenance est accessible « depuis le mode standard ou économique » (appui rouge 5 s),
  mais le même document dit qu'en économique l'appui rouge 5 s ramène en standard. Les deux
  règles utilisent le même geste ; la règle « économique → standard » est conservée. Donc la
  maintenance n'est accessible que depuis STANDARD, et son « mode précédent » est toujours STANDARD.
- **Économie d'énergie partielle.** Le GPS n'est pas mis hors tension (aucune broche de
  commande, SoftwareSerial continue de recevoir) : l'économie se limite à `LOG_INTERVAL x2`
  et au GPS une mesure sur deux. Le BME280 reste lu à chaque mesure.
