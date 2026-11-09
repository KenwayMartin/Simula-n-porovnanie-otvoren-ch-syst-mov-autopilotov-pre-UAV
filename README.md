# Simulačné porovnanie otvorených systémov autopilotov pre UAV

Bakalársky projekt zameraný na porovnanie autopilotov **PX4** a **ArduPilot** v simulovanom prostredí s využitím **ROS 2** a **Gazebo**. 

Aktuálne je pripravený experiment pre PX4: automatický vzlet, pomalý pohyb smerom k prekážke, zastavenie podľa údajov hĺbkovej kamery, udržanie polohy a pristátie. Priebeh experimentu sa zaznamenáva do CSV. Porovnávací experiment s ArduPilotom je ďalšou etapou projektu.

## Aktuálny stav

- PX4 SITL s modelom `gz_x500_depth` v prostredí `baylands`.
- Prepojenie PX4 s ROS 2 cez MAVROS.
- Prenos farebného obrazu a hĺbkových snímok z Gazebo do ROS 2.
- Externé riadenie experimentu v režime OFFBOARD.
- Diagnostické zobrazenie kontrolovanej oblasti hĺbkového obrazu.
- Záznam polohy, rýchlosti, hĺbky a fáz experimentu do CSV.

Rozhodnutie o zastavení vykonáva vlastný uzol ROS 2. PX4 zabezpečuje vykonanie zadaného pohybu a stabilizáciu vozidla. Tento experiment netestuje vstavanú funkciu vyhýbania sa prekážkam v autopilote a zatiaľ neplánuje oblet prekážky.

## Štruktúra projektu

Pracovný adresár projektu je `/home/user/Projects/BP/`. Strom zobrazuje používané lokálne rozloženie; obsah zostavení, virtuálneho prostredia a externých repozitárov je skrátený.

```text
BP/
├── README.md
├── .gitignore
├── dependencies.txt
├── start.txt
├── .venv/
├── Systems/
│   ├── PX4-Autopilot/
│   ├── ardupilot/
│   ├── betaflight/
│   └── Micro-XRCE-DDS-Agent/
├── tools/
│   └── QGroundControl-x86_64.AppImage
├── scripts/
│   ├── start_sim.sh
│   ├── obstacle_stop.py
│   ├── depth_debug.py
│   ├── record_telemetry.py
│   └── logs/                         # prípadné staršie záznamy
├── logs/
│   ├── obstacle_stop_<čas>.csv
│   └── telemetry_px4_<čas>.csv
├── simulation/
│   └── worlds/
│       ├── kosice.sdf
│       └── defaut.sdf.backup
└── Konzultations/
    ├── konsultacia2.txt
    └── konsultacia3.txt
```

### Adresáre a pomocné súbory

| Cesta | Účel |
| --- | --- |
| `Systems/PX4-Autopilot/` | Zdrojový kód PX4, konfigurácia modelov a zostavenie SITL. |
| `Systems/ardupilot/` | Zdrojový kód ArduPilotu pre pripravované porovnanie. |
| `Systems/Micro-XRCE-DDS-Agent/` | Zdrojový kód agenta pre komunikáciu cez Micro XRCE-DDS. Aktuálny experiment používa MAVROS a agenta nevyžaduje. |
| `tools/` | Samostatné aplikácie, aktuálne QGroundControl vo formáte AppImage. |
| `scripts/` | Spúšťacie, riadiace a diagnostické skripty projektu. |
| `logs/` | Hlavný výstupný adresár CSV záznamov experimentov a telemetrie. |
| `scripts/logs/` | Staršie záznamy vytvorené pred úpravou výstupných ciest, ak sa ešte nachádzajú v pracovnej kópii. |
| `simulation/worlds/` | Archivované súbory vlastných svetov; aktuálny test používa `baylands`. |
| `Konzultations/` | Poznámky z konzultácií k bakalárskej práci. |
| `.venv/` | Lokálne virtuálne prostredie Pythonu, ktoré spúšťací skript aktivuje pri štarte PX4. |
| `dependencies.txt` | Adresy zdrojových repozitárov a zaznamenané Git revízie závislostí. |
| `start.txt` | Stručné pomocné príkazy na spustenie. |
| `.gitignore` | Pravidlá určujúce, ktoré lokálne súbory Git pri pridávaní ignoruje. |

Umiestnenie externého repozitára v lokálnom strome samo osebe neznamená, že jeho celý obsah je uložený v tomto GitHub repozitári. Pri príprave ďalšieho pracovného prostredia treba overiť dostupnosť závislostí podľa `dependencies.txt`.

### Skripty

| Skript | Funkcia |
| --- | --- |
| `start_sim.sh` | Spustí PX4 SITL, Gazebo so svetom `baylands`, most farebnej kamery a `rqt_image_view`. |
| `obstacle_stop.py` | Riadi experiment so zastavením pred prekážkou a automaticky zaznamenáva CSV. Bez `--fly` pracuje iba ako monitor. |
| `depth_debug.py` | Zobrazuje hĺbku, oblasť vyhodnocovania a bod minimálnej vzorkovanej hĺbky. Neposiela letové príkazy. |
| `record_telemetry.py` | Samostatne zaznamenáva odhad polohy, orientáciu a údaje o lineárnom zrýchlení z IMU. |

Súbor `wall.sdf` použitý pri testoch opisuje statickú stenu s hrúbkou 0,3 m, šírkou 6 m a výškou 4 m. Obsahuje vizuálnu aj kolíznu geometriu. Odporúčané umiestnenie modelu je `simulation/models/wall.sdf`. Nasledujúci postup jeho vytvorenia v simulácii predpokladá, že súbor je už na tejto ceste; samotné spustenie hlavného skriptu stenu nepridáva.

## Prostredie a predpoklady

Projekt sa vyvíja na Arch Linuxe s prostredím Hyprland. ROS 2 Jazzy a súvisiace nástroje sa používajú v Docker kontajneri. Príkazy nižšie predpokladajú pripravené vývojové prostredie; nejde o úplný postup inštalácie od čistého systému.

Potrebné komponenty:

- PX4 s inicializovanými podmodulmi a funkčným zostavením SITL pre Gazebo;
- Gazebo s dostupným svetom `baylands` a modelom `gz_x500_depth`;
- ROS 2 Jazzy, MAVROS a balík `mavros_msgs`;
- `ros_gz_bridge` a `rqt_image_view`;
- systémový Python s `rclpy`, NumPy a OpenCV;
- Bash a príkaz `setsid`;
- prístup kontajnera ku grafickému prostrediu pre Gazebo a prehliadače obrazu.

Presné zaznamenané revízie zdrojových závislostí sú v `dependencies.txt`. Pythonové uzly sa spúšťajú cez `/usr/bin/python3`, aby používali prostredie s nainštalovanými balíkmi ROS 2.

## Spustenie experimentu

Každý dlhodobo bežiaci proces spustite v samostatnom termináli kontajnera. V novom termináli najprv načítajte ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/user/Projects/BP
```

### 1. PX4, Gazebo a farebná kamera

```bash
bash scripts/start_sim.sh
```

Skript používa zdrojový adresár `Systems/PX4-Autopilot/`, virtuálne prostredie `.venv/` a spúšťa ekvivalent príkazu:

```bash
PX4_GZ_WORLD=baylands make px4_sitl gz_x500_depth
```

V okne `rqt_image_view` vyberte farebný obraz:

```text
/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image
```

Skript nezapína MAVROS ani most hĺbkovej kamery. Tie sa spúšťajú samostatne v nasledujúcich krokoch.

### 2. MAVROS

```bash
ros2 launch mavros px4.launch fcu_url:='udp://:14540@'
```

V ďalšom termináli overte spojenie:

```bash
ros2 topic echo /mavros/state --once
```

Pred pokračovaním musí byť `connected: true`. Na začiatku automatického testu musí byť dron na zemi a odzbrojený (`armed: false`).

### 3. Most hĺbkovej kamery

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image'
```

Overenie prijatého obrazu bez výpisu celého poľa pixelov:

```bash
ros2 topic echo /depth_camera --once --no-arr \
  --qos-reliability best_effort
```

Používaná kamera poskytuje hĺbku s kódovaním `32FC1`; namerané hodnoty sa v skriptoch spracúvajú v metroch.

### 4. Kontrola údajov bez letu

```bash
/usr/bin/python3 scripts/obstacle_stop.py
```

Tento režim neposiela letové príkazy. Vo výpise skontrolujte `link=True`, `pose=True` a platnú číselnú hĺbku. Každý štart tohto skriptu vrátane monitorovania vytvorí nový CSV súbor.

Diagnostické okno možno spustiť paralelne:

```bash
/usr/bin/python3 scripts/depth_debug.py
```

- Žltý obdĺžnik označuje vyhodnocovanú oblasť obrazu.
- Červený krížik označuje bod minimálnej platnej vzorkovanej hĺbky.
- Text zobrazuje hĺbku, podiel platných vzoriek a vek prijatých údajov.
- Okno sa zatvára klávesom `Q` alebo `Esc`.

Pri státí na zemi môže minimum patriť povrchu zeme. Pred hodnotením zastavenia treba overiť, že počas vodorovného pohybu minimum zodpovedá testovanej prekážke.

### 5. Automatický letový test

Pred dron umiestnite širokú stenu tak, aby bola viditeľná aj po vzlete na 2,5 m. Pri skúške zastavenia z pohybu má byť po vzlete vzdialenejšia než prah 4 m, napríklad približne 6 až 7 m podľa kamery. Priestor nad dronom musí byť voľný, pretože skript pri vzlete nekontroluje prekážky nad vozidlom.

Najprv ukončite samostatné monitorovanie `obstacle_stop.py`, aby následne bežala iba jedna inštancia riadiaceho skriptu. Simuláciu, MAVROS a most kamery ponechajte zapnuté.

```bash
/usr/bin/python3 scripts/obstacle_stop.py --fly
```

Sled fáz experimentu:

```text
WAIT, WARMUP, OFFBOARD, ARM, TAKEOFF, HOVER, FORWARD, STOP, LAND
```

| Parameter | Predvolená hodnota | Význam |
| --- | --- | --- |
| `--height` | `2.5` | Výška vzletu nad počiatočnou odhadovanou polohou v metroch. |
| `--speed` | `0.3` | Rýchlosť posúvania zadanej polohy v m/s; nejde o priamy príkaz skutočnej rýchlosti. |
| `--stop-distance` | `4.0` | Prah vyhodnotenej hĺbky pre zastavenie v metroch. |
| `--max-travel` | `5.0` | Limit plánovaného posunutia zadanej polohy vpred v metroch. |
| `--depth-topic` | `/depth_camera` | Vstupný ROS 2 topik hĺbkového obrazu. |
| `--csv` | automatický názov | Voliteľná cesta k novému výstupnému CSV súboru. Existujúci súbor sa neprepisuje. |

Po dosiahnutí prahu prekážky sa ďalšie posúvanie zadanej polohy zastaví. Skript prikáže držať aktuálnu odhadovanú polohu, približne tri sekundy čaká a následne žiada pristátie. Zastavenie môže vyvolať aj limit dráhy alebo času; príčina sa zapisuje do denníka a CSV.

Pri strate požadovaných vstupných údajov počas riadenej časti testu skript zruší ďalší pohyb a pokúsi sa vyžiadať pristátie. Úspech príkazu závisí od dostupnosti spojenia a stavu autopilota.

### 6. QGroundControl

V prostredí s dostupným grafickým výstupom:

```bash
/home/user/Projects/BP/tools/QGroundControl-x86_64.AppImage
```

QGroundControl slúži na sledovanie stavu vozidla, kontrolu hlásení pred vzletom, nastavenie parametrov a manuálne vyžiadanie režimu alebo pristátia. Automatický test si bežne vyžiada OFFBOARD a armovanie sám.

### 7. Ukončenie

Počas letu `Ctrl+C` v termináli `obstacle_stop.py --fly` vyžiada pristátie. Počkajte na hlásenie `Disarmed. Test finished.`. Pri normálnom dokončení skript uloží a zatvorí CSV.

Až po ukončení letu zastavte MAVROS, most hĺbky a spúšťací skript. `Ctrl+C` v `start_sim.sh` ukončuje procesy, ktoré tento skript spustil.

## Pridanie testovacej steny

Ak je model uložený v `simulation/models/wall.sdf`, pridajte ho do bežiaceho sveta:

```bash
gz service -s /world/baylands/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "/home/user/Projects/BP/simulation/models/wall.sdf", name: "bp_test_wall"'
```

V Gazebo vyberte model `bp_test_wall`. Kláves `T` zapína presúvanie a `R` otáčanie. Stenu umiestnite pred nos drona, kolmo na smer približovania. Pri zmene rozmerov SDF upravte rovnako rozmery vizuálnej aj kolíznej geometrie; existujúci model treba potom nahradiť novou inštanciou.

## Samostatný záznam telemetrie

Pri bežiacom MAVROS:

```bash
/usr/bin/python3 scripts/record_telemetry.py
```

Skript prijíma polohu z `/mavros/local_position/pose` a IMU z `/mavros/imu/data` s režimom spoľahlivosti `BEST_EFFORT`. Zapisuje čas, súradnice X/Y/Z, orientáciu vo forme kvaterniónu a lineárne zrýchlenie z IMU do `logs/telemetry_px4_<čas>.csv`. Záznam ukončíte cez `Ctrl+C`.

Tento jednoduchý záznam kombinuje poslednú prijatú polohu a IMU údaj. Nie je časovo synchronizovaný a nemá rovnakú štruktúru ako CSV z `obstacle_stop.py`.

## Výsledky a interpretácia CSV

`obstacle_stop.py` zapisuje približne 20 riadkov za sekundu do hlavného adresára `logs/`. Cesta k súboru sa vypíše pri štarte. Ak je skript v `scripts/`, predvolená cesta sa odvodzuje od koreňa projektu, nie od aktuálneho adresára terminála.

| Skupina | Príklady stĺpcov |
| --- | --- |
| Čas a fáza | `wall_time_utc`, `elapsed_monotonic_s`, `stage`, `stage_reason`, `event_id` |
| Stav autopilota | `connected`, `armed`, `mode` |
| Odhadovaná poloha | `x_m`, `y_m`, `z_m`, `height_from_start_m` |
| Rýchlosť z MAVROS | `vx_m_s`, `vy_m_s`, `vz_m_s`, `horizontal_speed_m_s` |
| Hĺbka | `depth_min_m`, `depth_valid_fraction`, `depth_available` |
| Čerstvosť a čas merania | `pose_fresh`, `velocity_fresh`, `pose_stamp_ns`, `velocity_stamp_ns`, `depth_stamp_ns` a vek jednotlivých vstupov |
| Odoslaná žiadaná poloha | `setpoint_published`, `setpoint_x_m`, `setpoint_y_m`, `setpoint_z_m` |
| Nastavenia experimentu | Výška, rýchlosť posúvania zadania, prah zastavenia a limit dráhy. |

Pri analýze platí:

- Riadok je snímkou posledných dostupných údajov z viacerých asynchrónnych topikov, nie spoločným synchronizovaným meraním.
- Prázdna hodnota znamená chýbajúci údaj, nie nameranú nulu. Hodnoty s príznakom čerstvosti `0` sa nesmú považovať za aktuálne.
- Časové pečiatky hĺbky môžu používať simulačný čas, zatiaľ čo MAVROS používa inú časovú základňu. Bez zosúladenia ich nemožno priamo odčítať na určenie oneskorenia.
- Hĺbka je minimum vo vzorkovanej oblasti kamery. Nie je to automaticky vzdialenosť steny od stredu alebo okraja drona.
- `horizontal_speed_m_s` je norma prijatých zložiek `vx` a `vy`. Pri porovnávaní treba overiť súradnicové rámce zaznamenané v `pose_frame` a `velocity_frame`.
- Poloha z MAVROS je odhad autopilota, nie referenčná skutočná poloha z fyziky Gazebo.

Kontrolovaná oblasť kamery pokrýva 20 až 80 % šírky a 30 až 70 % výšky snímky. Vyhodnocuje sa každý štvrtý pixel v oboch smeroch a vyžaduje sa aspoň 10 % konečných kladných vzoriek. Tento postup slúži na prvé skúšky so širokou prekážkou; neoddeľuje zem od ostatných objektov a neposkytuje priestorové plánovanie.

## Cesty a presúvanie projektu

Aktuálny `start_sim.sh` predvolene predpokladá koreň `/home/user/Projects/BP`. Pri inom umiestnení ho možno spustiť s premennou `BP_DIR`:

```bash
BP_DIR=/nova/cesta/BP bash scripts/start_sim.sh
```

Pythonové záznamové skripty odvodzujú hlavný adresár `logs/` od svojho umiestnenia v `scripts/`. Absolútne cesty v príkazoch na pridávanie SDF modelov a spúšťanie QGroundControl treba prispôsobiť novému umiestneniu.

Po presunutí zdrojového repozitára môže existujúce zostavenie CMake obsahovať staré absolútne cesty. Pri PX4 sa tento problém riešil odložením pôvodného `build/px4_sitl_default` a vytvorením novej zostavy. Podobná kontrola je potrebná pred opätovným zostavením presunutého Micro-XRCE-DDS-Agent. Presunuté virtuálne prostredie Pythonu môže tiež vyžadovať nové vytvorenie.

## Riešenie bežných problémov

| Problém | Kontrola alebo riešenie |
| --- | --- |
| `connected: false` | Overiť bežiaci PX4, MAVROS, UDP adresu a dostupnosť spojenia v použitom kontajneri. |
| `depth=UNKNOWN/STALE` | Overiť most `/depth_camera`, bežiacu simuláciu a podiel platných hĺbkových meraní v diagnostickom okne. |
| `pose=False` | Overiť tok údajov na `/mavros/local_position/pose`. Samotná existencia topika nestačí. |
| GeographicLib: chýbajúci `egm96-5.pgm` | V prostredí MAVROS doinštalovať dátové súbory pomocou `ros2 run mavros install_geographiclib_datasets.sh`, s oprávneniami potrebnými na zápis do systémového adresára. |
| Nezlučiteľné QoS pri prijímaní údajov | Overiť nastavenie `BEST_EFFORT`; záznamové skripty ho používajú pre senzorové odbery. |
| CMake odkazuje na neexistujúci starý adresár | Vytvoriť nové zostavenie pre aktuálne umiestnenie zdrojov. |
| `MicroXRCEAgent` sa nenašiel | Zdrojový adresár sám nepridáva spustiteľný program do `PATH`. Aktuálny experiment s MAVROS agenta nepotrebuje. |
| Armovanie alebo OFFBOARD bolo odmietnuté | Skontrolovať hlásenia PX4 a QGroundControl; skript neobchádza predletové kontroly. |

## Ďalšie etapy

1. Opakovanie rovnakých testov a vyhodnotenie rozptylu výsledkov.
2. Doplnenie referenčnej polohy z Gazebo.
3. Príprava zodpovedajúceho experimentu pre ArduPilot.
4. Porovnanie sledovania trajektórie, zastavenia a udržania polohy pri zdokumentovaných nastaveniach oboch autopilotov.
5. Rozšírenie o plánovanie pohybu okolo prekážok podľa dohodnutého rozsahu práce.

## Dokumentácia použitých projektov

- [PX4](https://docs.px4.io/)
- [ArduPilot](https://ardupilot.org/copter/docs/)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Gazebo](https://gazebosim.org/docs/)
- [MAVROS](https://github.com/mavlink/mavros)
- [Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)


