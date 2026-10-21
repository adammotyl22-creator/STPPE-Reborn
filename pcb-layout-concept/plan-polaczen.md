# Układ pod krótkie połączenia silnikowe

Sześć potwierdzonych modułów BIGTREETECH TMC2209 V1.3 umieszczono w jednym dolnym rzędzie. Każdy ma własne czteropolowe złącze obok dolnej krawędzi; wszystkie 24 przewody fazowe pozostają na tej samej części płytki co sterownik. Ta aktualizacja dotyczy najnowszego planu połączeń silnikowych; wcześniejsze koncepcje są historyczne i pozostają bez zmian, podobnie jak podział A/B. To koncepcja montażu przewodowego, nie pełny schemat ani projekt PCB do produkcji.

## Geometria i orientacje

Wymiary i współrzędne w mm, początek jak w poprzednich koncepcjach. A: X −23,5…40; Y 0,5…120,5; wcięcie górne lewe 25,4 × 71,12; dolny lewy narożnik R7, środek (−16,5; 113,5). B: X 42…102,96; Y 0,5…120,5. Szczelina 2 mm. Obie części mieszczą się w osobnych formatach 80 × 120 mm.

Teensy: (−22,23; 72,18), 60,96 × 17,78; USB w lewo. Hub8: (7; 7), 31 × 18,3; XIAO: (7; 38), 21 × 17,8; LM2596: (57; 7), orientacyjnie 43 × 21. Antena XIAO pozostaje zewnętrzna.

Wszystkie sterowniki mają korpus 15,24 × 20,32, potwierdzony rysunkiem wymiarowym V1.3, oraz Y = 92,5. Wszystkie złącza mają Y = 113,53, korpus 10,62 × 6,2 i oś pinów Y = 116,63.

| Moduł | X sterownika | Orientacja | X złącza | Część |
|---|---:|---|---:|---|
| M1 | −22,23 | R, VM prawa góra | −17,38 | A |
| M2 | 0,63 | 180°, VM lewy dół | 2,94 | A |
| M3 | 23,49 | 180°, VM lewy dół | 25,80 | A |
| M4 | 44,54 | R, VM prawa góra | 46,85 | B |
| M5 | 64,86 | R, VM prawa góra | 67,17 | B |
| M6 | 85,18 | R, VM prawa góra | 87,49 | B |

Rozstaw modułów: lewy bank 22,86 mm (przerwa 7,62), prawy 20,32 mm (przerwa 5,08). Złącze M1 przesunięto o dodatkowe 2,54 mm w prawo, aby ominąć R7. Pozostałe złącza: X sterownika + 2,31.

Marginesy Teensy: po 1,27 mm z lewej i prawej, 0,56 mm od poziomu wcięcia; od Teensy do TMC 2,54 mm. Od TMC do korpusu złącza 0,71 mm, od złącza do dolnej krawędzi 0,77 mm. Są to marginesy samych brył; pola lutownicze, tolerancje i dostęp narzędzi pozostają do sprawdzenia.

## Pinout i 24 połączenia fazowe

**Potwierdzona wersja: BIGTREETECH TMC2209 V1.3.** Pinout zweryfikowano z oficjalnym zdjęciem opisanym TOP, przedstawiającym stronę ze złotym polem termicznym. Nie jest to uniwersalny pinout wszystkich modułów TMC2209. Przed włożeniem oraz podłączeniem USB sprawdzić VM, GND i fazy na faktycznych płytkach. Obrót 180° oznacza obrót w płaszczyźnie płytki, bez odwracania modułu spodem do góry.

Założenie geometryczne: 2 listwy po 8 pinów, podziałka 2,54 mm, odległość rzędów 12,7 mm, odsunięcie pinów 1,27 mm od krawędzi korpusu. Sam korpus 15,24 × 20,32 jest potwierdzony; podziałkę, odstęp rzędów i odsunięcia nadal trzeba zmierzyć. Wymiarów centralnego obszaru 8,40 × 12,88 na rysunku nie należy utożsamiać z rozstawem listew. Dla R lista silnikowa po prawej od góry: **VM, GND, A2, A1, B1, B2, VIO, GND**. Dla obrotu 180° lista po lewej od góry: **GND, VIO, B2, B1, A1, A2, GND, VM**.

Fizyczna listwa sterująca w widoku TOP/R po lewej od góry: **EN, MS1, MS2, RX, TX, CLK, STEP, DIR**. Po obrocie 180° znajduje się po prawej, od góry: **DIR, STEP, CLK, TX, RX, MS2, MS1, EN**. To kolejność fizyczna, nie numeracja P1A na schemacie. Rysunek nie przypisuje rzeczywistych numerów pinów Teensy.

Złącze każdego silnika od lewej: **A2, A1, B1, B2**. Cewka A to dwa pierwsze pola, B to dwa kolejne. Sprawdzić pary uzwojeń rzeczywistego silnika; jego kolory przewodów nie są uniwersalne. Przerywane linie to osobne izolowane przewody pod płytką, nie ścieżki PCB; przecięcia nie są węzłami. Zadbać o izolację, wysokość pod płytką, podparcie i odciążenie przewodów. Kolory na diagramie oznaczają cewki, nie wymagane kolory izolacji.

Poniżej współrzędne źródła i końca w mm oraz geometryczna odległość w linii prostej. To **nie są długości cięcia**: około 1–3 cm przewodu po ułożeniu; długości do przymiarki, z uwzględnieniem lutowania, gięcia i zapasu.

| Moduł | Sieć | Pin modułu X; Y | Pin złącza X; Y | Linia prosta mm |
|---|---|---|---|---:|
| M1 | A2 | -8.26; 98.85 | -15.88; 116.63 | 19.34 |
| M1 | A1 | -8.26; 101.39 | -13.34; 116.63 | 16.06 |
| M1 | B1 | -8.26; 103.93 | -10.80; 116.63 | 12.95 |
| M1 | B2 | -8.26; 106.47 | -8.26; 116.63 | 10.16 |
| M2 | A2 | 1.90; 106.47 | 4.44; 116.63 | 10.47 |
| M2 | A1 | 1.90; 103.93 | 6.98; 116.63 | 13.68 |
| M2 | B1 | 1.90; 101.39 | 9.52; 116.63 | 17.04 |
| M2 | B2 | 1.90; 98.85 | 12.06; 116.63 | 20.48 |
| M3 | A2 | 24.76; 106.47 | 27.30; 116.63 | 10.47 |
| M3 | A1 | 24.76; 103.93 | 29.84; 116.63 | 13.68 |
| M3 | B1 | 24.76; 101.39 | 32.38; 116.63 | 17.04 |
| M3 | B2 | 24.76; 98.85 | 34.92; 116.63 | 20.48 |
| M4 | A2 | 58.51; 98.85 | 48.35; 116.63 | 20.48 |
| M4 | A1 | 58.51; 101.39 | 50.89; 116.63 | 17.04 |
| M4 | B1 | 58.51; 103.93 | 53.43; 116.63 | 13.68 |
| M4 | B2 | 58.51; 106.47 | 55.97; 116.63 | 10.47 |
| M5 | A2 | 78.83; 98.85 | 68.67; 116.63 | 20.48 |
| M5 | A1 | 78.83; 101.39 | 71.21; 116.63 | 17.04 |
| M5 | B1 | 78.83; 103.93 | 73.75; 116.63 | 13.68 |
| M5 | B2 | 78.83; 106.47 | 76.29; 116.63 | 10.47 |
| M6 | A2 | 99.15; 98.85 | 88.99; 116.63 | 20.48 |
| M6 | A1 | 99.15; 101.39 | 91.53; 116.63 | 17.04 |
| M6 | B1 | 99.15; 103.93 | 94.07; 116.63 | 13.68 |
| M6 | B2 | 99.15; 106.47 | 96.61; 116.63 | 10.47 |

## Konkretne złącze i kondensatory

[Phoenix Contact MPT 0,5/4-2,54, 1725672](https://www.phoenixcontact.com/en-us/products/printed-circuit-board-terminal-mpt-05-4-254-1725672?type=pdf): korpus 10,62 × 6,2 mm, raster liniowy 4 pinów 2,54 mm, nominalny prąd 6 A, wysokość montażowa 8,5 mm, otwór montażowy 1,1 mm. Na koncepcji piny zaczynają się 1,5 mm od lewej krawędzi i leżą 3,1 mm od krawędzi korpusu w Y; **te odsunięcia trzeba potwierdzić rysunkiem konkretnego egzemplarza przed montażem**. Sprawdzić, czy otwory płytki można bezpiecznie dostosować do 1,1 mm bez zniszczenia pierścieni padów. Nominalne 6 A złącza nie wyznacza przekroju przewodów wspólnego zasilania.

Przykład C1–C6: Rubycon ZLH 100 µF / 35 V low ESR, korpus Ø6,3 × 11 mm, raster wyprowadzeń 2,5 mm. Źródło: [katalog Rubycon ZLH](https://www.rubycon.co.jp/wp-content/uploads/catalog-aluminum/ZLH.pdf). Sprawdzić konkretny numer zamówieniowy i warunki pracy; to przykład lokalnej filtracji, nie kompletny dobór całej sieci zasilania. Uwaga na wysokość 11 mm. Piny w koncepcji po Y ±1,27 mm: wymaga to ostrożnego uformowania nominalnego rastra 2,5 mm do 2,54, bez naprężania uszczelnienia. **Plus do VM, minus do lokalnego GND.**

| C | Środek X; Y | Przypisanie VM |
|---|---|---|
| C1 | −3,18; 97,58 | M1, VM u góry |
| C2 | −3,18; 107,74 | M2, VM u dołu |
| C3 | 19,68; 107,74 | M3, VM u dołu |
| C4 | 55,97; 87,42 | M4, VM u góry |
| C5 | 76,29; 87,42 | M5, VM u góry |
| C6 | 96,61; 87,42 | M6, VM u góry |

Kondensatory są nad płytką, przewody fazowe pod spodem. C1 i C4–C6: plus u góry (Y środka − 1,27); C2/C3: plus u dołu (Y środka + 1,27), do VM obróconego sterownika. Pozostałe wyprowadzenie do lokalnego GND. Rysunek nie pokazuje wszystkich lokalnych przewodów VM/GND; należy je wykonać możliwie krótko do właściwych pinów i sprawdzić polaryzację obróconych modułów. Założenia o listwach i osi pinów złącza nie są szablonem do wiercenia.

## Połączenia funkcjonalne

- Chronione 24 V IN → osobne sparowane przewody VM/GND dla lewego banku M1–M3 i prawego M4–M6, z lokalnymi kondensatorami. Rozdział mocy dedykowanymi przewodami; nie łańcuchem cienkich padów ani zwykłym połączeniem sygnałowym. Przekroje i zabezpieczenia zależą od rzeczywistych prądów wejściowych, nie od prostego przemnożenia prądu złącza.
- Z tego samego wejścia 24 V równoległa gałąź → LM2596 → 5 V do Teensy VIN i XIAO 5 V. VM silników nie przechodzi przez LM2596. Uwzględnić właściwe rozdzielenie VIN/VUSB Teensy i ochronę XIAO przed zasilaniem zwrotnym przy USB + zewnętrznym 5 V (według Seeed dioda: anoda od źródła, katoda do 5 V XIAO).
- Poziomy sygnałów, VIO oraz podciąganie I2C Hub8: 3,3 V. Nie łączyć wyjść regulatorów 3,3 V Teensy i XIAO. Ustalić źródło VIO i masę odniesienia w schemacie.
- Teensy → sześć niezależnych par STEP/DIR; EN wspólne, bezpiecznie nieaktywne podczas resetu. Hub8 na I2C, komunikacja XIAO niezależna. Nie przypisano numerów GPIO ani elementów ustalających stany.
- UART V1.3, jeśli potrzebny: dwie osobne grupy po trzy sterowniki, adresy 0/1/2 w każdej. TMC2209 ma tylko cztery adresy; sześciu unikalnych na jednej magistrali nie uzyskamy. U1 pin14 PDN_UART jest połączony z siecią UART_RX i pinem opisanym RX. Pin TX dochodzi do RX przez **R10 oznaczony NC**, zatem fabrycznie nie jest dołączony do tego węzła. W proponowanej topologii używać RX modułów, TX pozostawić niepodłączone; nie trzeba mostkować R10. R9 jest potencjometrem 20 kΩ dla VREF, nie elementem UART. Każdy moduł ma R1 = 20 kΩ podciągający EN do VIO; wspólne EN musi zachować bezpieczny stan nieaktywny przy resecie.
- Przez szczelinę przechodzą wyłącznie sterowanie i zasilanie; **żadne z 24 połączeń fazowych silników nie przecina szczeliny**. To koncepcja, nie pełny rozkład wiązek.
- Każda część wymaga własnych dystansów; przewody i lut nie zastępują połączenia mechanicznego. Posiadany wentylator musi chłodzić cały rząd na obu płytkach. Instrukcja V1.3 z 26 stycznia 2024 r., strona 4, podaje **„Peak Current Support: 2A”**, a strona 8 wymaga aktywnego chłodzenia powyżej 1,2 A. Nie nazywa tego 2 A RMS ani gwarantowanym prądem ciągłym. Ogólna strona BTT podająca 2 A RMS jest niespójna z instrukcją tego wariantu; dla V1.3 zachowujemy ostrożniejsze sformułowanie **2 A peak, prąd ciągły wymaga weryfikacji chłodzenia**. Sprawdzić radiatory, obudowę, temperatury i jednoczesne obciążenie; nie ustalono nastawy prądu firmware.

Przykład funkcjonalny jednej magistrali single-wire, zgodny z ideą ADI Fig. 4.1/4.2 (sygnały 3,3 V, wspólna masa):

```text
Teensy TXn -- 1k --+-- RX M1 (adres 0)
Teensy RXn --------+-- RX M2 (adres 1)
                   +-- RX M3 (adres 2)
```

Dla M4–M6 drugi niezależny UART, ponownie adresy 0/1/2. TX modułów pozostaje wolne; R10 pozostaje NC. To przykład topologii, nie końcowy przydział GPIO ani pełny schemat okablowania; sprawdzić stany, adresowanie i masę przed uruchomieniem.

## Źródła pinoutu i ograniczenia

- [BIGTREETECH V1.3 — instrukcja](https://raw.githubusercontent.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/master/TMC2209/V1.3/manual/BIGTREETECH%20TMC2209%20V1.3%20User%20Manual.pdf)
- [BIGTREETECH V1.3 — katalog schematów i rysunków](https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/tree/master/TMC2209/V1.3/Schematic)
- [V1.3 — zdjęcie pinoutu TOP](https://raw.githubusercontent.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/master/TMC2209/V1.3/Schematic/TMC2209-V1.3-Pin.jpg)
- [V1.3 — schemat](https://raw.githubusercontent.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/master/TMC2209/V1.3/Schematic/TMC2209%20V1.3-SCH.pdf)
- [V1.3 — wymiary](https://raw.githubusercontent.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/master/TMC2209/V1.3/Schematic/TMC2209%20V1.3-SIZE.pdf)
- [Analog Devices — TMC2209, rewizja 1.09, Fig. 4.1/4.2](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf)
- [BTT — TMC2209](https://neo.bttwiki.com/en/docs/accessories-docs/tmc-driver/tmc-2209/)
- [Watterott — warianty pinów SilentStepStick](https://learn.watterott.com/silentstepstick/pinconfig/)

Współrzędne otworów mieszczą się na lokalnej siatce 2,54 mm lewego i prawego banku, ale rzeczywisty początek siatki formatki pozostaje nieznany. Przymierzyć wszystkie części przed cięciem/wierceniem. Skrypt sprawdza bryły w obu częściach (włącznie z R7), kolizje z kondensatorami, 24 unikalne sieci fazowe i ich przypisania przy obrocie, lokalny raster, brak faz przez szczelinę i marginesy. Nie sprawdza obciążalności, termiki, izolacji, padów, wymiarów rzeczywistych części, połączeń wewnętrznych płytki ani pełnej zgodności elektrycznej.
