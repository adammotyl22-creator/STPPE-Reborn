# Plan cięcia dwóch płytek uniwersalnych 120 × 80 mm

Potrzebne są **co najmniej dwie płytki**. Jeden format 120 × 80 mm nie pokryje pola 104,5 × 121 mm z występem. Bok 120 mm ustawiono pionowo. Rysunek pokazuje nominalny podział, a nie szablon do bezpośredniego cięcia; najpierw wykonać model kartonowy i przymiarkę części.

## Nominalny podział

Współrzędne odnoszą się do wcześniejszej koncepcji: (0,0) w lewym górnym narożniku głównego pola; X w prawo, Y w dół.

| Część | Położenie | Zachowany format | Usuwany materiał |
|---|---|---|---|
| A, lewa L | X −23,5…40; Y 0,5…120,5 | obwiednia 63,5 × 120 mm | pas 16,5 × 120 mm oraz górne lewe wcięcie 25,4 × 71,12 mm; zaokrąglenie dolnego lewego narożnika R7 |
| B, prawa | X 42…102,96; Y 0,5…120,5 | 60,96 × 120 mm, w przybliżeniu 61 × 120 mm | pas 19,04 × 120 mm |

Szczelina między częściami ma nominalnie **2 mm**. Po wycięciu wcięcia górna lewa krawędź części A leży na X = 1,9; dolna krawędź wcięcia na Y = 71,62. Środek zaokrąglenia części A: (−16,5; 113,5), promień 7 mm.

Wymiary odpadu są teoretyczne i **nie obejmują rzazu**. Ciąć od strony odpadu, pozostawić materiał do dopasowania pilnikiem. Rzeczywiste wymiary płytek, położenie pierwszego otworu, tolerancje i rodzaj laminatu nie są znane.

Raster 2,54 mm jest typowy dla wielu płytek, ale trzeba zmierzyć konkretny egzemplarz. Wcięcie 25,4 × 71,12 mm to nominalne odniesienie 10 × 28 kroków po 2,54 mm, nie potwierdzenie dopasowania do wszystkich płytek uniwersalnych. Globalne współrzędne modułów nie pokrywają się w całości z rastrem: układać rzeczywiste moduły na otworach, sprawdzić listwy, orientacje pinów i dopiero ustalić linię cięcia.

## Moduły i złącza

Położenia brył modułów pozostają bez zmian względem koncepcji: Teensy (−23; 76), Hub8 (7; 7), XIAO (7; 38), LM2596 (57; 7); TMC2209 X = 44 / 64,32 / 84,64, Y = 52 / 84. Wymiary i założenia modułów opisano w `rozmieszczenie.md`.

Sześć rezerw złączy 15 × 10 mm **przeniesiono**, aby nie przecinały szczeliny. Wszystkie mają Y = 109 i wyjście przewodów w dół:

| Część A | X | Część B | X |
|---|---:|---|---:|
| M1 | −17 | M4 | 45 |
| M2 | 0 | M5 | 64 |
| M3 | 17 | M6 | 83 |

Sterowniki wszystkich sześciu osi są na części B. Do trzech lewych wyjść potrzeba **12 przewodów fazowych** przechodzących między częściami. Prowadzić je w izolacji w wolnej przestrzeni nad lub pod płytkami. Szczelina 2 mm nie jest zweryfikowanym kanałem na wiązkę: sprawdzić ułożenie na sucho, promienie gięcia, wysokość obudowy i dostęp do złączy. Rysunek nie pokazuje rzeczywistych tras.

Bryła Teensy mieści się zaledwie **0,5 mm** od lewej krawędzi płytki. Obudowa USB celowo wystaje w lewo; trzeba sprawdzić wtyk, przewód, listwy oraz tolerancje. Wszystkie moduły i złącza mają własną część pod całą narysowaną bryłą, także przy zaokrągleniu. To nie jest walidacja padów, pinów, wysokości, podparcia ani obudowy. Nie dodano wymyślonych otworów montażowych.

## Kolejność pracy

1. Sprawdzić rzeczywisty format, podziałkę i elektryczne połączenia otworów: pojedyncze izolowane pady czy połączone paski. Przygotować kartonowy model i przymiarkę wszystkich części oraz wiązki.
2. Odciąć najpierw długi pas szerokości nominalnej 63,5 lub 60,96 mm. Potem wyciąć wcięcie L, bez wyłamywania narożnika. Ciąć po stronie odpadu drobnozębną piłką ręczną przy zamocowanej i podpartej płytce; wykończyć pilnikiem i usunąć zadziory. Wymiary końcowe ustalić po przymiarce.
3. Nacinanie i łamanie można rozważyć wyłącznie jako alternatywę dla cięcia prostego, po sprawdzeniu podłoża. Nie stosować do narożnika L. Jeśli to FR4, ograniczyć pył, stosować ochronę oczu, maskę i wentylację; oczyścić stanowisko i płytkę przed elektroniką.
4. Każdą część osadzić na własnych dystansach; połączenie mechaniczne zapewnia obudowa lub osobny wspornik. Lut i przewody nie są łącznikiem mechanicznym płytek.
5. Rozprowadzenie mocy dla sześciu sterowników wykonać dedykowanymi przewodami dobranymi do rzeczywistego obciążenia, zamiast łańcuchów cienkich padów. Zachować zasady VM 24 V, logiki i ochrony z `rozmieszczenie.md`; rysunek nie określa dopuszczalnego prądu.

## Źródła i sprawdzenie

- [BusBoard — wskazówki dotyczące pracy z płytkami i cięcia](https://www.busboard.com/support)
- [BusBoard ST3U — przykład rastra 0,1 cala / 2,54 mm](https://www.busboard.com/ST3U)

Źródła opisują przykłady produktów i metod, nie potwierdzają materiału ani połączeń użytkowanych płytek. Skrypt `draw_cutting_plan.py` generuje SVG i PNG z jednej geometrii (Python/Pillow, Segoe UI) i sprawdza bryły na pojedynczych częściach, zaokrąglenie, brak wzajemnych kolizji, sześć sterowników, sześć złączy w podziale 3+3, granice formatek, szczelinę i margines Teensy. Status konsoli używa ASCII, dokumenty UTF-8.
