# Koncepcja rozmieszczenia PCB robota

Rekomendowany układ: sześć TMC2209 po prawej, w dwóch rzędach po trzy. Wyjścia silników na dolnej krawędzi; USB Teensy skierowane w lewo. SVG i PNG przedstawiają jednakowo skalowane korpusy, bez tras elektrycznych. To koncepcja mechaniczna, nie plik PCB do produkcji.

Współrzędne w mm: (0,0) to lewy górny narożnik głównego prostokąta 104,5 × 121; X w prawo, Y w dół. Występ z lewej jest prowizoryczny, wymaga pomiaru. Nie dodano nieznanych otworów montażowych.

| Rodzina / element | X | Y | Szerokość | Wysokość |
|---|---:|---:|---:|---:|
| Teensy 4.1 | −23 | 76 | 60,96 | 17,78 |
| KAmod I2C Hub8 | 7 | 7 | 31 | 18,3 |
| Seeed XIAO ESP32C3 | 7 | 38 | 21 | 17,8 |
| LM2596, moduł orientacyjny | 57 | 7 | 43 | 21 |
| StepStick TMC2209, wspólna bryła | poniżej | poniżej | 15,24 | 20,32 |
| M1 / M2 / M3 | 44 / 64,32 / 84,64 | 52 | 15,24 | 20,32 |
| M4 / M5 / M6 | 44 / 64,32 / 84,64 | 84 | 15,24 | 20,32 |
| Złącze silnika M1 | 3 | 109 | 15 | 10 |
| Złącze silnika M2 | 20 | 109 | 15 | 10 |
| Złącze silnika M3 | 37 | 109 | 15 | 10 |
| Złącze silnika M4 | 54 | 109 | 15 | 10 |
| Złącze silnika M5 | 71 | 109 | 15 | 10 |
| Złącze silnika M6 | 88 | 109 | 15 | 10 |
| Wejście 24 V — rezerwa | 83 | 0 | 16 | 6 |
| Zabezpieczenia 24 V — rezerwa | 57 | 31 | 43 | 8 |

Sześć rezerw kondensatorów C_VM ma średnicę **8 mm**; środki: X = 51,62 / 71,94 / 92,26 oraz Y = 46 / 78. To obwiednie miejsca, bez doboru pojemności, napięcia i BOM. Minimalna odległość obwiedni od korpusu wynosi 1,68 mm. Symbol radiatora ma 9 × 9 mm i nie określa wysokości ani wymaganego modelu.

Odstępy korpusów: kolumny TMC 5,08 mm; Teensy–TMC 6,04 mm; dolny TMC–złącze 4,68 mm. Ostatnie złącze kończy się na X = 103 mm, przy krawędzi 104,5 mm. Są to odstępy mechanicznych brył, bez oceny odstępów elektrycznych, pól lutowniczych ani dostępu narzędzi.

## Założenia i dalszy projekt

- Wymiary Teensy i XIAO odpowiadają dokumentacji producentów; KAmod 31 × 18,3 mm według danych producenta. Kontur występu, wariant StepStick 15,24 × 20,32 mm, moduł LM2596 ok. 43 × 21 mm oraz wszystkie obwiednie złączy trzeba potwierdzić pomiarem konkretnych części. Nie założono zgodności z określonym złączem.
- Górny rząd sterowników oznacza dłuższe trasy do dolnych wyjść silników. W projekcie właściwym trzeba rozwiązać ich prowadzenie, powroty prądu i ewentualną zmianę kolejności wyprowadzeń. Rysunek nie symuluje tras dużego prądu.
- Preferowane cztery warstwy, po obliczeniu prądów, grubości miedzi, szerokości ścieżek/pól i przelotek oraz sprawdzeniu termiki. Sama liczba warstw nie zapewnia obciążalności.
- **VM sterowników zasilać bezpośrednio z chronionego wejścia 24 V. LM2596 służy do zasilania logiki 5 V**, nie do zasilania VM. Dobór zabezpieczeń, filtracji, kondensatorów oraz powrotów masy pozostaje do wykonania.
- Logika sygnałów: **3,3 V**. Przy równoczesnym USB i zewnętrznym 5 V Teensy należy zapewnić właściwe rozdzielenie VIN/VUSB zgodnie z dokumentacją PJRC.
- XIAO również wymaga ochrony przed zasilaniem zwrotnym przy USB i zewnętrznym 5 V. Seeed zaleca diodę między zewnętrznym źródłem a pinem 5 V (anoda do źródła, katoda do XIAO); rozwiązanie trzeba uwzględnić w schemacie.
- Prostokąty StepStick nie potwierdzają footprintu, rozstawu listew ani orientacji pinów. Projekt musi odpowiadać dokumentacji wybranego wariantu modułu.
- XIAO ESP32C3 korzysta z anteny zewnętrznej na przewodzie; umieścić antenę poza metalową obudową i sprawdzić rzeczywisty zasięg. Symbol przewodu nie jest polem zakazu dla anteny PCB.
- TMC2209: wartości **2 A RMS / 2,8 A peak odnoszą się do maksymalnych możliwości układu scalonego**, nie stanowią gwarancji ciągłej obciążalności modułu StepStick. Trzeba zweryfikować konkretną płytkę, nastawę prądu, radiator, wentylator, temperaturę i jednoczesne obciążenie sześciu osi. Strzałki nawiewu nie są footprintem wentylatora ani gwarancją chłodzenia.
- Bryły mieszczą się w przyjętym obrysie, poza celowo wystającą obudową USB. Strefa wtyku USB i strzałki silników są ilustracyjne. Nie sprawdzono wysokości, dostępu do wtyków, tolerancji ani kolizji obudowy.

## Źródła

- [PJRC — Teensy 4.1](https://www.pjrc.com/store/teensy41.html)
- [Seeed — XIAO ESP32C3](https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/)
- [Kamami — KAmod I2C Hub8](https://wiki.kamamilabs.com/index.php?title=KAmod_I2C_Hub8_%28PL%29)
- [BIGTREETECH — TMC2209 V1.3, dokumentacja wariantu](https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/tree/master/TMC2209/V1.3)
- [Analog Devices — TMC2209](https://www.analog.com/en/products/tmc2209.html)

Renderowanie: `draw_layout.py`, Python + Pillow, wspólne współrzędne dla SVG i PNG, czcionka Segoe UI. Skrypt sprawdza kolizje prostokątów i obwiedni C_VM, liczbę elementów oraz kluczowe odstępy. Diagram jest statyczny, 1700 × 1450 px, 7 px/mm dla brył.
