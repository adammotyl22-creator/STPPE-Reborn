# Plan rozwoju STPPE Reborn

[English](ROADMAP.md) · **Polski**

[← Opis projektu](../README.pl.md) · [Pierwsze kroki](GETTING_STARTED.pl.md) · [Specyfikacja](STPPE_SYSTEM_SPECIFICATION.pl.md)

Ten plan oddziela funkcje dostępne obecnie od prac i pomiarów potrzebnych do odtworzenia fizycznego systemu. Etapy wyznaczają zamierzoną kolejność, a nie daty wydań ani deklarację zakończonej weryfikacji sprzętu.

## Dostępne obecnie

- Control Studio ze sterowaniem kartezjańskim i osiowym, wizualizacją CAD, podglądem celu i sekwencerem punktów trajektorii.
- Wirtualny sterownik, telemetria i konsola poleceń do poznawania aplikacji bez sprzętu.
- Kod komunikacji przez USB Web Serial oraz zgodny zewnętrzny mostek WebSocket.
- Źródła firmware Teensy 4.1, siatki modelu robota i opis URDF.
- Główny zestaw JavaScript z 618 sprawdzeniami (25 zestawów testowych) zakończonymi pełnym powodzeniem oraz zaliczony podstawowy test serwera Pythona; wyniki lokalne.

Wyniki testów oprogramowania nie potwierdzają dokładności fizycznego robota, powtarzalności kompilacji ani niezawodności całej instalacji bezprzewodowej.

## 1. Dostosowanie firmware do konstrukcji

**Cel:** udokumentowana konfiguracja firmware dla sprzętu bez enkoderów, opisanego w specyfikacji.

- [ ] Wdrożyć i sprawdzić jawną konfigurację z wyłączoną obsługą enkoderów.
- [ ] Wdrożyć i sprawdzić planowane ręczne bazowanie przy uruchomieniu oraz pozycję odniesienia.
- [ ] Uzgodnić zachowanie `HOME`, `HOME MANUAL`, telemetrii i poleceń ruchu między firmware, aplikacją a specyfikacją.
- [ ] Zapisać wersje narzędzi Teensy i bibliotek oraz dokładną procedurę kompilacji i wgrywania.

**Kryterium ukończenia:** świeża kopia repozytorium kompiluje się w opisanym środowisku, docelowa konfiguracja nie zależy od nieobecnych czujników, a pozycja odniesienia w dokumentacji zgadza się z zachowaniem firmware.

## 2. Pomiary na stanowisku

**Cel:** opublikowane wyniki pozwalające ocenić możliwości ramienia.

- [ ] Sprawdzić kierunki J1–J6, zakresy ruchu, przełożenia i działanie chwytaka.
- [ ] Opisać zatrzymywanie, ponowne włączanie i odzyskiwanie odniesienia pozycji.
- [ ] Zmierzyć błąd pozycjonowania końcówki i powtarzalność w reprezentatywnych punktach obszaru roboczego.
- [ ] Opublikować metodę pomiaru, konfigurację narzędzia i obciążenia, wyniki oraz ograniczenia.
- [ ] Nagrać krótką demonstrację fizycznego robota wraz z odpowiadającym jej widokiem Control Studio.

**Kryterium ukończenia:** repozytorium zawiera raport umożliwiający powtórzenie pomiarów oraz rzeczywiste wyniki. Wartość ≤ 0,5 mm pozostaje celem, dopóki nie potwierdzą jej pomiary.

## 3. Możliwość odtworzenia projektu

**Cel:** nowa osoba może zrozumieć, zbudować i sprawdzić projekt na podstawie repozytorium.

- [ ] Opublikować listę części z ilościami, informacjami o zakupie i kosztami z podaną datą, w odniesieniu do budżetu 2000 PLN.
- [ ] Dodać schematy połączeń, zdjęcia montażu i instrukcje kalibracji.
- [ ] Automatycznie uruchamiać testy przy pull requestach i opisać ich zakres.
- [ ] Uporządkować historyczne warianty interfejsu wokół głównej aplikacji `WebControl/`.
- [ ] Określić licencję własnych materiałów i zasady wykorzystania zasobów zewnętrznych przed przedstawieniem projektu jako dystrybucji do ponownego użycia.

**Kryterium ukończenia:** osoba spoza pierwotnego środowiska projektowego potrafi wykonać instrukcję, odtworzyć wyniki testów oprogramowania i wskazać wszystkie wymagane elementy sprzętowe.

## 4. Rozbudowa teleoperacji

**Cel:** dedykowany ręczny panel sterowania (Teach Pendant) i udokumentowana opcja bezprzewodowa.

- [ ] Określić architekturę sprzętową konsoli: manipulatory drążkowe (joysticki), zadajnik kółkowy / impulsator MPG oraz trójpozycyjny przycisk zezwolenia na ruch (Deadman / Enabling switch).
- [ ] Zaimplementować obsługę protokołu konsoli, zoptymalizowany firmware oraz konfigurację mostka ESP32.
- [ ] Opisać priorytety sterowania i mechanizm arbitrażu, gdy komputer stacjonarny i konsola ręczna są połączone jednocześnie.
- [ ] Zmierzyć opóźnienia transmisji (jitter/latency) i zweryfikować procedury bezpieczeństwa po utracie oraz przywróceniu łączności.

**Kryterium ukończenia:** demonstracje połączenia przewodowego i bezprzewodowego obejmują konfigurację, pomiary czasu reakcji i sprawdzone zachowanie przy rozłączeniu.

## Pomóż zrealizować etap

Utwórz zgłoszenie opisujące etap, konkretny rezultat, który chcesz dostarczyć, i sposób jego sprawdzenia. Pomagają również poprawki dokumentacji, zgłoszenia błędów symulatora, ulepszenia CAD i wyniki pomiarów. Zobacz [zasady współpracy](../CONTRIBUTING.pl.md).
