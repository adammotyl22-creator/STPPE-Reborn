# Pierwsze kroki ze STPPE Reborn

[English](GETTING_STARTED.md) · **Polski**

[← Opis projektu](../README.pl.md) · [Specyfikacja](STPPE_SYSTEM_SPECIFICATION.pl.md) · [Plan rozwoju](ROADMAP.pl.md)

Zacznij od wirtualnego sterownika, aby poznać Control Studio. Korzysta z tego samego interfejsu co połączenie ze sprzętem i nie wymaga Teensy, silników ani portu szeregowego.

## Wymagania

| Zadanie | Wymagania |
| :--- | :--- |
| Poznanie aplikacji i symulatora | Python 3 oraz przeglądarka z WebGL |
| Główne testy automatyczne | Node.js; główny zestaw nie wymaga instalowania pakietów npm |
| Połączenie USB | Przeglądarka udostępniająca `navigator.serial`, kabel USB do transmisji danych i skonfigurowany sterownik |
| Połączenie Wi-Fi | Osobno skonfigurowany, zgodny mostek między WebSocket a sterownikiem |

Frontend korzysta z natywnych modułów ES i dołączonych plików Three.js. Skrypt Pythona używa biblioteki standardowej. Uruchamiaj aplikację przez serwer, zamiast otwierać `index.html` bezpośrednio z dysku.

## Uruchomienie Control Studio

Sklonuj repozytorium lub pobierz i rozpakuj je przez **Code → Download ZIP** na GitHubie:

```bash
git clone https://github.com/adammotyl22-creator/STPPE-Reborn.git
cd STPPE-Reborn
python WebControl/launch.py
```

Jeżeli Python uruchamia się u Ciebie poleceniem `py` w Windows albo `python3` w macOS/Linux, użyj go w ostatnim wierszu. Dostępne są również skrypty dla Windows: [`start_webcontrol.bat`](../WebControl/start_webcontrol.bat) i [`start_webcontrol.ps1`](../WebControl/start_webcontrol.ps1).

Skrypt szuka wolnego portu, zaczynając od 8000, wypisuje **Local URL** i otwiera domyślną przeglądarkę. Jeśli przeglądarka się nie otworzy, skopiuj do niej podany adres. Terminal musi pozostać uruchomiony; `Ctrl+C` zatrzymuje serwer.

## Pierwszy ruch w symulatorze

1. Kliknij wskaźnik **Rozłączono (Disconnected)** w nagłówku, aby otworzyć menu połączenia. Możesz tam również przełączyć język aplikacji na polski.
2. Zaznacz opcję **Symulacja (Mock)**, a następnie kliknij **Połącz (Connect)**. Terminal szeregowy potwierdzi nawiązanie sesji z wirtualnym kontrolerem Teensy, a wykresy telemetrii zaczną odbierać strumień danych w czasie rzeczywistym.
3. W lewym panelu sterowania ruchem wybierz zakładkę **Pozycjonowanie TCP (IK)**. Zmodyfikuj współrzędne docelowe suwakami lub przyciskami impulsowymi (Jog) i obejrzyj bursztynowy cień (Ghost) ramienia oraz wskaźnik poprawności kinematycznej.
4. Kliknij **Wykonaj ruch PTP**, aby zasymulować płynny dojazd ramienia do pozycji docelowej i obserwuj przebiegi uchybu oraz kątów w telemetrii.
5. Przejdź do zakładki **Sterowanie osiami (FK)**, aby przetestować bezpośrednie zadawanie kątów J1–J6, lub do **Sekwencer**, aby poznać edycję klatek kluczowych i interpolację trajektorii.

Ustawienia **PODGLĄD (CIEŃ)** oraz **ŚLEDZENIE NA ŻYWO / Napęd bezpośredni** zmieniają sposób wysyłania zmian pozycji docelowej. Podgląd pozwala sprawdzić cel przed jawnym wysłaniem polecenia; nie oznacza, że wszystkie przyciski aplikacji są nieaktywne. Podczas tej instrukcji pozostaw połączenie w trybie symulacji.

[Zrzut interfejsu](assets/webcontrol-dashboard.png) przedstawia układ paneli w wersji angielskiej. Nazwy przycisków zależą od wybranego języka.

## Podłączanie fizycznego robota

Repozytorium zawiera szkic [`IK_Control.ino`](../Firmware/Teensy_4.1/IK_Control/IK_Control.ino), deklarujący wersję firmware `v3.2.0`. Docelowy układ obejmuje Teensy 4.1, sześć sterowników TMC2209 i chwytak napędzany serwem. Przypisanie pinów oraz wymiary mechaniczne podano w [specyfikacji systemu](STPPE_SYSTEM_SPECIFICATION.pl.md).

Między konfiguracją sprzętu opisaną w specyfikacji a kodem szkicu istnieją różnice:

| Obszar | Obecne zachowanie / pozostała praca |
| :--- | :--- |
| Enkodery | Kod obsługi enkoderów istnieje, ale `ENABLE_ENCODERS` nie jest zdefiniowane. Tryb pracy bez enkoderów wymaga wdrożenia i weryfikacji. |
| Bazowanie | `MANUAL_HOMING` nie jest zdefiniowane. Samo `HOME` uruchamia procedurę bazowania na krańcówkach. |
| Ręczne ustawienie odniesienia | Polecenie `HOME MANUAL` istnieje, lecz jego obsługa przyjmuje zerowe kąty jako pozycję odniesienia. Różni się to od proponowanej pozycji startowej `[0, 25, 140, 0, 0, 0]`. |
| Powtarzalna kompilacja | Szkic korzysta z bibliotek zewnętrznych, ale ta instrukcja nie potwierdza zweryfikowanego środowiska ani kompilacji firmware. Ustalenie wersji bibliotek i zapis poprawnej procedury kompilacji są częścią planu rozwoju. |

Dla sterownika już skonfigurowanego i sprawdzonego z Twoją konstrukcją: otwórz lokalny adres Control Studio, rozłącz symulator, odznacz **Symulacja (Mock)**, wybierz **USB** i kliknij **Połącz**, aby wskazać port sterownika. Przed zadaniem ruchu sprawdź połączenie i telemetrię. Rozwiąż opisane wyżej kwestie bazowania i enkoderów, zanim potraktujesz specyfikację jako instrukcję uruchamiania sprzętu.

## Dostęp z innego urządzenia

W głównym katalogu repozytorium:

```bash
python WebControl/launch.py --lan
```

Otwórz wypisany **Phone URL** na urządzeniu w tej samej zaufanej sieci. Jeśli Zapora Windows wyświetli pytanie, zezwól Pythonowi na dostęp w sieci prywatnej. Bez `--lan` serwer jest dostępny tylko na komputerze, na którym został uruchomiony.

Serwer LAN udostępnia pliki interfejsu, a nie połączenie USB komputera. Sterowanie przez USB zależy od przeglądarki i kontekstu bezpieczeństwa urządzenia otwierającego stronę. Tryb Wi-Fi wymaga działającego, zgodnego mostka; samo wybranie Wi-Fi go nie tworzy.

## Rozwiązywanie problemów

| Objaw | Co sprawdzić |
| :--- | :--- |
| Polecenie `python` nie jest rozpoznawane | Spróbuj `py` w Windows lub `python3` w macOS/Linux; sprawdź instalację Pythona 3. |
| Przeglądarka się nie otworzyła | Skopiuj dokładny adres **Local URL** wypisany przez skrypt. |
| Pusta strona lub błędy ładowania modułów | Użyj adresu HTTP zamiast `file://`; sprawdź konsolę przeglądarki i komunikaty serwera. |
| Brak wyboru USB / brak obsługi Web Serial | Do poznania aplikacji użyj symulatora. Dla USB sprawdź obsługę `navigator.serial` i użyj lokalnego adresu serwera. |
| Port szeregowy jest niedostępny | Sprawdź kabel USB do transmisji danych i zamknij inne programy korzystające z portu, np. monitor szeregowy. |
| Brak telemetrii symulatora | Zaznacz **Symulacja (Mock)** przed kliknięciem Połącz. Rozłącz się przed zmianą trybu połączenia. |
| Telefon nie otwiera strony | Uruchom serwer z `--lan`, użyj adresu sieciowego i sprawdź zaporę oraz połączenie Wi-Fi. |
| Połączenie Wi-Fi nie działa | Sprawdź działanie osobnego mostka i zgodność jego adresu WebSocket z ustawieniami aplikacji. |

Jeżeli problem pozostaje, [utwórz zgłoszenie](https://github.com/adammotyl22-creator/STPPE-Reborn/issues/new/choose) z krokami odtworzenia, wersją systemu i przeglądarki, trybem połączenia oraz odpowiednim fragmentem logu konsoli.

## Lokalna weryfikacja oprogramowania

Uruchom z głównego katalogu repozytorium:

```bash
node WebControl/tests/run_tests_node.js
python WebControl/tests/test_server.py
```

Główny zestaw Node.js i test serwera nie wymagają instalowania pakietów. Pozostałe pliki w `WebControl/tests/` mogą mieć własne zależności; przykładowo test przeglądarkowy E2E importuje Playwright. Sposób opisywania wyników znajdziesz w [zasadach współpracy](../CONTRIBUTING.pl.md).
