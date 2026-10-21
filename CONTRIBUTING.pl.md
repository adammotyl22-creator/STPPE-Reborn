# Współpraca przy STPPE Reborn

[English](CONTRIBUTING.md) · **Polski**

[Opis projektu](README.pl.md) · [Pierwsze kroki](docs/GETTING_STARTED.pl.md) · [Plan rozwoju](docs/ROADMAP.pl.md)

Możesz rozwijać aplikację sterującą, firmware, modele CAD, dokumentację lub bazę wyników pomiarów konstrukcji. Jeśli nie masz sprzętu, zacznij od symulatora.

## Zgłoszenie problemu

Użyj [szablonu zgłoszenia błędu](https://github.com/adammotyl22-creator/STPPE-Reborn/issues/new?template=bug_report_pl.md). Podaj kroki odtworzenia, zachowanie oczekiwane i rzeczywiste, system operacyjny, przeglądarkę oraz rodzaj połączenia: symulacja, USB lub Wi-Fi. W przypadku sprzętu podaj także płytkę, wersję firmware i istotne informacje o okablowaniu lub kalibracji.

Przydatne są logi konsoli i zrzuty ekranu. Przed ich udostępnieniem usuń dane logowania, hasła Wi-Fi i dane osobowe.

## Propozycja ulepszenia

Użyj [szablonu propozycji funkcji](https://github.com/adammotyl22-creator/STPPE-Reborn/issues/new?template=feature_request_pl.md). Opisz, jakie zadanie jest obecnie trudne, jakie zachowanie proponujesz i jak sprawdzić, że zmiana spełnia cel. W razie potrzeby wskaż odpowiedni [etap planu rozwoju](docs/ROADMAP.pl.md).

## Wprowadzanie zmian

1. Utwórz fork i sklonuj repozytorium, a następnie załóż gałąź dla jednego konkretnego ulepszenia.
2. Uruchom `python WebControl/launch.py` z głównego katalogu repozytorium i odtwórz interesujące Cię zachowanie.
3. Wprowadź zmianę w głównych katalogach `WebControl/`, `Firmware/` lub `docs/`. Nie dodawaj wygenerowanych logów, wyników kompilacji ani lokalnych ustawień edytora.
4. Wykonaj sprawdzenia odpowiednie do zmiany i zapisz ich wyniki.
5. Otwórz pull request opisujący problem, działanie po zmianie i sposób weryfikacji. Dla zmian wizualnych dodaj zrzuty przed zmianą i po niej.

## Weryfikacja

Podstawowe sprawdzenia uruchamiane z głównego katalogu repozytorium:

```bash
node WebControl/tests/run_tests_node.js
python WebControl/tests/test_server.py
```

Główny zestaw JavaScript obejmuje logikę aplikacji i zgodność elementów interfejsu. Test Pythona uruchamia tymczasowy serwer lokalny i pobiera pliki aplikacji. Żaden z nich nie wymaga robota ani instalowania dodatkowych pakietów.

Przy zmianach dokumentacji sprawdź linki względne, przykłady poleceń i zgodność opisów ze stanem projektu. Przy zmianach interfejsu przejdź zmieniony scenariusz w symulatorze i wykonaj odpowiednie testy. Dla firmware osobno opisz środowisko kompilacji, wynik kompilacji i ewentualne próby na stanowisku; wynik symulatora nie jest weryfikacją sprzętu.

Dodawaj testy regresji wykazujące poprawiany błąd lub zmieniane zachowanie. Dodatkowe testy przeglądarkowe mogą wymagać Playwright i plików przeglądarek; opisz te zależności, jeśli Twoja zmiana z nich korzysta.

## Zasady projektu

- Zachowuj zgodność frontendu z natywnymi modułami ES i uruchamianiem bez budowania aplikacji.
- Zachowuj interfejsy kontrolerów oraz identyfikatory DOM i atrybuty danych, chyba że zmiana obejmuje też korzystający z nich kod i testy.
- Aktualizuj polskie i angielskie tłumaczenia, gdy zmieniasz lokalizowane etykiety interfejsu.
- Określaj, czy informacja o sprzęcie jest założeniem projektowym, zachowaniem wynikającym z kodu czy wynikiem pomiaru.
- Utrzymuj zgodność specyfikacji i instrukcji uruchomienia z implementacją.
