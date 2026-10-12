# Dokumentacja Techniczna - System Teleoperacyjny do Precyzyjnych Prac Serwisowych

## 1. Przegląd Systemu

### 1.1 Opis Projektu
System teleoperacyjny przeznaczony do precyzyjnych prac serwisowych w elektronice, stworzony w ramach konkursu **Inżynierowie Przyszłości 2025/2026**. Konstrukcja opiera się na manipulatorze wykonawczym strowanym przez interfejs typu master-slave, co pozwala na intuicyjne przenoszenie ruchów operatora na narzędzie robocze.

### 1.2 Główne Cele
- Zwiększona precyzja dzięki zastosowaniu silników krokowych z przekładniami
- Eliminacja drgań i płynne odwzorowanie ruchu
- Poprawa ergonomii stanowiska pracy
- Możliwość pracy z różnymi narzędziami (chwytak, lutownica, kamera)

## 2. Specyfikacja Techniczna

### 2.1 Konfiguracja Napędów (Model URDF stppe_zlozenie_nowe)
- **6 Stopni Swobody (6-DOF) + Chwytak:**
  - **Oś J1 (Podstawa - Yaw):** Silnik krokowy NEMA 17 (TMC2209), przełożenie 8.33:1, zakres [-90°, +90°]
  - **Oś J2 (Ramię główne - Shoulder Pitch):** Silnik krokowy NEMA 17 (TMC2209) z przekładnią planetarną 20:1, zakres [-65°, +115°]
  - **Oś J3 (Przedramię - Elbow Pitch):** Silnik krokowy NEMA 17 (TMC2209) z przekładnią planetarną 20:1, zakres [-35°, +170°]
  - **Oś J4 (Obrót przedramienia - Forearm Roll):** Silnik krokowy (TMC2209), zakres [-90°, +90°]
  - **Oś J5 (Pochylenie nadgarstka - Wrist Pitch):** Silnik krokowy (TMC2209), zakres [-90°, +90°]
  - **Oś J6 (Flansza narzędziowa - Tool Roll):** Silnik krokowy (TMC2209), zakres [-180°, +180°]
  - **Chwytak narzędziowy (Gripper):** Serwomechanizm PWM (Pin 21 Teensy 4.1), zakres [0°, 180°]

### 2.2 Wymiary Kinematyczne (URDF)
- **BASE_H (Wysokość podstawy do osi J2):** 143.896 mm
- **L1 (Długość ramienia J2–J3):** 185.845 mm
- **L2 (Długość przedramienia J3–J5):** 225.478 mm
- **L3 (Długość nadgarstka J5–J6):** 36.105 mm
- **TOOL_TOTAL_LEN (Efektywny wysięg narzędzia):** 31.530 mm

### 2.3 Precyzja i Sterowanie
- **Dokładność:** Zależna od mikrokroku i przekładni (teoretycznie <0.1 mm)
- **Płynność:** Wysoka dzięki sterowaniu mikrokrokowemu
- **Moment obrotowy:** Znacznie zwiększony przez przekładnie 20:1

## 3. Komponenty Mechaniczne

### 3.1 Struktura
Konstrukcja hybrydowa wykorzystująca elementy drukowane 3D oraz części metalowe.
- **Napęd:** NEMA 17 + Przekładnia 20:1 zapewniające sztywność i moment trzymający
- **Łożyskowanie:** Dedykowane łożyska (zgodnie z projektem 1.5)

### 3.2 Materiały
- **Korpus:** PLA / PETG (elementy drukowane)
- **Elementy mechaniczne:** Silniki, przekładnie, łożyska metalowe

## 4. System Elektroniczny

### 4.1 Mikrokontrolery
- **Master (Kontroler ręczny):** ESP32-C3 (kompaktowy, obsługa HID/Serial)
- **Slave (Robot):** Teensy 4.1 (wysoka wydajność obliczeniowa do sterowania silnikami krokowymi)

### 4.2 Sterowanie Napędami
- **Silniki Krokowe:** Dedykowane sterowniki (TMC2209 w trybie StealthChop/SpreadCycle) dla płynnej pracy NEMA 17
- **Serwo Chwytaka:** Bezpośrednie sterowanie PWM z Teensy

### 4.3 Zasilanie
- **Sekcja Silników:** Zasilacz 12V/24V (wymagany dla NEMA 17)
- **Sekcja Logiki:** 5V (zasilanie mikrokontrolerów)

### 4.4 Komunikacja
- **Protokół:** Szeregowa (Serial/UART) między Master a Slave
- **Prędkość:** High-speed Serial (możliwe dzięki Teensy)

## 5. System Wizyjny

### 5.1 Wyposażenie
- **Kamera:** USB HD 1080p lub Raspberry Pi Camera
- **Oświetlenie:** Pierścień LED

## 6. Interfejs Kontrolny
Panel sterujący typu **Master** oparty na ESP32-C3.
- Odczyt pozycji operatora (enkodery / potencjometry)
- Komunikacja z jednostką centralną (Teensy)
- Obsługa przycisków funkcyjnych

## 7. Oprogramowanie

### 7.1 Funkcje
- **Kinematyka:** Obsługa High-Speed Stepper (np. biblioteka AccelStepper)
- **Synchronizacja:** Płynne interpolowanie ruchów wieloosiowych
- **Sterowanie:** Przekładnie 20:1 wymagają precyzyjnego rampowania prędkości

## 8. Procedury Montażu
1. Montaż silników NEMA 17 z przekładniami do korpusu.
2. Kalibracja sterowników silników krokowych (ustawienie prądu Vref).
3. Podłączenie elektroniki sterującej (Teensy 4.1).
4. Zaprogramowanie ESP32-C3 jako interfejsu wejściowego.

---
**Wersja dokumentacji:** 1.5 (Zaktualizowana)
**Data aktualizacji:** 2025-12-24  
**Tytuł:** System Teleoperacyjny – Dokumentacja Techniczna  
**Autor:** Adam Motyl
---
**Wersja dokumentacji:** 1.5  
**Data aktualizacji:** 2025-12-24  
**Tytuł:** System Teleoperacyjny – Dokumentacja Techniczna  
**Autor:** Adam Motyl 

