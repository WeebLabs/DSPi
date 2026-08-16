# Kombiniertes Akustik- + DC-Kraftmess-System (Piezo-Kraftwandler-Erweiterung)

Dieses Projekt erweitert das bestehende akustische Transfer-Function-Messgerät (DSPi / Pico 2) um einen DC-fähigen Messpfad für Piezo-Kraftwandler, Dehnungsmessstreifen-Brücken und quasistatische Sensoren.

---

## Inhaltsverzeichnis

1. [Projektbeschreibung](#1-projektbeschreibung)
2. [Anwendungsfälle](#2-anwendungsfälle)
3. [Systemarchitektur](#3-systemarchitektur)
4. [Bill of Materials (BOM)](#4-bill-of-materials-bom)
5. [Schaltungsbeschreibung DC-Pfad](#5-schaltungsbeschreibung-dc-pfad)
6. [Verdrahtungsanleitung](#6-verdrahtungsanleitung)
7. [Leistungskenndaten](#7-leistungskenndaten)
8. [Synchronisation Audio- und DC-Zeitbasis](#8-synchronisation-audio--und-dc-zeitbasis)
9. [Messprozedur](#9-messprozedur)

---

## 1. Projektbeschreibung

Das DSPi-Basisgerät misst akustische Transferfunktionen mit einem I²S-DAC als Stimulus und zwei Mikrofon-/Linieneingängen (PCM1808 Audio-ADC, INA217 Vorverstärker). Da der PCM1808 AC-gekoppelt ist, können keine DC- oder quasistatischen Signale erfasst werden.

Diese Erweiterung fügt zwei DC-fähige Eingänge hinzu:

- **2× BNC-Eingänge** (F1, F2) für Piezo-Kraftwandler oder Dehnungsmessstreifen
- **INA128** Instrumentenverstärker mit einstellbarem Gain (×1 bis ×99)
- **ADS1115** 16-bit I²C-ADC (bis 860 SPS, ±0,256 V bis ±6,144 V Eingangsbereich)
- **I²C-Anbindung** an Pico 2 über GPIO 0 (SDA) und GPIO 1 (SCL)

Beide Messpfade (Audio und DC) laufen parallel und werden auf dem Host-Rechner über Timestamps korreliert.

---

## 2. Anwendungsfälle

| Anwendung | Akustik-Pfad | DC-Pfad |
|-----------|-------------|---------|
| **Piezo-Kraftwandler-Kalibrierung** | Referenzmikrofon (Schalldruckpegel) | Kraft F(t) vom Kraftwandler |
| **Akustische Impedanzmessung** | Schallschnelle via Mikrofon-Differenz | Membrandruck oder Auslenkung |
| **Mechanische Schwingungsanalyse** | Schallemission, Körperschall | Dehnungsmessstreifen-Brücke |
| **Modal-Analyse** | Akustische Antwort der Struktur | Anregungskraft (Impulshammer) |
| **Lärmquellenidentifikation** | Schallfeld-Scan | Strukturkraft an Einleitungspunkt |
| **Lautsprecher-Impedanzmessung** | Audio-Stimulus + Mikrofon | — (nur Akustik) |

---

## 3. Systemarchitektur

```
                    ┌─────────────────────────────────────────────────────┐
                    │                   Pico 2 (RP2350)                   │
                    │                                                     │
  PCM5102A ────I²S──► GPIO 6 (TX)         GPIO 4 (RX) ◄──I²S──── PCM1808│
  (DAC, 1 out)      │  GPIO 13 MCK        GPIO 13 MCK              │     │
                    │  GPIO 14 BCK        GPIO 14 BCK               │     │
                    │  GPIO 15 LRCLK      GPIO 15 LRCLK             │     │
                    │                                          INA217 ×2  │
                    │  GPIO 0 (SDA) ◄──I²C──► ADS1115                    │
                    │  GPIO 1 (SCL)          ADDR=GND → 0x48             │
                    │  GPIO 2 (ALERT/RDY)    (optional)                  │
                    │                                                     │
                    │  USB CDC/Vendor ──────────────────────► Host PC     │
                    └─────────────────────────────────────────────────────┘

  Signalkette DC:
  BNC F1/F2 → 100kΩ → TVS → 1kΩ → INA128 (G=1..99) → Spannungsteiler → ADS1115

  Signalkette Audio:
  XLR → INA217 (+20/+30/+40 dB) → PCM1808 → I²S → Pico 2
```

---

## 4. Bill of Materials (BOM)

### 4.1 Basisgerät (unverändert)

| Bauteil | Bezeichnung | Anzahl | Bemerkung |
|---------|-------------|--------|-----------|
| U1 | Raspberry Pi Pico 2 (RP2350) | 1 | Mikrocontroller |
| U2 | PCM5102A | 1 | I²S DAC, 32-bit/384 kHz |
| U3 | PCM1808 | 1 | I²S ADC, 24-bit/96 kHz |
| U4, U5 | INA217 | 2 | Audio-Instrumentenverstärker, ±9 V |
| U6 | MT3608 | 1 | Boost-Converter für 48-V-Phantomspeisung |
| C1 | 10 µF MKT | 1 | AC-Kopplung DAC-Ausgang |
| R1 | 47 Ω | 1 | Ausgangswiderstand DAC |
| J1 | RCA-Buchse | 1 | Ausgang zu Aktivlautsprecher |
| J2, J3 | XLR-Buchse female | 2 | Mikrofoneingänge |

### 4.2 DC-Erweiterung (neu)

| Bauteil | Bezeichnung | Anzahl | Bemerkung |
|---------|-------------|--------|-----------|
| U7 | ADS1115 | 1 | 16-bit I²C ADC, 4-ch, TI, ADDR→GND = 0x48 |
| U8, U9 | INA128 | 2 | Präzisions-Instrumentenverstärker, G=1..10000 |
| D1, D2 | CDSOT23-SM712 | 2 | TVS-Diode, BNC-Eingangsschutz |
| R2, R3 | 100 kΩ, 1% | 2 | Eingangs-Ruhepfad (1× pro Kanal) |
| R4, R5 | 1 kΩ | 2 | Vorwiderstand vor INA128 |
| R6–R9 | Gain-Widerstand R_G (s. Tabelle) | 2 | Gain-Einstellung INA128, bestückt nach Anwendung |
| R10–R13 | Spannungsteiler (z.B. 10kΩ/10kΩ) | 4 | Anpassung auf ADS1115-Eingangsbereich |
| C2, C3 | 100 nF Keramik | 2 | Bypass ADS1115 VDD |
| C4, C5 | 10 µF Tantal | 2 | Bulk-Bypass INA128 +VS/−VS |
| J4, J5 | BNC-Buchse | 2 | Kraftwandler-Eingänge F1, F2 |
| — | I²C-Pullups 4,7 kΩ nach 3,3 V | 2 | SDA, SCL |

**Optionale Ladungsverstärker-Komponenten (bei Piezo mit Ladungsausgang):**

| Bauteil | Wert | Bemerkung |
|---------|------|-----------|
| R_feedback | 1 GΩ | Biasstrom-Pfad für Ladungsverstärker |
| C_F | 10 nF, C0G | Rückkopplungskondensator, bestimmt Empfindlichkeit (pC/V) |

---

## 5. Schaltungsbeschreibung DC-Pfad

### 5.1 Eingangsbeschaltung BNC (pro Kanal)

```
BNC-Eingang ──┬── TVS (CDSOT23-SM712) ── GND
              │
              ├── 100 kΩ ── GND          (Ruhepfad, verhindert Floating)
              │
              └── 1 kΩ ──► INA128 IN+
                                   IN− ── GND (Single-ended)
```

- Der **100-kΩ-Widerstand** stellt einen definierten Ruhepegel sicher, wenn kein Sensor angeschlossen ist.
- Die **TVS-Diode** begrenzt Überspannungen auf ±5,5 V und schützt INA128 und ADS1115.
- Der **1-kΩ-Vorwiderstand** begrenzt den Kurzschlussstrom und dämpft HF-Einstreuung zusammen mit der Eingangskapazität.

### 5.2 INA128 Instrumentenverstärker

**Gain-Formel:** G = 1 + 50 kΩ / R_G

| R_G | Gain | Typische Anwendung |
|-----|------|-------------------|
| offen (∞) | ×1 | High-output Piezo (Spannungsausgang, 1–10 V) |
| 49,9 kΩ | ×2 | Mittlerer Bereich |
| 5,11 kΩ | ×10,8 | Schwacher Sensor (100 mV FS) |
| 511 Ω | ×99 | Sehr kleines Signal (10 mV FS) |

**Versorgung:** ±5 V oder 3,3 V single-supply mit V_REF = 1,65 V (Midpoint-Referenz)

> **Hinweis Single-Supply-Betrieb:** Bei 3,3 V Versorgung (aus Pico 3V3) wird V_REF des INA128 auf 1,65 V gelegt (Spannungsteiler 10kΩ/10kΩ + Buffer). Der ADS1115 misst dann differenziell. Der tatsächliche Eingangsbereich des ADS1115 sollte auf ±2,048 V oder ±4,096 V eingestellt werden.

### 5.3 Ladungsverstärker (optional, für Piezo mit Ladungsausgang)

```
Piezo ──── 1 GΩ ──┬── INA128 IN+
          ┌───────┘
          │   10 nF C_F
          └───┤├───── INA128 OUT
```

Die Empfindlichkeit berechnet sich als: **S = 1 / C_F [V/C]**

Beispiel: C_F = 10 nF → 100 mV/µC = 100 V/mC

### 5.4 ADS1115

**I²C-Adresse:** ADDR-Pin an GND → **0x48**

**Eingangsbereich (PGA-Einstellung):** *(Hinweis: Die AIN-Pins müssen für Normalbetrieb innerhalb **GND..VDD** liegen; die PGA-FSR ist die interne ADC-Skalierung und erlaubt keine Messung außerhalb der Versorgungsschienen.)*

| PGA | FSR | LSB-Größe |
|-----|-----|-----------|
| ±6,144 V | 6,144 V | 187,5 µV |
| ±4,096 V | 4,096 V | 125,0 µV |
| ±2,048 V | 2,048 V | 62,5 µV ← Standard |
| ±1,024 V | 1,024 V | 31,25 µV |
| ±0,512 V | 512 mV | 15,63 µV |
| ±0,256 V | 256 mV | 7,81 µV |

**Abtastrate:** 8 / 16 / 32 / 64 / 128 / 250 / 475 / **860 SPS** (konfigurierbar)

**Kanalzuordnung:**

| ADS1115-Kanal | Signal |
|--------------|--------|
| AIN0 | Kraftkanal F1 (nach INA128 + Spannungsteiler) |
| AIN1 | Kraftkanal F2 (nach INA128 + Spannungsteiler) |
| AIN2 | frei (z.B. Temperatursensor NTC) |
| AIN3 | frei |

---

## 6. Verdrahtungsanleitung

### 6.1 GPIO-Belegung Pico 2

| GPIO | Funktion | Richtung | Angeschlossen an |
|------|----------|----------|-----------------|
| 0 | I²C0 SDA | bidirektional | ADS1115 SDA + 4,7 kΩ → 3,3 V |
| 1 | I²C0 SCL | bidirektional | ADS1115 SCL + 4,7 kΩ → 3,3 V |
| 2 | ALERT/RDY | Eingang | ADS1115 ALERT (optional, für Interrupt-Betrieb) |
| 4 | I²S RX | Eingang | PCM1808 DOUT |
| 6 | I²S TX | Ausgang | PCM5102A DIN |
| 13 | MCK | Ausgang | PCM1808 + PCM5102A MCLK |
| 14 | BCK | Ausgang | PCM1808 + PCM5102A BCK |
| 15 | LRCLK | Ausgang | PCM1808 + PCM5102A LRCLK |

### 6.2 I²C-Adressierung ADS1115

| ADDR-Pin | I²C-Adresse | Verwendung |
|----------|------------|------------|
| GND | **0x48** | Standard (diese Konfiguration) |
| VDD | 0x49 | Alternative bei Bus-Konflikt |
| SDA | 0x4A | Alternative |
| SCL | 0x4B | Alternative |

### 6.3 Masseführung

- **Sternpunkt-Masse:** Audio-GND (PCM1808/PCM5102A), Signal-GND (INA128/ADS1115) und Chassis-GND werden an **einem einzigen Punkt** zusammengeführt.
- **BNC-Schirm:** Wird am Sternpunkt mit Chassis-GND verbunden. Kein umlaufender Massestrom.
- **Versorgung:** ADS1115 und INA128 werden aus Pico 3V3 (max. 300 mA) versorgt. Bei höherem Bedarf separater LDO (z.B. AP2112K-3.3).

### 6.4 Empfohlene Leitungsführung

- I²C-Leitungen (GPIO 0/1): max. 30 cm, verdrilltes Paar empfohlen
- BNC-Koaxialkabel: so kurz wie möglich, Schirm einseitig auf Sternpunkt
- INA128-Ausgänge zu ADS1115: kurze Leitungen, ggf. RC-Tiefpass (100 Ω, 100 nF) direkt am ADS1115-Eingang

---

## 7. Leistungskenndaten

### 7.1 Akustik-Pfad (Basisgerät, unverändert)

| Parameter | Wert |
|-----------|------|
| ADC | PCM1808, 24 bit |
| Abtastrate | 44,1 / 48 / 88,2 / 96 kHz |
| Eingangsempfindlichkeit | umschaltbar +20/+30/+40 dB (INA217) |
| Eingangsimpedanz | >10 kΩ (XLR balanced) |
| Phantomspeisung | 48 V (MT3608) |
| Dynamikbereich | ~100 dB (limitiert durch INA217 + PCM1808) |
| Frequenzgang | 20 Hz – 40 kHz (–3 dB bei 96 kHz) |
| Eingangsrauschen INA217 | 1 nV/√Hz (@ 1 kHz) |

### 7.2 DC-Kraft-Pfad (Erweiterung)

| Parameter | Wert |
|-----------|------|
| ADC | ADS1115, 16 bit (15 bit effektiv single-ended) |
| Abtastrate | 8 – 860 SPS (konfigurierbar) |
| Eingangsbereich | ±0,256 V bis ±6,144 V (PGA) |
| Auflösung bei ±2,048 V | 62,5 µV/LSB |
| Verstärkungsbereich INA128 | ×1 bis ×10.000 |
| Eingangsrauschen INA128 | 8 nV/√Hz |
| CMRR INA128 | 120 dB (min.) |
| Bandbreite bei G=1 | 1,3 MHz; bei G=100: 50 kHz |
| DC-Genauigkeit ADS1115 | ±0,05% FSR (typisch) |
| Offset-Drift | 0,1 µV/°C (ADS1115) |
| Schutzlevel BNC-Eingang | ±5,5 V (TVS CDSOT23-SM712) |

---

## 8. Synchronisation Audio- und DC-Zeitbasis

### 8.1 Problem

Die Audio-Zeitbasis (I²S-Takt, z.B. 48 kHz) und die DC-Zeitbasis (ADS1115 SPS, z.B. 860 SPS) sind **vollständig unabhängig**. Es gibt keine hardwareseitige Synchronisation zwischen PCM1808 und ADS1115.

### 8.2 Empfohlene Lösung: Timestamp-basierte Kreuzkorrelation

1. **Pico 2 stempelt jeden ADS1115-Messwert** mit dem aktuellen Systemtimer-Wert (µs-Auflösung, `time_us_64()`).
2. **Audio-Frames werden ebenfalls gestempelt** (jeder I²S-DMA-Block erhält einen Timestamp bei Fertigstellung).
3. **Host-Software** erhält beide Datenströme über USB CDC oder Vendor-Interface und korreliert sie anhand der Timestamps.

```
Host-seitige Korrelation (Python-Pseudocode):

import numpy as np

# Interp DC auf Audio-Zeitraster
dc_interp = np.interp(audio_timestamps, dc_timestamps, dc_values)

# Kreuzkorrelation
xcorr = np.correlate(audio_samples[:, 0], dc_interp, mode='full')
lag_samples = np.argmax(xcorr) - len(dc_interp) + 1
lag_seconds = lag_samples / sample_rate_audio
```

### 8.3 Bekannte Einschränkungen

| Einschränkung | Auswirkung | Abhilfe |
|--------------|-----------|---------|
| ADS1115 max. 860 SPS | DC-Pfad limitiert auf ~430 Hz Nutzband (Nyquist) | Für höherfrequente Kraft-Signale externen schnelleren ADC verwenden (z.B. ADS8866, 1 MSPS) |
| Keine Hardwaresynchronisation | Zeitjitter zwischen Pfaden typisch <1 ms | Timestamp-Interpolation ausreichend für niederfrequente Anwendungen |
| USB-Latenz | Schwankende Übertragungszeit | Host-seitiger Jitter-Buffer; Timestamps auf Pico-Seite vergeben |

---

## 9. Messprozedur

### 9.1 Kalibrierung DC-Pfad

1. **Offset-Kalibrierung:** Eingang kurzschließen (BNC terminiert mit 50 Ω), 100 Messwerte mitteln → Offset speichern.
2. **Gain-Kalibrierung:** Präzisionsspannungsquelle anlegen (z.B. 1,000 V), Abweichung bestimmen → Gain-Faktor korrigieren.
3. **Linearitätstest:** 5 Punkte über den Eingangsbereich messen (0, 25, 50, 75, 100% FSR).

### 9.2 Kombinierte akustisch-mechanische Messung

**Szenario: Impedanzmessung Lautsprecher mit Kraftwandler**

1. Lautsprecher aufspannen, Kraftwandler (piezoelektrisch) an Membranrand ankleben.
2. BNC F1 an Kraftwandler anschließen, INA128-Gain je nach Empfindlichkeit wählen.
3. Referenzmikrofon vor Lautsprecher aufstellen, XLR-Eingang 1 anschließen.
4. Messung starten:
   - DSPi sendet Sweep-Signal (MLS oder Sinus-Sweep) über DAC-Ausgang.
   - PCM1808 zeichnet Schalldrucksignal auf (Kanal 1).
   - ADS1115 zeichnet Kraftsignal auf (Kanal F1).
5. Host-Software:
   - Transferfunktion H_akustisch = FFT(Mic) / FFT(Stimulus)
   - Transferfunktion H_mechanisch = FFT(Kraft) / FFT(Stimulus)
   - Mechanische Impedanz Z_mech = H_mechanisch / H_akustisch

### 9.3 Messprozedur Piezo-Kraftwandler-Kalibrierung

1. Kraftwandler in Kalibrierpresse einspannen.
2. Bekannte Kraft F_ref anlegen (Referenz-Kraftaufnehmer).
3. DSPi misst gleichzeitig:
   - F_ref über kalibrierten Referenzaufnehmer (Kanal F2)
   - F_DUT über zu kalibrierenden Wandler (Kanal F1)
4. Empfindlichkeit S = V_out / F_ref [mV/N] bestimmen.
5. Linearität über den Messbereich überprüfen (10%, 25%, 50%, 75%, 100% FS).

### 9.4 Checkliste vor der Messung

- [ ] ADS1115 I²C-Kommunikation prüfen (Scan auf 0x48)
- [ ] Gain-Widerstand R_G korrekt bestückt?
- [ ] Versorgungsspannung INA128: ±5 V oder 3,3 V single-supply?
- [ ] BNC-Schirm am Sternpunkt aufgelegt?
- [ ] Offset-Kalibrierung DC-Pfad durchgeführt?
- [ ] Phantomspeisung aktiviert (falls Kondensatormikrofon)?
- [ ] Pico 2 Firmware: I²C und I²S gleichzeitig aktiv?
- [ ] USB-Verbindung stabil, Baudrate korrekt?

---

## Anhang: Weiterführende Literatur

- Texas Instruments: [ADS1115 Datasheet (SBAS444)](https://www.ti.com/lit/ds/symlink/ads1115.pdf)
- Texas Instruments: [INA128 Datasheet (SBOS051)](https://www.ti.com/lit/ds/symlink/ina128.pdf)
- PCM1808 Datasheet: [SBAS357](https://www.ti.com/lit/ds/symlink/pcm1808.pdf)
- Raspberry Pi: [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- IEEE 1057: Standard for Digitizing Waveform Recorders
