# TwoPropMixTiny
## 2-Schrauben Mischer für ein RC-Modell mit 2 Antriebs-Schrauben, Ruder und Querstrahlruder
![TwoPropMixTiny](https://marcostoffers.github.io/twopropmixtiny_1.png)

- Je nach Richtung wird der Kurven innere Motor gebremst oder auf Rückwärts geregelt (über ein Setup einstellbar). Die Empfindlichkeit, ab welcher Kurvenlage der Mischer eingreifen soll, kann über ein Poti eingestellt werden.
- Wird ein Querstrahlruder mit verwendet, kann auch dieses zur Unterstützung der Kurvenfahrt mitverwendet werden (ebenfalls über Setup zuschaltbar). Bis zu welcher Geschwindigkeit das QSR eingreift, ist wiederrum mit dem 2. Poti regelbar.

## Voraussetzung
- Alle Motoren (Antrieb rechts, Antrieb links, Querstrahlruder) müssen über einen eigenen Fahtsteller verfügen und die beiden Antriebe gleich laufen (das Modell zieht bei Geradeausfahrt nicht zu einer Seite weg).
- Das Modell ist über eine RC-Fernsteuerung "normal" fahrfähig (Vorwärts | Rückwärts | Links | Rechts | QSR Links | QSR Rechts)

## Basis
Das Modul basiert auf einem Microchip ATtiny841, der über die [Arduino IDE](https://arduino.cc/) programmiert wird. Um den Mikrocontroller nutzen zu können, ist die Erweiterung von [Spence Konde](https://github.com/SpenceKonde/ATTinyCore) nötig. Programmiert wird ohne Bootloader direkt über die ISP Kontakte auf der Unterseite der Platine mit einem AVR-Programmer.

## License
Das Projekt steht unter der Creative Common License. Ein Nachbau für Privat oder gemeinnützlichen Vereinen ist erwünscht, kommerziellen Nachbau oder Vertrieb untersage ich. Sollte jemand den Mischer weiter entwickeln, bitte ich um die Nennung des Ursprungprojektes.

![CreativeCommonLicense](https://marcostoffers.github.io/cc.png)
