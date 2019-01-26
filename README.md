# TwoPropMixTiny
## 2-prop mixer for RC model with 2 drive props, rudder and bow thruster
![TwoPropMixTiny](https://marcostoffers.github.io/twopropmixtiny_1.png)

- Depending on the direction, the curve inner motor is braked or controlled to reverse (adjustable via a setup). The sensitivity at which curve position the mixer should intervene can be set via a potentiometer.
- If a bow thruster is also used, this can also be used to support cornering (can also be switched on via setup). Up to which speed the bow thruster intervenes can be adjusted with the 2nd potentiometer.

## Requirement
- All motors (right-hand drive, left-hand drive, bow thruster) must have their own speed controller and the two drives must run equally (the model does not pull away to one side when driving straight ahead).
- The model can be "normally" driven via an RC remote control (Forward | Reverse | Left | Right | Bow Thruster Left | Bow Thruster Right)
- The power supply for the module (4.5V - 5.5V) is guaranteed (see manual / only german at the moment).

## Base
The module is based on a microchip ATtiny841 which is programmed via the [Arduino IDE](https://arduino.cc/). To use the microcontroller, the extension of [Spence Konde](https://github.com/SpenceKonde/ATTinyCore) is necessary. Programming is done without a bootloader directly via the ISP contacts on the bottom side of the PCB with an AVR programmer.

## License
The project is licensed under the Creative Common License. A rebuild for private or non-profit associations is desired, commercial rebuild or distribution I forbid. If someone should develop the mixer further, I ask for the naming of the original project.

![CreativeCommonLicense](https://marcostoffers.github.io/cc.png)
