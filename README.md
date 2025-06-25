# RCPowerSwitch

A simple design to switch an electrical load, upto 12V@ 3A (36W) using the PWM output of a standard radio control receiver.
You can set the on/off thresholds and the design supports normal and reverse servo positions. The design can operate, just from a 1S Lipo (4.2V) upto a 12V PB or 3S Lipo battery.

Two variants of the PCB exist, the original one, not published here and the smaller 27mm x 33mm PCB, published here. 

The Microchip PIC18F1822 microcontroller is used to detect the Servo pulses, using the capture compare function of the timers, this provides a timer resolution of <100ns, more than enough for the appplication in hand.
