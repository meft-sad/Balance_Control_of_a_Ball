# Balance Fontol of a Ball

##### By: Pedro Rossa

## Introduction

This project aim is to

## Components of the project
In this section are the list of all componets used to do this project and the corresponding datasheet.
### Microcontroller
[Arduino MEGA 2560](https://ww1.microchip.com/downloads/en/devicedoc/atmel-2549-8-bit-avr-microcontroller-atmega640-1280-1281-2560-2561_datasheet.pdf);
### Inputs 
Buttons;
### Outputs
[HDSP-B0xE](http://www.farnell.com/datasheets/2095876.pdf) :  Four digit seven segment display;


### Sensors
The sensors used in this project and corresponding datasheet are:
* [HC-SR04](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf) : Sonar to measure distances;
* [MG90S](https://www.electronicoscaldas.com/datasheet/MG90S_Tower-Pro.pdf) : Servo motor metal gear;
* [MPU-6000A](https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf) : Accelerometer and gyroscope module 3 axes.

## [HC-SR04](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf) : Sonar to measure distances

## [MG90S](https://www.electronicoscaldas.com/datasheet/MG90S_Tower-Pro.pdf) : Servo motor metal gear

## [MPU-6000A](https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf) : Accelerometer and gyroscope module 3 axes

## [HDSP-B0xE](http://www.farnell.com/datasheets/2095876.pdf) :  Four digit seven segment display

To work with this type of diplay its needed to send to the HDSPBOxE 12 outs so for that purpose was created the following table to help control the display:

![tabella](Tables_Imag/7_S_D.png)
And after that using the ABC... scheme showed above it was code the bit of the prots PF0 to PF7 to write the numbers:
![tabella](Tables_Imag/Code_numb.png)

# Test of the code 
In the following gif is showing one of the test done of the machine working and as you can see the objective of balance a ball and keep it in the middle is achieved.
![Alt Text](/Tables_Imag/test.gif)