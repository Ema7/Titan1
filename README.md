More information on the Titan 1 mission can be found here:
www.bombasaro.org/titan1


/*
 TITAN HAB flight computer software
 Modifications copyright 2015 by Emanuel Bombasaro

 The original code and subfiles is by:
   HABDuino Tracker
   http://www.habduino.org
   (c) Anthony Stirk M0UPU
   November 2014 Version 3.0.3
   Please visite for lates release and details: https://github.com/HABduino/HABduino

   Modifications done are:
   Make it work with Arduino Mega 2560 rev3
   All parts regarding additional sensors and logging to SD-card.
   Some modifications on formatting and variable naming.
   However, please start with looking into the original code first!

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 See <http://www.gnu.org/licenses/>.

 This code is suitable for the Arduino MEGA 2560 rev3.
 Pin connection as outlined below.

 HABduino, mounat as it is.
 NB. I2C is connected as follows:
   connect SCL to analog 21
   connect SDA to analog 20
 thus the pins on the HABduino are not correct.

 SD card attached to SPI bus as follows:
   MISO - pin 50
   MOSI - pin 51
   SLK - pin 52
   SS - pin 53

+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

created 25-XII-2014 EMBO
edited  20- II-2015 EMBO
edited  03- III-2015 EMBO fixed sd write, no in the delay loop seems to work better.
edited  16- III-2015 EMBO reduced error value for sensors
edited  18- III-2015 EMBO gyro, acc and mag calibration ands sensor redadings
edited  24- III-2015 EMBO mag calibration now advance
*/
