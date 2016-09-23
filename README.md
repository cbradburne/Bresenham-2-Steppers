# Bresenham-2-Steppers
Arduino controlled, 2 steppers for Pan / Tilt system using Bresenham line algorithm and sudo acceleration.

I couldn't find any code online for the system I was after so I've created this.  It's in it's very early stages but works ok so far.  Try not to judge the coding too harshly, it's not my forte!

The code is written to work with:
  - Arduino Uno
  - Adafruit Motor Sheild v2.3
  - 2x 4 wire stepper motors
  - (soon to work with nrf24l01+ transceiver for remote operation)

Usage: Connect 1 end of wire to gnd, tap the other end on 
  - pin 2 or 3 to move motor 1, CW - CCW
  - pin 4 or 5 to move motor 2, CW - CCW
  - pin 6 or 7 to set location, 1 - 2
  - pin 8 or 9 to recall location, 1 - 2

Things to sort:
  - Add code for wireless nrf24l01+ transceiver, including mapped joystick control (untested)
  - Add logarithmic lookup table for smoother acceleration / deceleration (tested, not working very well)
