ESC Control with an Arduino
================

ESC control is used to have a raw control of the ESC.

* Flash the program
* Open Serial (115200 baudrate)
* Connect pin 9 to ESC signal (orange or yellow) 
* Connect GND to esc GND (brown). 
* You are ready to operate.

To use the program, simply send an int to the Arduino, between 800 and 2000, it will send the command to the ESC. 800 is OFF, 1000 is the minimum, 2000 the maximum.

Test the motor/ESC
--------------------

* Send a PWM of 900
* Power the ESC
* *(hear a bip of initialisation saying the ESC is well connected and receive a command)*
* Send a value between 1000 & 2000 to give the wanted speed
* Send 900 again if you want the motor to be stopped.


Reprogram the ESC
--------------------

> The bip sequences are different for every ESC. 

* Send a PWM of 2000
* Power the ESC
* *Hear the bip sequence meaning you are in programmation mode*
* Send a PWM of 1500 or 1000 to go in the menu (depending on the ESC)
* Follow instructions of your ESC datasheet

Change the rotation of a motor using a Firefly (GoodvRC) ESC
--------------------------

> This do not work every time, you may need to proceed multiple times.

* Send a PWM of 900
* Power the ESC
* Send 1100 : The motor rotate in the wrong direction
* Power off the ESC
* Power on the ESC
* Rotate MANUALLY the motor in the right direction
* Power off the ESC
* Power on the ESC
* Send 900
* Send 1100

> The rotation should be changed. (We are unsure about the reboot of the ESC after turning the motor in the right direction)

