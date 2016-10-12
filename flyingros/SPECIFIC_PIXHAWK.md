PixHawk specific documentation
===============================

The PixHawk is the flight controller. In our case, we are always using it in `OFFBOARD` mode, which allows the onboard computer to be the real master. For more complete informations, you should go to [the development website](http://dev.px4.io/).

Configurations
-----------------

### Connect the PixHawk to the Odroid

* Change the parameter `SYS_COMPANION` to `921600` instead of `57600` to have a higher baudrate (921600 8N1 instead of 57600 8N1) on TELEM2.
* Connect TELEM2 to the Odroid XU4 via the Odroid XU4 shifter shield (GPA0.0(RX), GPA0.1(TX), GND)
* With the pinout : VCC, TX, RX, CTS, RTS, GND of the TELEM2 port, you have to connect GPA0.0(RX) to TX and GPA0.1(TX) to RX

### Enable External pose input

* `CBRK_NO_VISION` to 0 to enable Vision position integration
* `ATT_EXT_HDG_M` to 0 to disable external heading, to 1 to use the vision heading and to 2 to use the mocap heading.

### LPE (Local Position Estimation) configuration

* Enable LPE with the parameter `SYS_MC_EST_GROUP` to local_position_estimator
