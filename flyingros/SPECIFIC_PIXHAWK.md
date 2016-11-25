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

### Choice of the position estimator

* `INAV` - deprecated (I had issues with the barometer)
* `LPE` - Works better indoor
* `EKF2` - Works better outdoor

### LPE (Local Position Estimation) configuration

* Enable LPE with the parameter `SYS_MC_EST_GROUP` to local_position_estimator

Pitfalls
-----------

### Wrong connection port

If you connect the Pixhawk via the USB, with an Arduino also in USB, you will have multiple ports with the same name, like `ttyACM0` and `ttyACM1`. To know which port is what, you can create a symlink to the right device, like `ttyPixhawk` will always be the Pixhawk.

```
cat > $HOME/rule.tmp <<_EOF
# Pixhawk
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", SYMLINK+="ttyPixhawk"
# An Arduino or other device (get the id with lsusb)
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="ttyArduino"

_EOF
sudo mv $HOME/rule.tmp /etc/udev/rules.d/10-px4-names.rules
sudo /etc/init.d/udev restart
```

### User rights

To avoid having to use sudo for your ROSLaunch, you need to add yourself in the autorized group.

```
cat > $HOME/rule.tmp <<_EOF
# All 3D Robotics (includes PX4) devices
SUBSYSTEM=="usb", ATTR{idVendor}=="26AC", GROUP="plugdev"
# FTDI (and Black Magic Probe) Devices
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", GROUP="plugdev"
# Olimex Devices
SUBSYSTEM=="usb",  ATTR{idVendor}=="15ba", GROUP="plugdev"
_EOF
sudo mv $HOME/rule.tmp /etc/udev/rules.d/10-px4-rights.rules
sudo /etc/init.d/udev restart

sudo usermod -a -G tty $USER
sudo usermod -a -G plugdev $USER
sudo usermod -a -G dialout $USER
``` 