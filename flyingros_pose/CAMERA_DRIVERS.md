Tested camera drivers
======================

Odroid 720p camera
--------------------

Simply install `usb-cam` from package or from source

Mako G 131-b - Ethernet camera from Alliance Vision
--------------

### Old Gige SDK

Install the following packages:
* https://github.com/ros-drivers/driver_common hydro-devel
* https://github.com/ros-drivers/prosilica_driver
* https://github.com/ros-drivers/prosilica_gige_sdk

NOTE : Gige SDK did not work for me.

### New Vimba SDK

```
sudo apt-get install ros-indigo-polled-camera ros-indigo-driver-base

git clone https://github.com/srv/avt_vimba_camera
cd avt_vimba_camera
# kinetic branch
# apply this fix https://github.com/srv/avt_vimba_camera/issues/16
```
