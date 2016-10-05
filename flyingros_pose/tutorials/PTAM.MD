ETHZ-ASL PTAM
===========================

> ETHZ-ASL PTAM tested
>
> Calibration working
>
> PTAM "working" only if you keep a part of the initial image into the FOV. 

Dependencies
-----------


Installation
-----------

```
cd ~/Workspace/Catkin/src
git clone https://github.com/ethz-asl/ethzasl_ptam

cd ..
catkin_make
```
Calibration ATAN
-----------

Print the Checkerboard, then start the camera.

```
roslaunch ptam ptamcalibrator
```

Take multiple captures with the checkerboard, multiple angles of view, everywhere on the screen. Once done, click optimize and copy the results.
