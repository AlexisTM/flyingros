Fixed lasers algorithm
==============

Fixed lasers is one of the numerous localisation system. The main (and only) advantages is to be *relative* to the room, relativly cheap (600$) and in 3D. 


2D (altitude) algorithm
---------------

Depending on the right Euler convension, the altitude can be independant to the Yaw. By default, we will use the convention `sxyz` of the [transformation.py](http://www.lfd.uci.edu/~gohlke/code/transformations.py.html) library, meaning Static axes, rotation around X, Y then Z. We can use any convention for our computations as we receive a quaternion, which do NOT have any misunderstanding possible. (You can experiment this in [Geogebra](https://www.geogebra.org/))

This means we can compute the altitude without having to *correct* the yaw angle which means nothing when we have no walls. Consider the lasers have an offset on the measure, an offset on the position and an offset on the orientation. 

As we explain in the 3D algorithm, we will simply have the equation `Z=f(Measure, roll, pitch)` to compute the number of times we have lasers. 

The computation itself becomes really simple.

* Rotate the position of the lasers on the multicopter to know its actual position 
* Rotate the orientation of the lasers on the multicopter to know its actual orientation 
* Extrapolate the measure to know what is actually pointed. `target = measure*orientation - position`
* The Z coordinate of the target is the distance to the floor 

3D algorithm
-----------------------

As for the altitude, Z do not depend on the yaw (`sxyz`), X do not depend on the roll (`syzx`) and Y do not depend on the pitch (`sxzy`). The conventions are the same as [transformation.py](http://www.lfd.uci.edu/~gohlke/code/transformations.py.html) library, meaning Static axes, rotation around X, Y then Z. We can use any convention for our computations as we receive a quaternion, which do NOT have any misunderstanding possible.

The algorithm simply changes the problem to a simpler one. Instead of thinking "Where are we in the room?", simply think "How far we are from the wall/floor". Instead of a complex 6 dimension problem `[x,y,z] = f(Measure_x1, Measure_x2, Measure_y, Measure_z, roll, pitch, yaw)` we have a pretty simple 3 dimensional problem. The problem is split into multiple equations;

```
yaw_c1 = f(Measure_x1, Measure_x2, pitch)
yaw_c2 = f(Measure_y1, Measure_y2, roll)
yaw_c = (yaw_c1+yaw_c2)/2
X=f(Measure, pitch, yaw)
Y=f(Measure, roll, yaw)
Z=f(Measure, roll, pitch)
```

The computation is the same as for the altitude, but we have to correct the heading (yaw) before doing the computations 

* Create a quaternion with zero yaw rotation
* Rotate the position of the lasers on the multicopter (X and Y) to know its actual position with null yaw
* Rotate the orientation of the lasers on the multicopter (X and Y) to know its actual orientation with null yaw
* Compute the yaw angle which is function of the difference we measured between the two lasers and their position.
* Create the final quaternion with the computed yaw
* Rotate the position of the lasers on the multicopter to know its actual position with the computed yaw
* Rotate the orientation of the lasers on the multicopter to know its actual orientation with the computed yaw
* Extrapolate the measure to know what is actually pointed. `target = measure*orientation - position`
* The X coordinate of the target is the distance to the wall X
* The Y coordinate of the target is the distance to the wall Y
* The Z coordinate of the target is the distance to the floor