Visual estimation
==================

Informations comes from : [VO_Tutorial.pdf](http://mrsl.grasp.upenn.edu/loiannog/tutorial_ICRA2016/VO_Tutorial.pdf)

Here is a list of the visual estimation possibilities

Visual Odometry (VO)
-----------------

* Original PTAM (feature-based, mono, for desktop AR) - http://www.robots.ox.ac.uk/~gk/PTAM/
* PTAM (feature-based, mono, for MAV) - http://wiki.ros.org/ethzasl_ptam
* PTAMM (feature-based, mono, for AR) - http://www.robots.ox.ac.uk/~bob/research/research_ptamm.html
* LIBVISO2 (feature-based, mono and stereo, for marine) - http://www.cvlibs.net/software/libviso/
* SVO (semi-direct, mono, stereo, multi-cameras, for MAV) - https://github.com/uzh-rpg/rpg_svo

Visual Inertial Odometry (VIO)
------------------

* rovio (tightly coupled EKF) - https://github.com/ethz-asl/rovio
* okvis  (non-linear optimization) - https://github.com/ethz-asl/okvis_ros
* ETHZASL MSF (Multi sensor fusion, EKF, loosely-coupled) - https://github.com/ethz-asl/ethzasl_msf
* SVO + GTSAM - https://bitbucket.org/gtborg/gtsam, informations here : http://arxiv.org/pdf/1512.02363


Visual Simultaneous Localization And Mapping (VSLAM)
------------------

* ORB-SLAM (feature based, mono) - https://github.com/raulmur/ORB_SLAM
* ORB-SLAM2 (feature based, mono, stereo, RGB-D, relocalization) - https://github.com/raulmur/ORB_SLAM2
* LSD-SLAM (semi-dense, direct, mono) - https://github.com/tum-vision/lsd_slam
* iSAM : Incremental Smoothing and Mapping - http://people.csail.mit.edu/kaess/isam/

Libraries widely used
-----------------

### Optimization
* GTSAM - https://collab.cc.gatech.edu/borg/gtsam?destination=node/299
* G2O - https://openslam.org/g2o.html
* Google Ceres Solver - http://ceres-solver.org/

### Place recognition
* DBoW2 - https://github.com/dorian3d/DBoW2
* FABMAP - http://mrg.robots.ox.ac.uk/fabmap/

### SFM Tools for MAV (3D/2D reconstruction)
* MAVMAP (Open source) -  https://github.com/mavmap/mavmap
* Pix4D (Closed source) - https://pix4d.com/

Performances
-----------

SVO for more fps (60cm error RMS)

SVO + BA (bundle adjustment) for more precision (7cm error RMS)

SVO works better on a down facing camera

SVO works better with a wide angle camera

Some more theory
-----------------

VO : Visual Odometry is the process of incrementally estimating the pose of the vehicle by examining the changes that motion induces on the images of its onboard camera(s). Assumptions : **Sufficient illumination**, **dominance of static** scene over moving objects, **Enough texture** to allow apparent motion to be extracted and **sufficient scene overlap** between consecutive frames. It focuses on **real-time**, on a **sequential** dataset.

VSLAM : Visual Simultaneous Localization And Mapping is similar to VO but aims a **global consistency** by keeping old frames or keyframes. We can build VSLAM from VO, by closing the loop.

SFM : Structure from Motion is more general than VO and tackles the problem of 3D
reconstruction and 6DOF pose estimation from unordered image sets.

BA : Bundle adjustment is more costly because it will select better keyframe for better position

MAV Datasets
----------------
* EUROC MAV Dataset (forward-facing stereo) - http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
* RPG-UZH dataset (downward-facing monocular) - http://rpg.ifi.uzh.ch/datasets/dalidation.bag
