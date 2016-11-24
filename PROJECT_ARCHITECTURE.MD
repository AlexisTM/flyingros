Project Architecture
===================

The project is made of 6 idependant packages. 

* flyingros
  - Main informations 
  - The global launch files
* flyingros_libs
  - Python libraries 
* flyingros_msgs
  - Message definition 
  - Service definition
* flyingros_nav
  - Navigation software (task node,  manual node)
  - Python interface for the tasks 
  - Scenari for the copter in Python and in JSON
* flyingros_pose
  - Localisation nodes
  - Localisation informations, comparaison (choose yours)
  - Tutorials 
* flyingros_web
  - The non ros interface (though and non ROS 
  - Web export 
  - CSV export 