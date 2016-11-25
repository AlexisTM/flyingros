flyingros_nav
=============

flyingros_nav is the navigation package. This is still **experimental** and is subject to change (specifically ondemand, issues, etc.).


Navigation methods
-------------

* Tasklist
* Manual control (with minimal user interface)

Available nodes
-------------

* task_node - Allow to manage tasks from another nodes or (with rosbridge) from distant computer
* nav_application - Using task\_node, is an onboard application (to launch through ssh) with a basic URWID interface
* manual_node - Use the multicopter though SSH without high level tasks. (Direct access of the `/mavros/local_setpoint/pose`)

Available scenari
-------------

### Standalone in Python (direct access to tasks.py, no node)

* [scenario_circle](scenari/scenari_py/scenario_circle) - Make a circle (standalone)
* [scenario_square](scenari/scenari_py/scenario_square) - Make a square
* [scenario_stairway](scenari/scenari_py/scenario_stairway) - Make a stairway

### Though task_node via the web application (flyingros\_pose) 

* [scenario_web_circle.json](scenari/scenari_web/scenario_web_circle.json) - Make a circle (standalone)
* [scenario_web_square.json](scenari/scenari_web/scenario_web_square.json) - Make a square