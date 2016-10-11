#!/usr/bin/env python
from flyingros_libs.tasks import *
import rospy
from signal import signal, SIGINT
from geometry_msgs.msg import Point
import sys

def signal_handler(signal, frame):
        print('You pressed Ctrl+C')
        print('Leaving the Controller & closing the UAV')
        Controller.__exit__()
        sys.exit(0)

rospy.init_node('test_tasks')
Controller = taskController(rate=3, setpoint_rate=10)
rospy.loginfo("Controller initiated")
signal(SIGINT, signal_handler)

# target("name", destination, yaw, precision_xy, precision_z, precision yaw)
# destination = Point(0,0,0)
# yaw = 0 - default
# precision_xy = 0.05 - default
# precision_z = 0.05 - default
# precision_yaw = 1 - default


""" Solution 1 : From a lot of data, like a JSON input. Note that we can directly call
Controller.addTask if we use the tasks once.

# Typical way to fill from any source
for data in dataIn :
    if data.type == "init" :
        task = init_UAV("Init UAV")
    if data.type == "target" :
        task = target("target1", 10,10,10, 0)
    tasks.append(task)
Controller.addTasks(tasks)
"""

""" Solution 2 : Create tasks separetly, create an array than add the whole array via Controller.addTasks
Usefull if we create multiple task array we can use later, like if we want to do a circle,
create an array circle, then if you want to do it again, just add the array again.

tasks = []
rospy.loginfo("Adding tasks")
task1 = init_UAV("Init UAV")
task2 = arm("Arming")
task3 = target("target1", Point(0,0,1))
task4 = disarm("Disarming")
tasks = [task1, task2, task3, task4]
Controller.addTasks(tasks)
rospy.loginfo("Tasks added")
"""

""" Solution 3 :  Directly add tasks via Controller.addTask
Preferred if we just want to try some tasks, added manually.
rospy.loginfo("Adding tasks")
Controller.addTask(init_UAV("Init UAV"))
Controller.addTask(target("target1", Point(0,0,1))
Controller.addTask(arm("Arming")
Controller.addTask(disarm("Disarming")
rospy.loginfo("Tasks added")
"""


rospy.loginfo("Adding tasks")
Controller.addTask(init_UAV("Init UAV"))
Controller.addTask(loiter("WAAAAIT", 2))
Controller.addTask(loiter_callback("WAAAAIT", 2))
Controller.addTask(target("target1", Point(0,0,1)))
Controller.addTask(arm("Arming"))
Controller.addTask(disarm("Disarming"))
rospy.loginfo("Tasks added")

# for i in range(100):
while True:
    Controller.rate.sleep()
    Controller.spinOnce()
    rospy.loginfo("Task %s on %s : %s", Controller.current+1, Controller.count, Controller.getCurrentTask().__str__())

Controller.__exit__()
sys.exit(0)
