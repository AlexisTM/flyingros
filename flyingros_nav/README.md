flyingros_nav
=============

FlyingROS_nav is the navigation package. This is still **experimental** and is subject to API changes (specifically on demand, issues, etc.).

API
------------
There are multiple ways to use the task API. The best way is to take a look on the `scenari` folder or `tests` folder.


Feed the task Controller
------------
There are multiple ways to feed the task controller. The best way is to take a look on the `scenari` folder or `tests` folder.

### Solution 1 :
From a lot of data, like a JSON input. Note that we can directly call
```python
Controller.addTask if we use the tasks once.
# Typical way to fill from any source
for data in dataIn :
    if data.type == "init" :
        task = init_UAV("Init UAV")
    if data.type == "target" :
        task = target("target1", 10,10,10, 0)
    tasks.append(task)
Controller.addTasks(tasks)
```

### Solution 2 :
Create tasks separetly, create an array than add the whole array via Controller.addTasks.

Usefull if we create multiple task array we can use later, like if we want to do a circle, create an array circle, then if you want to do it again, just add the array again.

```python
tasks = []
rospy.loginfo("Adding tasks")
task1 = init_UAV("Init UAV")
task2 = arm("Arming")
task3 = target("target1", Point(0,0,1))
task4 = disarm("Disarming")
tasks = [task1, task2, task3, task4]
Controller.addTasks(tasks)
rospy.loginfo("Tasks added")
```

### Solution 3 :
Directly add tasks via Controller.addTask

Preferred if we just want to try some tasks, added manually.

```python
rospy.loginfo("Adding tasks")
Controller.addTask(init_UAV("Init UAV"))
Controller.addTask(target("target1", Point(0,0,1))
Controller.addTask(arm("Arming")
Controller.addTask(disarm("Disarming")
rospy.loginfo("Tasks added")
```

Navigation methods
-------------

* Tasklist
* Manual control (with minimal user interface)

Future
------------

* Interactive Tasklist
