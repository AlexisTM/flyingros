flyingros_nav
=============

FlyingROS_nav is the navigation package. This is still **experimental** and is subject to API changes (specifically on demand, issues, etc.).

API
------------
There are multiple ways to use the task API. The best way is to take a look on the `tasks` library into the `flyingros_libs` folder

### Tasks
Every task is a child of the main task class. Here are the most useful informations you need to know.

* On the speed of the update rate, the controller will execute the `run(UAV)` function.
* The `run` function will return `isDone()`.
* Every task has a `Name` and a `Type` wich are strings.
* The `Type` is hardcoded into the task instance
* The `Name` is free for the user to put any information he wants
* The `task` should have the `__str__` function used to have more informations on the actual task.

#### target(name, pointXYZ, yaw = 0, precisionXY = 0.05, precisionZ = 0.05, precisionYAW = 1)
Ask the UAV to go to a certain position. The precision is the expected exactitude of the position. If the multicopter never goes at the exact position +/- the precision, then the task will never pass.

* name : string
* pointXYZ(m) : geometry_msgs/Point
* yaw(rad) : float python
* precisionXY(m) : float python
* precisionZ(m) : float python
* precisionYAW(rad) : float python

#### loiter(name, waitTime)
Ask the multicopter nothing but the last setpoint for waitTime time.

* name : string
* waitTime(s) : float python

> Note that if the rate is 1Hz, it can be up to one second more than waitTime

#### loiter_callback(name, waitTime)
**DO NOT USE** : WIP

> The aim is to use a callback to avoid the 1/rate time.


#### takeoff(name, precision = 0.05)
Takeoff up to the `UAV.takeoff_altitude` with a certain precision.

* name : string
* precision(m) : float python

#### land(name, precision = 0.05)
Land until the `UAV.landing_altitude` with a certain precision.

* name : string
* precision(m) : float python

#### init_UAV(sleep = 5)
Init the PixHawk home. To be optimized, API keep the same.

* name : string
* sleep(s) : float python, time to wait.

#### arm(name, timeout = 1)
Arm the multicopter, retry until armed.

* name : string
* timeout(s) : Time to wait between Arming and Offboard mode.

#### disarm(name, timeout = 1)
Disrm the multicopter, retry until disarmed.

* name : string
* timeout(s) : Time to wait before checking if the disarming succeeded.

### Controller
It maintain the task list and send it when needed to the UAV class.

#### Controller(rate=10, setpoint_rate=10)
Init the controller.

* rate(Hz): Rate of update of the controller. Using smaller rate reduce the reactivity. For example, 2Hz means it waits one half second between two check of the position.
* setpoint_rate(Hz): The setpoint rate, passed to the UAV instance.

#### ID = Controller.addTask(task)
Add one task, it **must be** a child of the task *class*. Returns the ID (float) of the task.

#### LastID = Controller.addTasks(tasks)
Add multiple tasks, it **must be** childs of the task *class* concatenated into an array of tasks. It returns the ID of the last task added.

#### tasks = Controller.getTasks()
Returns the array with all tasks.

#### task = Controller.getTask(index)
Returns a specific task from the ID. Returns -1 if no task found.

#### task = Controller.getCurrentTask()
Returns the current task executed.

#### Controller.setRate(rate)
Change the current update rate.

#### Controller.getUAV()
Actually returns the instance of the UAV.

> CAUTION : This can be dangerous if you use the UAV at the same time as the Controller. Please double check.

#### Controller.spinOnce()
Update the task controller.

> You have to call this mutltiple times. It  

```python
# Typical use
while not stop :
    Controller.rate.sleep()
    Controller.spinOnce()
```

#### Controller.rate.sleep()
Wait the time needed to spin one more time.

### UAV
It represents the UAV to control. This class actually sends the MavLink packet (through `mavros`). Mainly used in the controller, you are not supposed ot use it unless you are improving the library (Thanks to you).

#### UAV(setpoint_rate=10)
Init the UAV object with a setpoint_rate of 10 Hz

#### UAV.setpoint_position(position, yaw)
Change the setpoint to send to a position setpoint with a certain orientation.

* position (m): `geometry_msgs/Point`
* yaw (rad): will be `Float32`, not used now

#### UAV.setpoint_takeoff_here_position(altitude)
Takeoff here, to a certain altitude (m).

#### UAV.setpoint_land_here_position()
Emergency landing, land here.

#### UAV.setpoint_takeoff()
Takeoff at the current setpoint.

#### UAV.setpoint_land()
Land at the current setpoint.

#### UAV.setpoint_loiter()
Keep the current setpoint, just keep stabilized.

#### UAV.arm(arming_state)
Arm or disarm the multicopter.

* arming_state = True to arm
* arming_state = False to disarm

#### UAV.set_offboard()
Put the controller in Offboard mode

#### UAV.die()
Kill the UAV, it has 3 seconds to land then shutdown the motors.

> CAUTION: Killing the controller automatically kills the UAV.

#### [position,yaw] = UAV.getPosition()
Returns the actual position and yaw of the UAV.

* position(m) [Point]
* yaw(rad) [Float32]


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
