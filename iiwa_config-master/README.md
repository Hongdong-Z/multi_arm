## Description for the robot control related programs
The model can now drive both "kuka" and "iiwa" dual robotic arms. <br>
In order to realize the function of driving different robotic arms, you need to modify the following parameters. <br>
```
roslaunch robot_moveit_config demo.launch panda:=true (or iiwa:=true)

```
Secondly, if you need to modify the motion trajectory of the robotic arm, you also need to modify the tf coordinates in <br> `robot_moveit_config/launch/demo.launch`, of course, you can also not modify it.<br>
Next, you only need to run the following two commands to drive the corresponding robotic arm.<br>
```
roslaunch robot_control non_blocking.launch panda:=true (or iiwa:=true)

```
And
```
rosrun robot_control adhoc_dispatcher.py 

```

## Description for the task planning related programs
### Building the Packages
Finally the planning packages are no more dependent on ``occupancy_grid_utils``. Old notes: Line 91 in ``occupancy_grid_utils`` was obsolete and has been changed to ``cvtColor(imgColor, img, cv::COLOR_BGR2GRAY);``. 


The planning is not dependent on rddl (with randomness). By the compilation, it throws the error in ROS distribution noetic:
```
implicitly-declared operator is deprecated
```
This is fixed by adding the compile option ``-Wno-error=deprecated-copy`` in Line 5, ``rosplan/rosplan_dependencies/CMakeLists.txt``
However, in Melodic, the line should be commented out.
Make python file executable:
```
sudo chmod +x src/iiwa_config/rosplan/rosplan_action_interface/src/ActionInterfaceManager.py
sudo chmod +x src/iiwa_config/action_simulation/action_interface/scripts/simulation_server.py
```

### Caveat for Python Versions:
The current repository is designed to be able to run both in melodic and noetic. The difference is in the python. Melodic runs python2 but noetic python3. There are the following changes to be made by switching versions:
1. Check the first line of python files, for melodic python2:
```
#!/usr/bin/env python
```
For Noetic python3:
```
#!/usr/bin/env python3
```

### Ausfürhung der Simulation


Starte ROSPlan: 
```
roslaunch systempkg simulation_KUKA_launch.launch
```
Damit werden Knowledge Base, problem interface, parsing interface und plan dispatcher gestartet.

Lade Parameter:

```
cd src/iiwa_config/task_planning/systempkg/config/simulation_KUKA/

rosparam load config_simulation_KUKA.yaml /RPActionInterfaceManager
```
Starte anschließend den Action Interface Manager von ROSPlan:

```
rosrun rosplan_action_interface ActionInterfaceManager.py
```
Starte simulation_server:

```
rosrun action_interface simulation_server.py
```
Durch diesen Befehl wird eine Kommunikation mit dem symbolic maneuver converter hergestellt.

Starte die Simulation mit Rviz

Erzeuge eine Probleminstanz in ROSPlan
```
rosservice call /rosplan_problem_interface/problem_generation_server
```

Anschließend muss ein Plan in rosplan erzeugt werden. Für die Simulation wird ein bereits erstellter Plan verwendet.
```
rosservice call /rosplan_parsing_interface/parse_plan_from_file [full_path_to_directory]/task_planning/systempkg/common/plan/plan_simulation.pddl
```
Schließlich kann der Plan durch den Dispatcher ausgeführt werden:
```
rosservice call /rosplan_plan_dispatcher/dispatch_plan
```
## How to quckily convert between different python versions
There are generally different python versions installed in ubuntu system. You can view and quckily switch the default python version <br> in the following ways. <br> 
1. View the python version and installation path in system.
```
whereis python
```
You will the python versions and installation pathes.Next you need install several different python versions.<br>
For example.
```
sudo update-alternatives --install /usr/bin/python python /usr/bin/python2.7 1
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.6 2
```
Number 1 and 2 are default priorities.
2. Select the desired python version.
```
sudo update-alternatives --config python

``` 
You can see the optional python versions in terminal. You can set the default python version by entering the number before the version.



