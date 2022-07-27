
# The position and function of this project in the overall process
![whole process](https://user-images.githubusercontent.com/77051392/181216600-cbee685f-3d24-4869-b1cd-b4fef672fb78.png)

**Figure 0: Overall process**



https://user-images.githubusercontent.com/77051392/181215889-259c168a-9fd1-4b51-8385-22d43bd2874c.mp4


# How to build dual robot arm simulation control from 0
Next, I will give you some advice on how to extend the existing system to the new robotic arm, which generally includes the following steps.
## URDF writing
In this step I assume that you already have a URDF file for a single robot arm, all you need to do is to create a new URDF file for a dual manipulator.
### *1. Build the manipulator scenes and the 3D model of the coupling module*
1. First, store the corresponding model created by the 3D modeling software as stl format in the meshes folder under the robot_description folder.

![Screenshot 2022-07-27 10:37:23](https://user-images.githubusercontent.com/77051392/181216743-79e7511b-cd3a-41cc-b034-208ea063b925.png)

**Figure 1: Workspace settings**

2. Next, you need to establish the relative positional relationship of the dual robotic arms and import the corresponding modules into the new urdf file. It is worth noting here that the relative position relationship between the two robotic arms and the selection of the world coordinate system, if there is no special instruction, you can refer to the previous model file, and take the base of the left arm as the world coordinate system, and the right arm at the The x-axis square of the world coordinate system.

![Screenshot 2022-07-27 10:47:42](https://user-images.githubusercontent.com/77051392/181216790-8b755cec-dcd9-41b7-92a9-5daf22c65355.png)

**Figure 2: Configuration example of the relative position relationship of the robot arm**
## moveit configuration
In this step, you need to open a terminal in your workspace and enter:
`roslaunch moveit_setup_assistant setup_assistant.launch`
and you will see.
![Screenshot 2022-07-27 10:51:51](https://user-images.githubusercontent.com/77051392/181216831-86d1d7c5-1d9b-4f77-b927-f8c58ba279c3.png)

**Figure 3: Moveit Assistant setup**

Here you can choose to create a new moveit configuration package or modify the existing configuration file. For the specific process, you can check the official tutorial
http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html
## ROS interface configuration
In this step, the control files of the dual robotic arms have been designed, and the general working principle is shown in the figure below.

![workflow](https://user-images.githubusercontent.com/77051392/181216891-801b8f23-c33f-4cbb-8446-772c27620e2e.png)

**Figure 4: Control principle of double manipulator**
Since the urdf configuration files of different robots are written differently, you need to create a new param file in the robot_control/config/ directory. The specific parameters to be rewritten are shown in the following figure

![Screenshot 2022-07-27 11:07:46](https://user-images.githubusercontent.com/77051392/181216939-013ea189-8bd1-4d8a-9415-2b71348566fd.png)

**Figure 5: Parameter file example - comau**

At the same time, you also need to add the corresponding param file path and loading method in launch.
![Screenshot_2022-07-27_11_12_45](uploads/2db063cdca6fad1322842aa236903f82/Screenshot_2022-07-27_11_12_45.png)

**Figure 6: Control launch file modification example**

## Trajectory adjustment of the end effector of the robotic arm

First, you need to publish the motion trajectory coordinates of the end effector of the robotic arm in moveit_config/demo.launch

![DeepinScreenshot_select-area_20220727112945](uploads/4ba74b97ec12e593102ae3cd20bf339c/DeepinScreenshot_select-area_20220727112945.png)

**Figure 7: Publish TF**

At this stage, the coordinated point-to-point motion of the arms is realized through the /src/robot_control/scripts/adhoc_dispatcher.py file. Of course, you can also realize it through task_planning. Please refer to the task_planning section of this project for the specific principle and implementation.

In the adhoc_dispatcher.py file, you need to provide the respective motion trajectories and motion modes of the arms, such as the figure below.
![DeepinScreenshot_select-area_20220727112040](uploads/6d375c42e51466c88005af20a7e9dac5/DeepinScreenshot_select-area_20220727112040.png)

**Figure 8: Robot arm end effector trajectory and motion configuration**

### The specific startup process is in the following file, and you can make the overall process run through continuous debugging. I wish you success and joy in the process.
