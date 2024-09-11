# Building a complete ROS project

## 1. Create a workspace

```
mkdir -p ros_tutorial_ws/src
cd ros_tutorial_ws
catkin_make
```

## 2. Describing the robot

### 2.1 ROS packages of the robot components

Create a new folder inside src:
```
mkdir robot
```

Copy the ROS packages of the following components into the folder:
- ABB robotic arm
- Intel realsense RGB-D camera

Compile the workspace:
```
catkin_make
```

### 2.2 Visualize the components

In a terminal, run the following command to visualize the robotic arm in Rviz:
```
roslaunch abb_irb1200_support test_irb1200_5_90.launch
```

In a separate terminal, run the following command to see all the frames and how they are linked to each other:
```
rosrun rqt_tf_tree rqt_tf_tree
```

Similarly, you can visualize the camera and its frames:
```
roslaunch realsense2_description view_l515_model.launch
```
In a different terminal:
```
rosrun rqt_tf_tree rqt_tf_tree
```

### 2.3 Combining the arm and the camera using URDF

- Copy the package called abb_l515_description, which combines the ABB robotic arm and the L515 camera. Then, re-compile the workspace.
```
catkin_make
```
- Open the abb_l515.xacro file for details
- Visualize the final robot and the merged frames:
```
roslaunch abb_l515_description abb_l515_description.launch
```
In a different terminal:
```
rosrun rqt_tf_tree rqt_tf_tree
```

## 3. Motion planning and control

We use an existing ROS library called MoveIt, which enables setting up your robot with different motion planning and inverse kinematics solvers.

### 3.1 Setup the robot in MoveIt

Run the MoveIt setup assistant and follow the steps:
```
roslaunch moveit_setup_assistant setup_assistant.launch
```

One important step is setting up the controller. This depends on the hardware capability and what type of control you want to implement.

## 4. Simulation environment

Before we can test the motion planning and control of the robot, we need a simulation environment.

- Copy the package simulation_environment to the workspace
- Re-compile and source
```
catkin_make
source devel/setup.bash
```
- Launch the simulation environment
```
roslaunch simulation_environment simulation_environment.launch
```
- To edit the environment, open "bunny.world" file and change parameters, for example, the position of the bunny

### 4.1 Load the robot within the simulation environment

- We need to launch the robot and the simulation environment together
- This is done in the abb_l515_bringup package
- In this package, we launch the robot description, the robot controller from moveit, and also the robot visualisation in Rviz
- Copy the package to your workspace and re-compile
- Then, launch the robot together with the simulation environment
```
roslaunch abb_l515_bringup abb_l515_bringup.launch
```

### 4.2 Visualize simulation features

- Color and depth image
- Point cloud
- Pose of camera with respect to the world frame
```
rosrun tf tf_echo /world /camera_color_frame
```

### 4.3 Test the robot control in Rviz

- Provide a goal position or goal joint angles, and move the robotic arm to goal

## 5. Control the robot using code

- Copy the packages abb_control, ros_numpy, and utils to the workspace
- Re-compile and source
- Launch the robot in simulation
```
roslaunch abb_l515_bringup abb_l515_bringup.launch
```
- In a new terminal, run the python script for moving the robotic arm to a desired pose
```
rosrun abb_control arm_control_test.py
```
- Modify the arm_control_test.py file to move the robotic arm to different desired poses