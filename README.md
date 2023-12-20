# Intelligent Robotic Manipulation

The overall Goal of the project is to grasp a YCB-Object and place it in a goal basket while avoiding obstacles. To do
this you need to implement a Controller (task 1) to move your robot arm, sample and execute a grasp (task 2), localize
and track obstacles (task 3) and plan the trajectory to place the object in the goal, while avoiding the obstacles (task
4). Besides the task itself, we will also grade the structure of your codebase and the quality of your report.

We heighly recommend you go through the [Pybullet Documention](https://pybullet.org/wordpress/index.php/forum-2/)

Make sure you have [anaconda](https://www.anaconda.com/) or [miniconda](https://docs.conda.io/projects/miniconda/en/latest/miniconda-install.html) installed beforehand.
```shell
git clone https://github.com/RiicK3d/irobman_project.git
cd irobman_project
conda create -n irobman python=3.8
conda activate irobman
pip install pybullet matplotlib numpy
git clone https://github.com/eleramp/pybullet-object-models.git # inside the irobman_project folder
pip install -e pybullet-object-models/
```

## Task 1 (Control)

Implement an IK-solver for the Franka-robot. You can use the pseudoinverse or the transpose based solution. Use Your IK-solver to move the robot to a certain goal position. This Controller gets used throughout the project (e.g. executing the grasp - moving the object to the goal). It is not allowed to use the build in pybullet methods `calculateInverseDynamics/calculateInverseKinematics`. Feel free to use `calculateJacobian` from pybullet.

## Task 2 (Grasping)

Now that you have implemented a controller (with your IK solver) and tested it properly, it is time to put that to good use. From picking up objects to placing them a good well-placed grasp is essential. Hence, given an object you have to design a system that can effectively grasp it. You can use the model from the ![Grasping exercise](https://github.com/iROSA-lab/GIGA) and ![colab](https://colab.research.google.com/drive/1P80GRK0uQkFgDbHzLjwahyJOalW4M5vU?usp=sharing) to sample a grasp from a pointcloud. We have added a camera, where you can specify its position. You can set the YCB object to a fixed one (e.g. a Banana) for development. Showcase your ability to grasp random objects
for the final submission.

## Task 3 (Localization & Tracking)

After you have grasped the object you want to place it in the goal-basket. In order to avoid the obstacles (red spheres), you need to track them. Use the provided fixed camera and your custom-positioned cameras as sensors to locate and track the obstacles. Visualize your tracking capabilities in the Report (optional) and use this information to avoid collision with them in the last task. You could use a Kalman Filter (e.g. from Assignment 2).

## Task 4 (Planning)

After you have grasped the YCB object and localized the obstacle, the final task is to plan the robot’s movement in order to place the object in the goal basket. Implement a dynamic Planner and execute it with your controller. Once you are above the goal-basket open the gripper to drop the object in the goal.

