import os
import sys
import glob
import time
import math as m
import numpy as np
import pybullet as p
import pybullet_data
from enum import Enum
from robot import Robot
from objects import Obstacle, Table, Box, YCBObject, Goal
from pybullet_object_models import ycb_objects
from typing import Optional
import matplotlib.pyplot as plt

GRAVITY = -9.8

class Camera(Enum):
    FIXEDCAM = 1
    CUSTOMCAM = 2


class Simulation:
    """Simulation Class.

    The class initializes a franka robot with static and moving
    obstacles on a large table .

    Args:
        timestep: Simulation timestep.
        with_gui: Simulation mode (with or without GUI).
        camera_width: Camera image width.
        camea_height: Camera image height.
        cam_pose: Custom Camera placement position.
        target_pose: Custom Camera look-at position.
        orientation: Robot orientation in axis angle representation.
        table_scaling: Scaling parameter for the table.
        target_object: To select which object to grasp (cube + YCB available).
        randomize: Randomizes between the set of YCB objects.
    """
    def __init__(self,
                 timestep: float = (1/240.),
                 with_gui: bool = True,
                 camera_width: int = 256,
                 camera_height: int = 256,
                 cam_pose: np.ndarray = np.array([2.5, 0, 2.0]),
                 target_pose: np.ndarray = np.array([1.0, 0, 1.7]),
                 target_object: Optional[str] = "cube",
                 randomize: bool = False):
        # settings here
        self.g = GRAVITY
        self.timestep = timestep
        self.mode = p.GUI if with_gui else p.DIRECT
        self.target_object = target_object
        self.randomize = randomize

        # camera
        self.width, self.height = camera_width, camera_height
        # change to get a new view
        self.cam_pose = cam_pose
        self.target_pose = target_pose
        # internal
        self.cam_matrices = dict()
        self.cam_matrices[Camera.FIXEDCAM] = self.set_camera(np.array([2.5, 0, 2.0]),
                                                             np.array([1.0, 0, 1.7]))
        self.cam_matrices[Camera.CUSTOMCAM] = self.set_camera(self.cam_pose,
                                                              self.target_pose)

        self.box = self.table = self.robot = self.goal = self.wall = None
        # initial object position
        self.initial_position = (0.0, -0.65, 1.24)
        # starting the simulation
        self.start()

    def start(self):
        p.connect(self.mode)
        p.setGravity(0, 0, self.g)
        p.setTimeStep(self.timestep)
        p.setRealTimeSimulation(100)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # add objects
        p.loadURDF("plane.urdf")
        self.wall = p.loadURDF("plane_transparent.urdf", [-0.6, 0., 0.],
                               p.getQuaternionFromEuler([0, m.pi/2, 0]))
        self.table = Table()
        self.robot = Robot()
        self.goal = Goal(position=(0.65, 0.8, 1.24))

        if not self.randomize:
            if self.target_object == "cube":
                self.box = Box(position=self.initial_position)
            else:
                self.box = YCBObject(obj_name=self.target_object,
                                     position=self.initial_position)

        else:
            object_root_path = ycb_objects.getDataPath()
            files = glob.glob(os.path.join(object_root_path, "Ycb*"))
            obj_names = [file.split('/')[-1] for file in files]
            selected_object = obj_names[np.random.randint(0, len(obj_names))]
            self.box = YCBObject(obj_name=selected_object,
                                 position=self.initial_position)

        self._add_obstacles()

    def _add_obstacles(self):
        self.obstacles = []
        scales = [0.3, 0.2]
        planes = [((0.4, 0.9), (0.7, 1.0), (1.5, 2.0)),
                  ((0.4, 0.9), (0.5, 0.7), (1.5, 2.0))]
        for i, (plane, scale) in enumerate(zip(planes, scales)):
            self.obstacles.append(Obstacle(plane=plane,
                                           scale=scale,
                                           flip_index=i))

    def stop_obstacles(self):
        # for developing
        for obstacle in self.obstacles:
            obstacle.stop()

    def set_camera(self, camera_pose: np.ndarray, target_pose: np.ndarray):
        viewMat, projMat = self._compute_vision_matrixes(camera_pose, target_pose)
        return (viewMat, projMat)

    def _compute_vision_matrixes(self, camera_pose, target_pose):
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=camera_pose,
            cameraTargetPosition=target_pose,
            cameraUpVector=np.array([0, 0, 1]))

        projection_matrix = p.computeProjectionMatrixFOV(
            fov=70.0,
            aspect=1.0,
            nearVal=0.05,  # 0.05 5
            farVal=5.0)

        return view_matrix, projection_matrix

    def get_renders(self, cam_type: Camera = Camera.FIXEDCAM,
                    debug: bool = False):
        # here are some robot specific params
        # better way than p. get some
        # like self.physics_client.getCameraImage
        viewMat, projMat = self.cam_matrices[cam_type]
        _, _, rgbpx, depthpx, _ = p.getCameraImage(
            width=self.width, height=self.height, viewMatrix=viewMat,
            projectionMatrix=projMat, renderer=p.ER_TINY_RENDERER)  # ER_BULLET_HARDWARE_OPENGL)

        rgbpx = np.reshape(rgbpx, [self.width, self.height, 4])  # RGBA - channel Range [0-255]
        depthpx = np.reshape(depthpx, [self.width, self.height])  # Depth Map Range [0.0-1.0]
        # For debugging
        if debug:
            plt.imsave("cam_img.png", rgbpx[..., :3].astype(np.uint8))
            plt.imsave("depth_img.png", np.floor(depthpx*255))
        return rgbpx, depthpx

    def step(self):
        for obstacle in self.obstacles:
            contact_points = p.getContactPoints(self.robot.id, obstacle.id)
            wall_points = p.getContactPoints(self.robot.id, self.wall)
            if ((len(contact_points) > 0) or (len(wall_points) > 0)):
                print("ERROR! Robot in Collision")
                sys.exit()
            obstacle.move()
        p.stepSimulation()
        time.sleep(1./240.)

    def get_robot(self):
        return self.robot

