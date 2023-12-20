import os
import glob
import math
import random as r

import numpy as np
import pybullet as p
import pybullet_data
from pybullet_object_models import ycb_objects
from typing import Tuple

OFFSET_CONSTANT = 0.07
TABLE_SCALING = 2.0
OBJECT_SCALING = 0.1


class Obstacle:
    """General Obstacle Class.

    Creates Moving Obstacles for simulation.

    Args:
        plane: Object Plane (X-range, Y-range, Z-range)
        scale: Obstacle scaling
        axis: Axis of movement of obstacle
    """
    def __init__(
            self,
            plane: Tuple[Tuple[float, float],
                         Tuple[float, float],
                         Tuple[float, float]],
            scale: float = 0.3,
            flip_index: int = 0):
        self.point_plane = plane
        self.point_id = 0
        self.flip_index = flip_index
        self.ori = p.getQuaternionFromEuler([0, 0, 0])
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # check proper path
        start_pos = self.get_next_goal_point()

        self.stopped = False
        self.scaling = scale
        self.id = p.loadURDF("sphere2red.urdf", start_pos, self.ori,
                             globalScaling=self.scaling, useFixedBase=True)

        self.current_goal = self.get_next_goal_point()
        self.current_goal = np.clip(self.current_goal,
                                    np.array([-0.5, -1.5, -1.1]),
                                    np.array([1.5, 1.5, 2.0]))
        self.step = 0.005

    def get_next_goal_point(self):
        point = [0, 0, 0]
        for i in range(len(point)):
            point[i] = r.uniform(self.point_plane[i][0], self.point_plane[i][1])

        if self.point_id % 2 == 0:
            point[self.flip_index] = -point[self.flip_index]

        self.point_id += 1
        return point

    def stop(self):
        self.stopped = True

    def no_stop(self):
        self.stopped = False

    def move(self):
        if not self.stopped:
            pos = self.get_pos()
            # update new goal position
            if math.dist(pos, self.current_goal) < 0.05:
                self.current_goal = self.get_next_goal_point()
                self.current_goal = np.clip(self.current_goal,
                                    np.array([-0.5, -1.5, -1.1]),
                                    np.array([1.5, 1.5, 2.0]))
            direc = np.array([self.current_goal[i] - pos[i] for i in range(len(pos))])
            direc = direc / np.linalg.norm(direc)
            # adding noise
            noise = np.random.normal(0, 0.5, len(direc))
            direc_noisy = direc + noise

            new_pos = pos + self.step * direc_noisy
            p.resetBasePositionAndOrientation(self.id, new_pos, self.ori)
        else:
            pass

    def get_pos(self):
        pos, _ = p.getBasePositionAndOrientation(self.id)
        return pos


class Box:
    """Box Obstacle Class.

    Args:
        position: Set cube position
    """
    def __init__(self, position: Tuple[float, float, float] = (1.0, 0, 1.31)):
        table_scaling = TABLE_SCALING
        pos = [1.0, 0, 0.62 * table_scaling + OFFSET_CONSTANT]
        ori = p.getQuaternionFromEuler([0, 0, 0])
        scaling = OBJECT_SCALING
        self.id = p.loadURDF("cube.urdf", pos, ori,
                   globalScaling=scaling)
        p.changeDynamics(self.id,
                         0,
                         lateralFriction=10.0,
                         spinningFriction=1.0,
                         rollingFriction=1.0,
                         frictionAnchor=True)


class Table:
    """Table Obstacle Class."""
    def __init__(self):
        table_pos = [0.5, 0, 0]
        table_ori = p.getQuaternionFromEuler([0, 0, 90 * (math.pi/180)])
        scaling = TABLE_SCALING
        self.id = p.loadURDF("table/table.urdf", table_pos, table_ori,
                   globalScaling=scaling)


class Goal:
    """Goal Object Class

    Spawns a tray for  the goal position.
    Args:
        position: The goal position.
    """
    def __init__(self, position: Tuple[float, float, float]):
        self.goal_pos = position
        self.goal_ori = p.getQuaternionFromEuler([0, 0, 90 * (math.pi/180)])
        self.id = p.loadURDF("tray/traybox.urdf", self.goal_pos, self.goal_ori,
                             globalScaling=1.0)


class YCBObject:
    """YCB Object Class.

    Args:
        obj_name: Name of the YCB object.
        position: Position of the object spawn.
        orientation: Orientation of the object spawn.
    """
    def __init__(self,
                 obj_name: str,
                 position: Tuple[float, float, float] = (1.0, 0, 1.31),
                 orientation: Tuple[float, float, float] = (0, 0, 0)):
        self.obj_name = obj_name
        self.pos = position
        self.ori = p.getQuaternionFromEuler(orientation)
        scaling = 1.5  # 1.5 times normal
        object_root_path = ycb_objects.getDataPath()
        files = glob.glob(os.path.join(object_root_path, "Ycb*"))
        obj_names = [file.split('/')[-1] for file in files]
        if not (self.obj_name in obj_names):
            raise ValueError(f"Object not Available pick from : {obj_names}")
        path_to_urdf = os.path.join(object_root_path, self.obj_name, "model.urdf")
        self.id = p.loadURDF(path_to_urdf, self.pos, self.ori,
                   globalScaling=scaling)
        p.changeDynamics(self.id,
                         0,
                         lateralFriction=10.0,
                         spinningFriction=1.0,
                         rollingFriction=1.0,
                         frictionAnchor=True)


if __name__ == "__main__":
    object_root_path = ycb_objects.getDataPath()
    files = glob.glob(os.path.join(object_root_path, "Ycb*"))
    obj_names = [file.split('/')[-1] for file in files]
    print(obj_names)
    obj_name = 'YcbBanana' # name of the object folder
    path_to_urdf = os.path.join(ycb_objects.getDataPath(), obj_name, "model.urdf")
    print(path_to_urdf)
