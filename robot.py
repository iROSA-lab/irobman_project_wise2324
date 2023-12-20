import pybullet as p
import pybullet_robots
import numpy as np
from typing import Tuple


class Robot:
    """Robot Class.

    The class initializes a franka robot.

    Args:
        init_position: Initial position of the robot.
        orientation: Robot orientation in axis angle representation.
        table_scaling: Scaling parameter for the table.
    """
    def __init__(self,
                 init_position: Tuple[float, float, float] = [0, 0, 0.62],
                 orientation: Tuple[float, float, float] = [0, 0, 0],
                 table_scaling: float = 2.0):

        # load robot
        self.pos = init_position
        self.axis_angle = orientation
        self.tscale = table_scaling

        if self.tscale != 1.0:
            self.pos = [self.pos[0], self.pos[1], self.pos[2] * self.tscale]
        self.ori = p.getQuaternionFromEuler(self.axis_angle)

        self.arm_idx = [0, 1, 2, 3, 4, 5, 6]
        self.default_arm = [-1.8, 0.058, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]
        self.gripper_idx = [9, 10]

        self.ee_idx = 11

        self.id = p.loadURDF("franka_panda/panda.urdf", self.pos, self.ori,
                             useFixedBase=True)

        self.lower_limits, self.upper_limits = self.get_joint_limits()

        self.set_default_position()

        for j in range(p.getNumJoints(self.id)):
            p.changeDynamics(self.id, j, linearDamping=0, angularDamping=0)

    def set_default_position(self):
        for idx, pos in zip(self.arm_idx, self.default_arm):
            p.resetJointState(self.id, idx, pos)

    def get_joint_limits(self):
        lower = []
        upper = []
        for idx in self.arm_idx:
            joint_info = p.getJointInfo(self.id, idx)
            lower.append(joint_info[8])
            upper.append(joint_info[9])
        return lower, upper

    def print_joint_infos(self):
        num_joints = p.getNumJoints(self.id)
        print('number of joints are: {}'.format(num_joints))
        for i in range(0, num_joints):
            print('Index: {}'.format(p.getJointInfo(self.id, i)[0]))
            print('Name: {}'.format(p.getJointInfo(self.id, i)[1]))
            print('Typ: {}'.format(p.getJointInfo(self.id, i)[2]))

    def get_joint_positions(self):
        states = p.getJointStates(self.id, self.arm_idx)
        return [state[0] for state in states]

    def ee_position(self):
        ee_info = p.getLinkstate(self.id, self.ee_idx)
        ee_pos = ee_info[0]
        # ee_ori = ee_info[1]
        return ee_pos

    def position_control(self, target_positions):
        p.setJointMotorControlArray(
            self.id,
            jointIndices=self.arm_idx,
            controlMode=p.POSITION_CONTROL,
            targetPositions=target_positions,
        )
