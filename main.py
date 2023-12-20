import numpy as np
from simulation import Simulation, Camera

sim = Simulation(cam_pose=np.array([0, -2.5, 2.0]),
                 target_pose=np.array([1.0, 0, 1.7]),
                 target_object="YcbBanana",
                 randomize=False)

robot = sim.get_robot()

robot.print_joint_infos()

rgb, depth = sim.get_renders(cam_type=Camera.FIXEDCAM)
# rgb, depth = sim.get_renders(cam_type=Camera.FIXEDCAM, debug=True)

for _ in range(10000):
    # robot.do_something()
    sim.step()

