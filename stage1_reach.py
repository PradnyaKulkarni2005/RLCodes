# stage1_reach.py

import pybullet as p
import pybullet_data
import time
from utils.ik_utils import move_to_pose

def run_stage():
    # Start physics client with GUI
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load plane and robot
    plane_id = p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("robot/so100.urdf", useFixedBase=True)

    # Let things settle
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)

    # Define end effector link index manually (change based on your robot)
    end_effector_index = 5
    print("End effector index:", end_effector_index)

    # Define target position in world coordinates
    target_position = [0.4, 0.0, 0.3]  # X, Y, Z

    # Visual marker to verify target
    p.addUserDebugText("Target", target_position, [1, 0, 0], textSize=1.5)
    p.addUserDebugLine([0, 0, 0], target_position, [1, 0, 0], 2)

    # Move robot using IK
    move_to_pose(robot_id, end_effector_index, target_position)

    print("End of Stage 1")

    # Wait before shutting down
    time.sleep(2)
    p.disconnect()
