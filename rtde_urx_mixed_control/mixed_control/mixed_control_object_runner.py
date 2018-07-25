import mixed_control_object
import time

def generate_traj_there():
    list = []
    for j in range(0, 800):
        z = 0.000002 * j * j
        y = -1.80 + (0.001 * j)
        list.append([z, y, 0, 0, 0, 0])
    return list

def generate_traj_back():
    list = []
    for j in range(-800, 0):
        z = 0.000002 * j * j
        y = -1.80 + (0.001 * abs(j))
        list.append([z, y, 0, 0, 0, 0])
    return list
robot = mixed_control_object.mixedControlObject("192.168.12.50 ")
time.sleep(2)
robot.followTrajectory(generate_traj_there())
time.sleep(2)
robot.followTrajectory(generate_traj_back())
time.sleep(2)
robot.followTrajectory(generate_traj_there())
time.sleep(2)
robot.followTrajectory(generate_traj_back())
robot.disconnect()
