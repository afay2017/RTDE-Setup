import mixed_control_object
import time

def generate_traj():
    list = []
    for j in range(0, 800):
        z = 0.000002 * j * j
        y = -1.80 + (0.001 * j)
        list.append([z, y, 0, 0, 0, 0])
    return list

robot = mixed_control_object.mixedControlObject()
robot.connect("192.168.12.248")
##time.sleep(2)
robot.followTrajectory(generate_traj())
time.sleep(2)
robot.followTrajectory(generate_traj())
robot.disconnect()
