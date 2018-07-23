

import sys
import logging
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import urx.urscript
import urx.robot

#logging.basicConfig(level=logging.INFO)

ROBOT_HOST = '192.168.12.248'
ROBOT_PORT = 30004
config_filename = 'control_loop_configuration.xml'

keep_running = True

urxRob = urx.Robot(ROBOT_HOST)

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe('state')
setp_names, setp_types = conf.get_recipe('setp')
watchdog_names, watchdog_types = conf.get_recipe('watchdog')

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0

# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
watchdog.input_int_register_0 = 0



traj = []


def add_multiple_to_traj(new_setps):
    for new_setp in new_setps:
        add_to_traj(new_setp)


def add_to_traj(new_setp):
    list = [0,0,0,0,0,0]
    list[0] = new_setp[0]
    list[1] = new_setp[1]
    list[2] = new_setp[2]
    list[3] = new_setp[3]
    list[4] = new_setp[4]
    list[5] = new_setp[5]
    traj.append(list)

def setp_to_list(setp):
    list = []
    for i in range(0,6):
        list.append(setp.__dict__["input_double_register_%i" % i])
    return list

def list_to_setp(setp, list):
    for i in range (0,6):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp

def start_RTDE_servo_listener():
    file = open("rtde_control_servo.script","r")
    prog = urx.urscript.URScript()
    for line in file:
        prog.add_line_to_program(line)
    urxRob.send_program(prog.__call__())

def generate_traj():
    list = []
    for j in range(0, 800):
        z = 0.000002 * j * j
        y = -1.80 + (0.001 * j)
        list.append([z, y, 0, 0, 0, 0])
    return list

add_multiple_to_traj(generate_traj())

#start data synchronization
if not con.send_start():
    sys.exit()

start_RTDE_servo_listener()

# control loop
while keep_running:
    # receive the current state
    state = con.receive()
    
    if state is None:
        break

    # do something...
    if state.output_int_register_0 != 0:

        if len(traj) > 0:
            newSetp = traj.pop(0)
            setp = list_to_setp(setp, newSetp)

            # send new setpoint
            con.send(setp)

    # kick watchdog
    con.send(watchdog)

con.send_pause()

con.disconnect()
