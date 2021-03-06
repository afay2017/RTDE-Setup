
import sys
sys.path.append('..')
import logging
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import urx.urscript
import urx.robot



ROBOT_HOST = '192.168.12.248'
ROBOT_PORT = 30004
config_filename = 'control_loop_configuration.xml'

keep_running = True

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

urxRob = urx.Robot("192.168.12.248")

def setp_to_list(setp):
    list = []
    for i in range(0, 6):
        list.append(setp.__dict__["input_double_register_%i" % i])
    return list

def list_to_setp(setp, list):
    for i in range(0, 6):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp

def start_RTDE_servo_listener():
    file = open("rtde_control_servo.script","r")
    prog = urx.urscript.URScript()
    for line in file:
        prog.add_line_to_program(line)

    urxRob.send_program(prog.__call__())

#start data synchronization
if not con.send_start():
    sys.exit()

start_RTDE_servo_listener()

# control loop
log = open("controlLog.txt", "w")
j = 0
while keep_running:
    # receive the current state
    state = con.receive()
    
    if state is None:
        break

    # do something...
    if state.output_int_register_0 != 0:

        # generate dummy points
        z = 0.000002 * j * j
        y = -1.80 + 0.001 * j
        newSetp = [z,y,0,0,0,0]
        setp = list_to_setp(setp, newSetp)
        j = j + 1

        # send new setpoint
        con.send(setp)

    if j >= 800:
        break
    # kick watchdog
    con.send(watchdog)

con.send_pause()

con.disconnect()
