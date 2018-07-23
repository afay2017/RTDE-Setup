

import sys
import logging
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import urx.urscript
import urx.robot
import time


class mixedControlObject:
    def __init__(self):
        ##ROBOT_IP = '192.168.12.50'
        self.RTDE_PORT = 30004
        self.config_filename = 'control_loop_configuration.xml'

        logging.getLogger().setLevel(logging.INFO)

        self.conf = None
        self.state_names = None
        self.state_types = None
        self.setp_names = None
        self.setp_types = None
        self.setp = None
        self.watchdog_names = None
        self.watchdog_types = None
        self.watchdog = None

        self.con = None

        self.connected = False

        self.urxRobot = None

        self.traj = []

    def connect(self, ip):
        try:
            self.ROBOT_IP = ip

            self.urxRobot = urx.Robot(self.ROBOT_IP)

            self.conf = rtde_config.ConfigFile(self.config_filename)
            self.state_names, self.state_types = self.conf.get_recipe('state')
            self.setp_names, self.setp_types = self.conf.get_recipe('setp')
            #self.watchdog_names, self.watchdog_types = self.conf.get_recipe('watchdog')

            self.con = rtde.RTDE(self.ROBOT_IP, self.RTDE_PORT)
            self.con.connect()

            # get controller version
            self.con.get_controller_version()

            # setup recipes
            self.con.send_output_setup(self.state_names, self.state_types)
            self.setp = self.con.send_input_setup(self.setp_names, self.setp_types)
            #self.watchdog =self.con.send_input_setup(self.watchdog_names, self.watchdog_types)

            self.setp.input_double_register_0 = 0
            self.setp.input_double_register_1 = 0
            self.setp.input_double_register_2 = 0
            self.setp.input_double_register_3 = 0
            self.setp.input_double_register_4 = 0
            self.setp.input_double_register_5 = 0

            #self.watchdog.input_int_register_0 = 0

            self.connected = True

            # start data synchronization
            if not self.con.send_start():
                sys.exit()
            return True
        except:
            return False


    def __add_multiple_to_traj(self,new_setps):
        for new_setp in new_setps:
            self.__add_to_traj(new_setp)


    def __add_to_traj(self,new_setp):
        list = [0,0,0,0,0,0]
        list[0] = new_setp[0]
        list[1] = new_setp[1]
        list[2] = new_setp[2]
        list[3] = new_setp[3]
        list[4] = new_setp[4]
        list[5] = new_setp[5]
        self.traj.append(list)

    def __setp_to_list(setp):
        list = []
        for i in range(0,6):
            list.append(setp.__dict__["input_double_register_%i" % i])
        return list

    def __list_to_setp(setp, listp):
        for i in range(0,6):
            setp.__dict__["input_double_register_%i" % i] = listp[i]
        return setp

    def start_RTDE_servo_listener(self):
        file = open("rtde_control_servo.script","r")
        prog = urx.urscript.URScript()
        for line in file:
            prog.add_line_to_program(line)
        self.urxRobot.send_program(prog.__call__())

    def __run_traj(self):
        try:
            # control loop
            while len(self.traj) > 0:
                # receive the current state
                state = self.con.receive()
                self.traj.pop(0)

                # do something...
                if state is not None and state.output_int_register_0 != 0:


                        newSetp = self.traj.pop(0)
                        urx_setp = self.setp

                        for i in range(0, 6):
                            self.setp.__dict__["input_double_register_%i" % i] = newSetp[i]

                        # send new setpoint
                        self.con.send(self.setp)

                # kick watchdog
            #    self.con.send(self.watchdog)



            return True
        except:
            return False
    def followTrajectory(self, trajectory):
        self.__add_multiple_to_traj(trajectory)
        # are we where we are supposed to start? If not move there first
        if self.traj[0] != self.getj():
            self.urxRobot.movej(self.traj[0],wait = False)
            time.sleep(1)
        self.start_RTDE_servo_listener()
        return self.__run_traj()


    def disconnect(self):
        self.con.send_pause()
        self.con.disconnect()

    def getj(self):
        return self.urxRobot.getj(())

    def get_tool_pos(self):
        return self.urxRobot.get_pos()

    def get_tool_orietnatiopn(self):
        return self.urxRobot.get_orientation()