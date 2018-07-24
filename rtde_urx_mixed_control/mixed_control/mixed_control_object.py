

import sys
import logging
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import urx.urscript
import urx.robot
import time


class mixedControlObject:
    def __init__(self, ip):
        self.ROBOT_IP = ip
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

        self.urxRobot = None

        self.traj = []

    def connectRTDE(self):
        try:

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
            self.setp.input_double_register_6 = 0

            #self.watchdog.input_int_register_0 = 0
            self.con.send_start()

            return True
        except:
            print("RTDE failed to connect!")
            return False

    def connectURX(self):
        try:
            self.urxRobot = urx.Robot(self.ROBOT_IP)
            print("URX connected!")
            return True
        except:
            print("URX failed to connect!")
            return False


    def __add_multiple_to_traj(self,new_setps):
        for new_setp in new_setps:
            self.__add_to_traj(new_setp)


    def __add_to_traj(self,new_setp):
        list = [0,0,0,0,0,0,0]
        list[0] = new_setp[0]
        list[1] = new_setp[1]
        list[2] = new_setp[2]
        list[3] = new_setp[3]
        list[4] = new_setp[4]
        list[5] = new_setp[5]
        list[6] = len(self.traj)
        self.traj.append(list)

    def start_RTDE_servo_listener(self):
        file = open("rtde_control_servo.script","r")
        prog = urx.urscript.URScript()
        for line in file:
            prog.add_line_to_program(line)
        if not self.urxRobot.is_running():
            self.connectURX()
        self.urxRobot.send_program(prog.__call__())
        time.sleep(0.1)
        self.urxRobot.close()
        if  self.con is None:
            self.connectRTDE()
        elif not self.con.is_connected():
            self.con.send_start()

    def __run_traj(self):
        try:
            # control loop
            while len(self.traj) > 0:
                # receive the current state
                state = self.con.receive()

                # do something...
                if state is not None and state.output_int_register_0 != 0:


                        newSetp = self.traj.pop(0)
                        #self.setp = self.__list_to_setp(self.setp, newSetp)
                        for i in range(0, 7):
                            self.setp.__dict__["input_double_register_%i" % i] = newSetp[i]

                        # send new setpoint
                        self.con.send(self.setp)

                # kick watchdog
                #self.con.send(self.watchdog)



            return True
        except:
            return False
    def followTrajectory(self, trajectory):
        self.__add_multiple_to_traj(trajectory)
        if self.urxRobot is None or not self.urxRobot.is_running():
            i = 0
            while not self.connectURX() and i < 5:
                i = i + 1
                time.sleep(.1)
            time.sleep(.1)
        # are we where we are supposed to start? If not move there first
        ##if self.traj[0] != self.getj():
            ##self.urxRobot.movej(self.traj[0], acc = 0.3, wait = False)
          #  threshold = .05
           # while ((abs(self.getj()[0] - self.traj[0][0]) >= threshold) or (abs(self.getj()[1] - self.traj[0][1]) >= threshold) \
            #       or (abs(self.getj()[2] - self.traj[0][2]) >= threshold) or (abs(self.getj()[3] - self.traj[0][3]) >= threshold) \
             #      or (abs(self.getj()[4] - self.traj[0][4]) >= threshold) or (abs(self.getj()[5] - self.traj[0][5]) >= threshold)) :
              #  time.sleep(.25)
        self.start_RTDE_servo_listener()
        value = self.__run_traj()
        return value

    def disconnect(self):
        self.con.send_pause()
        self.con.disconnect()

    def getj(self):
        return self.urxRobot.getj(())

    def get_tool_pos(self):
        return self.urxRobot.get_pos()

    def get_tool_orietnatiopn(self):
        return self.urxRobot.get_orientation()