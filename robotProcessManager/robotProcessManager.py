import subprocess
import signal
import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from collections import deque

import sys
import os

module_path = os.path.join(os.path.dirname(__file__), '../move')
sys.path.append(module_path)

from move import execute_move

class RobotProcessManager:
    def __init__(self):
        self.roscore_process = None
        self.process = None
        self.current_robot_ip = None
        self.current_network_interface = None
        self.current_driver = None


    def start_roscore(self):     
        if self.roscore_process is None:
            try:
                self.roscore_process = subprocess.Popen(
                    ["roscore"], 
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE
                )
                print("Roscore started successfully")
            except Exception as e:
                print(f"Error starting roscore: {e}")

    def stop_roscore(self):
        if self.roscore_process:
            try:
                os.killpg(os.getpgid(self.roscore_process.pid), signal.SIGINT)
                self.roscore_process = None
                print("Roscore stopped successfully")
            except Exception as e:
                print(f"Error stopping roscore: {e}")
                

    def start_naoqi_driver(self, robot_ip, network_interface):
        self.stop_naoqi_driver()
        
        # Store current connection details
        self.current_robot_ip = robot_ip
        self.current_network_interface = network_interface
        
        try:
            naoqi_driver_command = [
                "roslaunch", "naoqi_driver", "naoqi_driver.launch", 
                f"nao_ip:={robot_ip}", 
                "roscore_ip:=localhost", 
                f"network_interface:={network_interface}"
            ]
            
            self.process = subprocess.Popen(
                naoqi_driver_command, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Allows killing entire process group
            )
            print(f"Naoqi driver started for robot {robot_ip}")
        except Exception as e:
            print(f"Error starting naoqi driver: {e}")

    def stop_naoqi_driver(self):
        if self.process:
            try:
                os.killpg(os.getpgid(self.naoqi_driver_process.pid), signal.SIGINT)
                self.naoqi_driver_process = None
                print("Naoqi driver stopped successfully")
            except Exception as e:
                print(f"Error stopping naoqi driver: {e}")


    def start_pepper_dcm_bringup(self, robot_ip, network_interface):
        self.stop_pepper_dcm_bringup()
        
        self.current_robot_ip = robot_ip
        self.current_network_interface = network_interface
        
        try:
            naoqi_driver_command = [
                "roslaunch", "pepper_dcm_bringup", "pepper_bringup.launch", 
                f"robot_ip:={robot_ip}", 
                "roscore_ip:=localhost", 
                f"network_interface:={network_interface}"
            ]
            
            self.process = subprocess.Popen(
                naoqi_driver_command, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Allows killing entire process group
            )
            print(f"pepper_dcm_bringup started for robot {robot_ip}")
        except Exception as e:
            print(f"Error starting pepper_dcm_bringup: {e}")

    def stop_pepper_dcm_bringup(self):
        if self.process:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
                self.process = None
                print("pepper_dcm_bringup stopped successfully")
            except Exception as e:
                print(f"Error stopping pepper_dcm_bringup: {e}")
