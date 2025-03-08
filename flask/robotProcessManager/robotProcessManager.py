import subprocess
import signal
import os

class RobotProcessManager:
    def __init__(self):
        self.naoqi_process = None
        self.pepper_process = None
        self.current_robot_ip = None
        self.current_network_interface = None

    def start(self, nao_ip, network_interface):
        """Start both naoqi_driver and pepper_dcm_bringup at the same time."""
        self.stop()  # Stop any existing processes

        # Store current connection details
        self.current_robot_ip = nao_ip
        self.current_network_interface = network_interface

        # Start naoqi_driver
        try:
            naoqi_driver_command = [
                "roslaunch", "naoqi_driver", "naoqi_driver.launch", 
                f"nao_ip:={nao_ip}", 
                "roscore_ip:=localhost", 
                f"network_interface:={network_interface}"
            ]
            
            self.naoqi_process = subprocess.Popen(
                naoqi_driver_command, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Allows killing entire process group
            )
            print(f"Naoqi driver started for robot {nao_ip}")
        except Exception as e:
            print(f"Error starting naoqi driver: {e}")

        # Start pepper_dcm_bringup
        try:
            pepper_dcm_command = [
                "roslaunch", "pepper_dcm_bringup", "pepper_bringup.launch", 
                f"robot_ip:={nao_ip}", 
                "roscore_ip:=localhost", 
                f"network_interface:={network_interface}"
            ]
            
            self.pepper_process = subprocess.Popen(
                pepper_dcm_command, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Allows killing entire process group
            )
            print(f"Pepper DCM bringup started for robot {nao_ip}")
        except Exception as e:
            print(f"Error starting pepper_dcm_bringup: {e}")

    def stop_naoqi_driver(self):
        if self.naoqi_process:
            try:
                os.killpg(os.getpgid(self.naoqi_process.pid), signal.SIGINT)
                self.naoqi_process = None
                print("Naoqi driver stopped successfully")
            except Exception as e:
                print(f"Error stopping naoqi driver: {e}")

    def stop_pepper_dcm_bringup(self):
        if self.pepper_process:
            try:
                os.killpg(os.getpgid(self.pepper_process.pid), signal.SIGINT)
                self.pepper_process = None
                print("Pepper DCM bringup stopped successfully")
            except Exception as e:
                print(f"Error stopping pepper_dcm_bringup: {e}")

    def stop(self):
        """Stop both naoqi_driver and pepper_dcm_bringup."""
        self.stop_naoqi_driver()
        self.stop_pepper_dcm_bringup()
        print("All processes stopped.")