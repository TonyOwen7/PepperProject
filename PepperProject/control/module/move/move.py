#!/usr/bin/env python
# movement_controller.py

# import rospy
# from geometry_msgs.msg import Twist
import sys
import time
from mymap import  pepper_position, pepper_direction

def move(driver, command_name, duration=None):
    if duration is None:
        duration = 0.5

    naoqi_commands = {
        "stop": {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/cmd_vel', 'direction': 'stop'},
        "avancer": {'linear': [0.5, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/cmd_vel', 'direction': 'down'},
        "reculer": {'linear': [-0.5, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/cmd_vel', 'direction': 'up'},
        "aller à gauche": {'linear': [0.0, 0.5, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/cmd_vel', 'direction': 'left'},
        "aller à droite": {'linear': [0.0, -0.5, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/cmd_vel', 'direction': 'right'},
        "avancer vers gauche": {'linear': [0.5, 0.5, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/cmd_vel', 'direction': 'down_left'},
        "avancer vers droite": {'linear': [0.5, -0.5, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/cmd_vel', 'direction': 'down_right'},
        "reculer vers gauche": {'linear': [-0.5, 0.5, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/cmd_vel', 'direction': 'up_left'},
        "reculer vers droite": {'linear': [-0.5, -0.5, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/cmd_vel', 'direction': 'up_right'},
        "tourner à gauche": {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, 0.7854], 'topic': '/cmd_vel', 'direction': 'left'},
        "tourner à droite": {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, -0.7854], 'topic': '/cmd_vel', 'direction': 'right'},
        "demi-tour gauche": {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, 1.5706], 'topic': '/cmd_vel', 'direction': 'left'},
        "demi-tour droit": {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, -1.5706], 'topic': '/cmd_vel', 'direction': 'right'},
    }

    dcm_commands = {
        "stop": {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/pepper_dcm/cmd_moveto', 'direction': 'stop'},
        "avancer": {'linear': [0.5, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/pepper_dcm/cmd_moveto', 'direction': 'down'},
        "reculer": {'linear': [-0.5, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/pepper_dcm/cmd_moveto', 'direction': 'up'},
        "aller à gauche": {'linear': [0.0, 0.5, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/pepper_dcm/cmd_moveto', 'direction': 'left'},
        "aller à droite": {'linear': [0.0, -0.5, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/pepper_dcm/cmd_moveto', 'direction': 'right'},
        "avancer vers gauche": {'linear': [0.5, 0.5, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/pepper_dcm/cmd_moveto', 'direction': 'down_left'},
        "avancer vers droite": {'linear': [0.5, -0.5, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/pepper_dcm/cmd_moveto', 'direction': 'down_right'},
        "reculer vers gauche": {'linear': [-0.5, 0.5, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/pepper_dcm/cmd_moveto', 'direction': 'up_left'},
        "reculer vers droite" : {'linear': [-0.5, -0.5, 0.0], 'angular': [0.0, 0.0, 0.0], 'topic': '/pepper_dcm/cmd_moveto', 'direction': 'up_right'},
        'tourner à gauche': {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, 0.7854], 'topic': '/pepper_dcm/cmd_moveto', 'direction': 'left'},
        'tourner à droite': {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, -0.7854], 'topic': '/pepper_dcm/cmd_moveto', 'direction': 'right'},
        "demi-tour gauche": {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, 1.5706], 'topic': '/cmd_vel', 'direction': 'left'},
        "demi-tour droit": {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, -1.5706], 'topic': '/cmd_vel', 'direction': 'right'},
    }

    command = naoqi_commands if driver == 'naoqi_driver' else dcm_commands

    if command_name not in command:
        rospy.logerr(f"Commande {command_name} non trouvée pour le driver {driver}")
        return pepper_position

    rospy.init_node('pepper_movement', anonymous=True, disable_signals=True)

    # Publisher for movement commands
    pub_move = rospy.Publisher(command[command_name]['topic'], Twist, queue_size=10)
    
    move_cmd = Twist()
    move_cmd.linear.x = command[command_name]['linear'][0]
    move_cmd.linear.y = command[command_name]['linear'][1]
    move_cmd.linear.z = command[command_name]['linear'][2]
    move_cmd.angular.x = command[command_name]['angular'][0]
    move_cmd.angular.y = command[command_name]['angular'][1]
    move_cmd.angular.z = command[command_name]['angular'][2]
    
    rate = rospy.Rate(10)
    time_elapsed = 0

    while time_elapsed < duration:
        pub_move.publish(move_cmd)
        time_elapsed += 0.1
        rate.sleep()

    # Stop movement
    move_cmd.linear.x = 0
    move_cmd.linear.y = 0
    move_cmd.linear.z = 0
    move_cmd.angular.x = 0
    move_cmd.angular.y = 0
    move_cmd.angular.z = 0

    time_elapsed = 0

    while time_elapsed < 0.5:
        pub_move.publish(move_cmd)
        time_elapsed += 0.1
        rate.sleep()
    

