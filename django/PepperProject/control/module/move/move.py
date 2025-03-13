import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from mymap import DIRECTIONS, pepper_position, pepper_direction
from collections import deque
import math
import time

movement_commands = [
            "stop", "avancer", "reculer", "aller à gauche", "aller à droite",
            "avancer vers gauche", "avancer vers droite", "reculer vers gauche", "reculer vers droite",
            "tourner à gauche", "tourner à droite", "demi-tour gauche", "demi-tour droit", "parler"
        ]

def move_arms(pub_joints, left_angle, right_angle):
    """
    Move the robot's arms to simulate human-like walking.
    
    Args:
        pub_joints: Publisher for joint commands
        left_angle: Angle for the left arm (in radians)
        right_angle: Angle for the right arm (in radians)
    """
    joint_msg = JointState()
    joint_msg.header.stamp = rospy.Time.now()
    
    # Define the joint names for Pepper's arms
    # These joint names might need to be adjusted based on your specific robot configuration
    joint_msg.name = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
                      'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
    
    # Set the arm positions
    # Left arm
    left_shoulder_pitch = 1.0 + left_angle  # Base angle + swing
    left_shoulder_roll = 0.2
    left_elbow_yaw = -1.0
    left_elbow_roll = -0.5
    
    # Right arm
    right_shoulder_pitch = 1.0 + right_angle  # Base angle + swing
    right_shoulder_roll = -0.2
    right_elbow_yaw = 1.0
    right_elbow_roll = 0.5
    
    joint_msg.position = [left_shoulder_pitch, left_shoulder_roll, left_elbow_yaw, left_elbow_roll,
                         right_shoulder_pitch, right_shoulder_roll, right_elbow_yaw, right_elbow_roll]
    
    pub_joints.publish(joint_msg)

def swing_arms(pub_joints, duration, rate):
    """
    Create a natural swinging motion for the arms while walking.
    
    Args:
        pub_joints: Publisher for joint commands
        duration: Duration of movement in seconds
        rate: ROS Rate object for timing
    """
    start_time = time.time()
    elapsed = 0
    
    while elapsed < duration:
        # Calculate arm swing based on a sine wave
        # This creates a natural pendulum-like motion
        swing_angle = 0.3 * math.sin(elapsed * 4)  # Amplitude * sine function
        
        # Right and left arms swing in opposite directions (180° out of phase)
        move_arms(pub_joints, swing_angle, -swing_angle)
        
        rate.sleep()
        elapsed = time.time() - start_time

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
        "demi tour gauche": {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, 1.5706], 'topic': '/cmd_vel', 'direction': 'left'},
        "demi tour droit": {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, -1.5706], 'topic': '/cmd_vel', 'direction': 'right'},
    }

    command = naoqi_commands if driver == 'naoqi_driver' else dcm_commands

    if command_name not in command:
        rospy.logerr(f"Commande {command_name} non trouvée pour le driver {driver}")
        return pepper_position  

    rospy.init_node('pepper_movement', anonymous=True, disable_signals=True)

    # Publisher for movement commands
    pub_move = rospy.Publisher(command[command_name]['topic'], Twist, queue_size=10)
    
    # Publisher for joint state commands (for arm movement)
    pub_joints = rospy.Publisher('/joint_states', JointState, queue_size=10)
    
    move_cmd = Twist()
    move_cmd.linear.x = command[command_name]['linear'][0]
    move_cmd.linear.y = command[command_name]['linear'][1]
    move_cmd.linear.z = command[command_name]['linear'][2]
    move_cmd.angular.x = command[command_name]['angular'][0]
    move_cmd.angular.y = command[command_name]['angular'][1]
    move_cmd.angular.z = command[command_name]['angular'][2]
    
    rate = rospy.Rate(10)
    time_elapsed = 0

    # Check if this is a movement command that should have arm swinging
    needs_arm_swing = (
        "avancer" in command_name or 
        "reculer" in command_name or 
        "aller à" in command_name
    )

    # Run both movement and arm swinging simultaneously
    if needs_arm_swing:
        # Initialize arm position
        move_arms(pub_joints, 0, 0)
        
        start_time = time.time()
        while time.time() - start_time < duration:
            # Publish movement command
            pub_move.publish(move_cmd)
            
            # Calculate arm swing based on a sine wave
            elapsed = time.time() - start_time
            swing_angle = 0.3 * math.sin(elapsed * 4)
            
            # Arms swing in opposite directions
            if "reculer" in command_name:
                # Reverse arm swing for backward movement
                move_arms(pub_joints, -swing_angle, swing_angle)
            else:
                move_arms(pub_joints, swing_angle, -swing_angle)
            
            rate.sleep()
    else:
        # For commands like turning, just execute the movement without arm swinging
        while time_elapsed < duration:
            pub_move.publish(move_cmd)
            
            # For turning commands, set arms in a stable position
            if "tourner" in command_name or "demi-tour" in command_name:
                # Slightly raised arms for balance during turning
                move_arms(pub_joints, -0.1, -0.1)
            
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
        
        # Return arms to neutral position when stopping
        if time_elapsed < 0.3:
            # Gradually return arms to neutral position
            move_arms(pub_joints, 0, 0)
        
        time_elapsed += 0.1
        rate.sleep()
    
    return pepper_position