import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from mymap import DIRECTIONS, pepper_position, pepper_direction
import math
import time
import sys
import multiprocessing

# Define movement commands
movement_commands = [
    "stop", "avancer", "reculer", "aller à gauche", "aller à droite",
    "avancer vers gauche", "avancer vers droite", "reculer vers gauche", "reculer vers droite",
    "tourner à gauche", "tourner à droite", "demi-tour gauche", "demi-tour droit"
]

def move_arms(pub_joints, left_angle, right_angle):
    """
    Move the robot's arms to simulate human-like walking.
    """
    joint_msg = JointState()
    joint_msg.header.stamp = rospy.Time.now()
    
    # Define the joint names for Pepper's arms
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

def arm_controller(command_name, duration, exit_event):
    """
    Process to control arm movements.
    """
    rospy.init_node('pepper_arm_controller', anonymous=True)
    pub_joints = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)
    
    # Determine movement type
    if "avancer" in command_name:
        movement_type = "forward"
    elif "reculer" in command_name:
        movement_type = "backward"
    elif "tourner" in command_name or "demi-tour" in command_name:
        movement_type = "turn"
    else:
        movement_type = "neutral"
    
    start_time = time.time()
    elapsed = 0
    
    # Initialize arm position
    move_arms(pub_joints, 0, 0)
    
    while elapsed < duration and not exit_event.is_set():
        if movement_type == "neutral":
            # Return to neutral position
            move_arms(pub_joints, 0, 0)
        elif movement_type == "turn":
            # Slightly raised arms for balance during turning
            move_arms(pub_joints, -0.1, -0.1)
        else:
            # Calculate arm swing based on a sine wave
            swing_angle = 0.3 * math.sin(elapsed * 4)  # Amplitude * sine function
            
            # Right and left arms swing in opposite directions
            if movement_type == "backward":
                # Reverse arm swing for backward movement
                move_arms(pub_joints, -swing_angle, swing_angle)
            else:  # forward
                move_arms(pub_joints, swing_angle, -swing_angle)
        
        rate.sleep()
        elapsed = time.time() - start_time
    
    # Return to neutral position at end
    move_arms(pub_joints, 0, 0)

def movement_controller(driver, command_name, duration, exit_event):
    """
    Process to control robot movement.
    """
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
        print(f"Commande {command_name} non trouvée pour le driver {driver}")
        exit_event.set()
        return

    rospy.init_node('pepper_movement', anonymous=True)

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
    start_time = time.time()
    elapsed = 0

    while elapsed < duration and not exit_event.is_set():
        pub_move.publish(move_cmd)
        rate.sleep()
        elapsed = time.time() - start_time

    # Stop movement
    move_cmd.linear.x = 0
    move_cmd.linear.y = 0
    move_cmd.linear.z = 0
    move_cmd.angular.x = 0
    move_cmd.angular.y = 0
    move_cmd.angular.z = 0

    # Publish stop command
    for _ in range(5):  # Send stop command multiple times to ensure it's received
        pub_move.publish(move_cmd)
        rate.sleep()

def move(driver, command_name, duration=2.0):
    """
    Coordinate movement and arm swinging in separate processes.
    """
    if command_name not in movement_commands:
        print(f"Unknown command: {command_name}")
        return
        
    
    # Create an event to signal processes to terminate
    exit_event = multiprocessing.Event()
    
    # Create processes
    movement_process = multiprocessing.Process(
        target=movement_controller,
        args=(driver, command_name, duration, exit_event)
    )
    
    arm_process = multiprocessing.Process(
        target=arm_controller,
        args=(command_name, duration, exit_event)
    )
    
    try:
        # Start both processes
        print(f"Starting combined movement: {command_name} for {duration} seconds")
        movement_process.start()
        arm_process.start()
        
        # Wait for processes to complete
        movement_process.join()
        arm_process.join()
        
        print("Movement completed")
        
    except KeyboardInterrupt:
        print("Interrupted by user")
        exit_event.set()
        
    finally:
        # Clean up
        if movement_process.is_alive():
            movement_process.terminate()
        if arm_process.is_alive():
            arm_process.terminate()
