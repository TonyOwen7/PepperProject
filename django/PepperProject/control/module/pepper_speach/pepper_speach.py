import rospy
from std_msgs.msg import String
import subprocess

def pepper_speak(text):
    """
    Publishes the given text to the /speech/ topic for Pepper to speak.
    """

    rospy.init_node('pepper_speak_node', anonymous=True)
    speech_pub = rospy.Publisher('/speech/', String, queue_size=10)

    if not rospy.is_shutdown():

        rospy.sleep(1)
        msg = String(data=text)
  
        # Publish the message
        speech_pub.publish(msg)
        
        # Log the published text
        rospy.loginfo(f"Published text to /speech/: {text}")
    else:
        rospy.logwarn("ROS node is shutdown. Cannot publish message.")


def pepper_speak2(text):
    """
    Publishes a speech message to Pepper using rostopic pub command.
    
    Args:
        text (str): The text that Pepper will say
    """
    # Escape single quotes in the text to prevent command injection
    escaped_text = text.replace("'", "\\'")
    
    # Construct the rostopic pub command
    command = f"rostopic pub -1 /speech std_msgs/String \"data: '{escaped_text}'\" "
    
    try:
        # Execute the command
        process = subprocess.Popen(
            command,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        stdout, stderr = process.communicate()
        
        if process.returncode == 0:
            print(f"Successfully published: '{text}'")
        else:
            print(f"Error publishing message: {stderr.decode('utf-8')}")
            
    except Exception as e:
        print(f"Exception occurred: {e}")
