import rospy
from std_msgs.msg import String


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