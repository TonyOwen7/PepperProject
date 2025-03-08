import rospy
from std_msgs.msg import String

def pepper_speak(text):
    rospy.init_node('pepper_speak_node', anonymous=True, disable_signals=True)
    pub = rospy.Publisher('pepper_speak', String, queue_size=10)
    rospy.sleep(1)
    pub.publish(String(data=text))
    rospy.loginfo(f"Published text: {text}")
    rospy.sleep(1)
