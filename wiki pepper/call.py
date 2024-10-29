import rospy
from std_msgs.msg import String  # Exemple : le type de message audio peut différer

def callback(data):
    rospy.loginfo("Audio reçu : %s", data.data)

def listener():
    rospy.init_node('audio_listener', anonymous=True)
    rospy.Subscriber("/pepper_robot/audio", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
