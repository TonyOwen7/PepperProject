import rospy
from std_msgs.msg import String

def pepper_listen():
    text_heard = None

    def callback(data):
        nonlocal text_heard  
        rospy.loginfo(f"Pepper heard: {data.data}")
        text_heard = data.data 
    rospy.init_node('pepper_listen_node', anonymous=True)
    rospy.Subscriber('pepper_heard', String, callback)  
    rospy.loginfo("Listening for what Pepper has heard...")

    rospy.sleep(0.5)  
    rospy.spin_once() 

    return text_heard  
