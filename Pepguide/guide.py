import rospy
from geometry_msgs.msg import PoseStamped

def move_to_classroom(classroom_coordinates):
    rospy.init_node('classroom_navigator', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    # Créer un message PoseStamped avec les coordonnées de la salle
    goal = PoseStamped()
    goal.header.frame_id = "map"  # Assurez-vous que cela correspond au cadre de votre carte
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = classroom_coordinates[0]
    goal.pose.position.y = classroom_coordinates[1]
    goal.pose.position.z = 0  # En général, z est 0 pour le sol

    # Orientation (quaternion) pour faire face à la direction
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0

    rospy.loginfo("En route vers la salle de classe...")
    pub.publish(goal)

def main():
    while not rospy.is_shutdown():
        question = input("Quelle salle voulez-vous visiter? ")
        coordinates = get_coordinates(question)  # Implémentez cette fonction pour obtenir les coordonnées
        if coordinates:
            move_to_classroom(coordinates)
        else:
            print("Salle non trouvée.")

if __name__ == '__main__':
    main()
