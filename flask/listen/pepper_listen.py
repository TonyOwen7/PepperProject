import sys
import os

move_path = os.path.join(os.path.dirname(__file__), '../move')
guide_path = os.path.join(os.path.dirname(__file__), '../guide')
mymap_path = os.path.join(os.path.dirname(__file__), '../mymap')
pepper_speak_path = os.path.join(os.path.dirname(__file__), '../pepper_speak')
wiki_response_path = os.path.join(os.path.dirname(__file__), '../wiki_response')

sys.path.append(move_path)
sys.path.append(guide_path)
sys.path.append(mymap_path)
sys.path.append(pepper_speak_path)
sys.path.append(wiki_response_path)

import rospy
from std_msgs.msg import String
from move import move
from guide import guide
from mymap import location_queries, university_matrix
from pepper_speak import pepper_speak
from wiki_response import ia

imperative_mapping = {
    "avance": {"command": "avancer", "response": "J'avance"},
    "recule": {"command": "reculer", "response": "Je recule"},
    "va à gauche": {"command": "aller à gauche", "response": "Je vais à gauche"},
    "va à droite": {"command": "aller à droite", "response": "Je vais à droite"},
    "avance vers la gauche": {"command": "avancer vers gauche", "response": "J'avance vers la gauche"},
    "avance vers la droite": {"command": "avancer vers droite", "response": "J'avance vers la droite"},
    "recule vers la gauche": {"command": "reculer vers gauche", "response": "Je recule vers la gauche"},
    "recule vers la droite": {"command": "reculer vers droite", "response": "Je recule vers la droite"},
    "tourne à gauche": {"command": "tourner à gauche", "response": "Je tourne à gauche"},
    "tourne à droite": {"command": "tourner à droite", "response": "Je tourne à droite"},
    "fais un demi-tour à gauche": {"command": "demi-tour gauche", "response": "Je fais un demi-tour à gauche"},
    "fais un demi-tour à droite": {"command": "demi-tour droit", "response": "Je fais un demi-tour à droite"},
    "arrête": {"command": "stop", "response": "Je m'arrête"},
}

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

def process_input(text_heard, language="fr"):
    if text_heard is None:
        return

    for imperative, details in imperative_mapping.items():
        if imperative in text_heard.lower():
            pepper_speak(details["response"])  
            move('naoqi_driver', details["command"]) 
            return

    for location in location_queries:
        if location.lower() in text_heard.lower():
            pepper_speak(f"Allons à la {location}") 
            guide('naoqi_driver', [location])  
            return

    response = wiki_response(text_heard, language)
    pepper_speak(response)
