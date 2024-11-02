from flask import Flask, render_template, request, jsonify, redirect, url_for
import sqlite3
import os
import requests
from bs4 import BeautifulSoup
import re

app = Flask(__name__)
wiki_mode_enabled = False  # Toggle for Wiki Pepper mode

# Initialize the database if it does not exist
def init_db():
    conn = sqlite3.connect('PepperProject.db')
    cursor = conn.cursor()
    cursor.execute('''
    CREATE TABLE IF NOT EXISTS ros_commands (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        command_name TEXT NOT NULL,
        description TEXT,
        parameters TEXT NOT NULL
    )
    ''')
    
    cursor.execute('SELECT COUNT(*) FROM ros_commands')
    if cursor.fetchone()[0] == 0:
        commands = [
            (1, 'Lancer DCM', 'Lance le DCM pour connecter Pepper via ROS avec IP.', 'roslaunch pepper_dcm_bringup pepper_bringup.launch robot_ip:=192.168.1.34 roscore_ip:=localhost network_interface:=enp0s3'),
            (2, 'Driver Naoqi', 'Démarre le driver Naoqi pour interagir avec Pepper.', 'roslaunch naoqi_driver naoqi_driver.launch nao_ip:=192.168.0.126 roscore_ip:=localhost network_interface:=enp0s8'),
            (3, 'Calibrer caméra', 'Lance l’outil de calibration de la caméra.', 'rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/cv_camera/image_raw camera:=/cv_camera'),
            (4, 'Contrôle joystick', 'Permet de contrôler Pepper via une manette (joystick).', 'roslaunch teleop_twist_joy pepper.launch'),
            (5, 'Avancer', 'Fait avancer Pepper en ligne droite.', 'rostopic pub /cmd_vel geometry_msgs/Twist "[0.5, 0, 0]" "[0, 0, 0]"'),
            (6, 'Reculer', 'Fait reculer Pepper en ligne droite.', 'rostopic pub /cmd_vel geometry_msgs/Twist "[-0.5, 0, 0]" "[0, 0, 0]"'),
            (7, 'Tourner à gauche', 'Fait tourner Pepper vers la gauche sur un angle donné.', 'rostopic pub /cmd_vel geometry_msgs/Twist "[0, 0, 0]" "[0, 0, 0.5]"'),
            (8, 'Tourner à droite', 'Fait tourner Pepper vers la droite sur un angle donné.', 'rostopic pub /cmd_vel geometry_msgs/Twist "[0, 0, 0]" "[0, 0, -0.5]"'),
            (9, 'Avancer vers la gauche', 'Fait avancer Pepper tout en se déplaçant vers la gauche.', 'rostopic pub /cmd_vel geometry_msgs/Twist "[0.5, 0.5, 0]" "[0, 0, 0]"'),
            (10, 'Avancer vers la droite', 'Fait avancer Pepper tout en se déplaçant vers la droite.', 'rostopic pub /cmd_vel geometry_msgs/Twist "[0.5, -0.5, 0]" "[0, 0, 0]"'),
            (11, 'Tourner sur soi-même vers la gauche', 'Fait tourner Pepper sur lui-même dans le sens anti-horaire (vers la gauche).', 'rostopic pub /cmd_vel geometry_msgs/Twist "[0, 0, 0]" "[0, 0, 0.5]"'),
            (12, 'Tourner sur soi-même vers la droite', 'Fait tourner Pepper sur lui-même dans le sens horaire (vers la droite).', 'rostopic pub /cmd_vel geometry_msgs/Twist "[0, 0, 0]" "[0, 0, -0.5]"'),
            (13, 'Tour complet vers la gauche', 'Fait tourner Pepper sur lui-même à 360 degrés dans le sens anti-horaire.', 'rostopic pub /cmd_vel geometry_msgs/Twist "[0, 0, 0]" "[0, 0, 3.14]"'),
            (14, 'Tour complet vers la droite', 'Fait tourner Pepper sur lui-même à 360 degrés dans le sens horaire.', 'rostopic pub /cmd_vel geometry_msgs/Twist "[0, 0, 0]" "[0, 0, -3.14]"'),
            (15, 'Indiquer la position de la paume de la main', 'Retourne la position de la paume de la main en utilisant les joints associés.', 'rostopic echo /joint_states | grep LHand'),
            (16, 'Parler', 'Fait dire un message prédéfini à Pepper.', 'rostopic pub /speech std_msgs/String "data: \'Hello ISTY students\'"'),
            (17, 'Saluer', 'Fait lever la main de Pepper pour dire bonjour.', 'rostopic pub /joint_angles naoqi_bridge_msgs/JointAnglesWithSpeed "joint_names: [\'LShoulderRoll\', \'LShoulderPitch\'], joint_angles: [0, 1.57], speed: 0.1"')
        ]
        cursor.executemany('INSERT INTO ros_commands (id, command_name, description, parameters) VALUES (?, ?, ?, ?)', commands)
        conn.commit()
    conn.close()

init_db()

# Wiki Pepper Mode utility functions
def extract_keywords(text):
    return set(re.findall(r'\w+', text.lower()))

def find_most_relevant_paragraphs(text, question_keywords, max_paragraphs=3):
    paragraphs = [p.strip() for p in text.split('\n') if p.strip()]
    scored_paragraphs = [
        (len(question_keywords & extract_keywords(paragraph)), paragraph)
        for paragraph in paragraphs
        if len(question_keywords & extract_keywords(paragraph)) > 0
    ]
    scored_paragraphs.sort(reverse=True, key=lambda x: x[0])
    return " ".join([p[1] for p in scored_paragraphs[:max_paragraphs]])

def rechercher_wikipedia(question, langue="fr"):
    url = f"https://{langue}.wikipedia.org/w/api.php"
    params = {"action": "query", "list": "search", "srsearch": question, "format": "json"}
    try:
        response = requests.get(url, params=params)
        data = response.json()
        keywords = extract_keywords(question)
        for result in data["query"]["search"]:
            page_id = result["pageid"]
            content_url = f"https://{langue}.wikipedia.org/w/api.php?action=query&prop=extracts&pageids={page_id}&exintro=&format=json"
            content_data = requests.get(content_url).json()
            page_content = BeautifulSoup(content_data["query"]["pages"][str(page_id)]["extract"], "html.parser").get_text()
            return {"text": find_most_relevant_paragraphs(page_content.lower(), keywords), "source": "Wikipedia"}
        return {"text": "Aucun résultat trouvé sur Wikipedia.", "source": "Wikipedia"}
    except Exception as e:
        return {"text": f"Erreur lors de la récupération d'informations : {e}", "source": "Wikipedia"}

def rechercher_information(question):
    response_wikipedia = rechercher_wikipedia(question)
    return response_wikipedia if response_wikipedia["text"] else {"text": "Information introuvable.", "source": "Aucune"}

def pepper_speak(text):
    # Command to make Pepper speak the response
    os.system(f'rostopic pub /speech std_msgs/String "data: \'{text}\'"')

def start_listening():
    # Begin listening for spoken commands
    os.system("rostopic pub /start_listening std_msgs/Bool true")

def stop_listening():
    # Stop listening when Wiki Mode is disabled
    os.system("rostopic pub /start_listening std_msgs/Bool false")

@app.route('/')
def index():
    return render_template('form.html')

@app.route('/control')
def control_page():
    robot_ip = request.args.get('robot_ip')
    network_interface = request.args.get('network_interface')
    return render_template('control.html', robot_ip=robot_ip, network_interface=network_interface)

@app.route('/submit', methods=['POST'])
def submit_robot_data():
    data = request.json
    robot_ip = data['robot_ip']
    network_interface = data['network_interface']
    conn = sqlite3.connect('PepperProject.db')
    cursor = conn.cursor()
    cursor.execute("SELECT parameters FROM ros_commands WHERE command_name = 'Lancer DCM'")
    lancer_dcm_command = cursor.fetchone()[0]
    os.system(lancer_dcm_command.replace('192.168.1.34', robot_ip).replace('enp0s3', network_interface))
    conn.close()
    return jsonify({'redirect_url': url_for('control_page', robot_ip=robot_ip, network_interface=network_interface)})

@app.route('/command', methods=['POST'])
def handle_command():
    data = request.get_json()
    command = data['command']
    robot_ip = data['robot_ip']
    network_interface = data['network_interface']
    conn = sqlite3.connect('PepperProject.db')
    cursor = conn.cursor()
    cursor.execute("SELECT parameters FROM ros_commands WHERE command_name = ?", (command,))
    ros_command = cursor.fetchone()
    if ros_command:
        os.system(ros_command[0].replace('192.168.1.34', robot_ip).replace('enp0s3', network_interface))
        return jsonify({'message': f'Commande {command} exécutée avec succès !'})
    else:
        return jsonify({'error': 'Commande non reconnue.'}), 400

@app.route('/set_wiki_mode', methods=['POST'])
def set_wiki_mode():
    global wiki_mode_enabled
    wiki_mode_enabled = request.json.get("wiki_mode", False)
    if wiki_mode_enabled:
        start_listening()
    else:
        stop_listening()
    return jsonify({"status": "success", "wiki_mode": wiki_mode_enabled})

@app.route('/ask', methods=['POST'])
def ask():
    if not wiki_mode_enabled:
        return jsonify({"error": "Wiki Mode désactivé."}), 400
    question = request.json.get("question")
    if not question:
        return jsonify({"error": "Aucune question fournie"}), 400
    answer = rechercher_information(question)
    pepper_speak(answer["text"])  # Make Pepper speak the response
    return jsonify({"question": question, "response": answer["text"], "source": answer["source"]})

if __name__ == '__main__':
    app.run(debug=True)
