import sqlite3
from flask import Flask, render_template, request, jsonify, redirect, url_for
import os

app = Flask(__name__)

# Fonction pour initialiser la base de données si elle n'existe pas
def init_db():
    conn = sqlite3.connect('PepperProject.db')  # Nom de la base de données
    cursor = conn.cursor()

    # Créer la table ros_commands si elle n'existe pas
    cursor.execute('''
    CREATE TABLE IF NOT EXISTS ros_commands (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        command_name TEXT NOT NULL,
        description TEXT,
        parameters TEXT NOT NULL
    )
    ''')

    # Insertion des commandes si elles n'existent pas déjà
    cursor.execute('SELECT COUNT(*) FROM ros_commands')
    if cursor.fetchone()[0] == 0:  # S'il n'y a pas encore de données, on insère les commandes
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

# Initialiser la base de données
init_db()

# Route pour afficher la page de formulaire
@app.route('/')
def index():
    return render_template('form.html')

# Route pour gérer la soumission du formulaire et rediriger vers la page de contrôle
@app.route('/submit', methods=['POST'])
def submit_robot_data():
    data = request.json
    robot_ip = data['robot_ip']
    network_interface = data['network_interface']

    # Exécuter la commande "Lancer DCM" après la redirection
    conn = sqlite3.connect('PepperProject.db')
    cursor = conn.cursor()
    cursor.execute("SELECT parameters FROM ros_commands WHERE command_name = 'Lancer DCM'")
    lancer_dcm_command = cursor.fetchone()[0]
    os.system(lancer_dcm_command.replace('192.168.1.34', robot_ip).replace('enp0s3', network_interface))  # Remplace les IP et interface réseau

    conn.close()

    # Rediriger vers la page de contrôle
    return jsonify({'redirect_url': url_for('control_page', robot_ip=robot_ip, network_interface=network_interface)})

# Route pour afficher la page de contrôle du robot
@app.route('/control')
def control_page():
    robot_ip = request.args.get('robot_ip')
    network_interface = request.args.get('network_interface')
    return render_template('control.html', robot_ip=robot_ip, network_interface=network_interface)

# Route pour gérer les commandes du joystick
@app.route('/command', methods=['POST'])
def handle_command():
    data = request.get_json()
    command = data['command']
    robot_ip = data['robot_ip']
    network_interface = data['network_interface']

    # Chercher la commande ROS dans la base de données en fonction de la commande envoyée par le joystick
    conn = sqlite3.connect('PepperProject.db')
    cursor = conn.cursor()
    cursor.execute("SELECT parameters FROM ros_commands WHERE command_name = ?", (command,))
    ros_command = cursor.fetchone()

    if ros_command:
        # Exécuter la commande ROS avec les IP et interfaces correctes
        os.system(ros_command[0].replace('192.168.1.34', robot_ip).replace('enp0s3', network_interface))
        return jsonify({'message': f'Commande {command} exécutée avec succès !'})
    else:
        return jsonify({'error': 'Commande non reconnue.'}), 400

if __name__ == '__main__':
    app.run(debug=True)
