Do that code: # flask_app.py

from flask import Flask, render_template, request, jsonify, redirect, url_for
import sqlite3
import os

app = Flask(__name__)
wiki_mode_enabled = False  # Toggle variable for Wiki mode

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

@app.route('/')
def index():
    return render_template('form.html')

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

@app.route('/control')
def control_page():
    robot_ip = request.args.get('robot_ip')
    network_interface = request.args.get('network_interface')
    return render_template('control.html', robot_ip=robot_ip, network_interface=network_interface)

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
    return jsonify({"status": "success", "wiki_mode": wiki_mode_enabled})

if __name__ == '__main__':
    app.run(debug=True). has a effect onthe robot pepper and we can do all the command : (3, 'Calibrer caméra', 'Lance l’outil de calibration de la caméra.', 'rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/cv_camera/image_raw camera:=/cv_camera'),
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
        ]. In the interface:  <!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Control your Pepper - Robot Control</title>
    <link rel="icon" href="../static/images/Pepper.png" type="image/png">
    <link rel="stylesheet" href="../static/css/control_style.css">
</head>
<body>
    <div class="bg_img"></div>
    <div class="control">
        <h1>Control Your Pepper Robot</h1>

        <!-- Toggle for Wiki Mode -->
        <label class="switch">
            <input type="checkbox" id="wiki-toggle">
            <span class="slider round"></span>
        </label>
        <p>Wiki Pepper Mode</p>

        <!-- Joystick Interface -->
        <div class="joystick-container">
            <div class="joystick-row">
                <button id="up-left" class="joystick-button">↖</button>
                <button id="up" class="joystick-button">↑</button>
                <button id="up-right" class="joystick-button">↗</button>
            </div>
            <div class="joystick-row">
                <button id="left" class="joystick-button">←</button>
                <button id="stop" class="joystick-button">⏹️</button>
                <button id="right" class="joystick-button">→</button>
            </div>
            <div class="joystick-row">
                <button id="down-left" class="joystick-button">↙</button>
                <button id="down" class="joystick-button">↓</button>
                <button id="down-right" class="joystick-button">↘</button>
            </div>
        </div>

        <div class="action-buttons">
            <button id="turn-left" class="action-button">Turn Left ↺</button>
            <button id="turn-right" class="action-button">Turn Right ↻</button>
        </div>
    </div>
    <script src="../static/js/control_script.js"></script>
    <script>
        // Fonction pour envoyer l'état du toggle Wiki Pepper
        document.getElementById("wiki-toggle").addEventListener("change", function() {
            const isEnabled = this.checked;
            fetch("/set_wiki_mode", {
                method: "POST",
                headers: {
                    "Content-Type": "application/json"
                },
                body: JSON.stringify({ wiki_mode: isEnabled })
            });
        });
    </script>
</body>
</html>. Whcih has the style: body {
    font-family: Arial, sans-serif;
    background-color: #e5e5e5;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    height: 100vh;
    margin: 0;
}

.bg_img{
    z-index: -1;
    position: absolute;
    height: 100%;
    width: 100%;
    background-image: url("../images/Pepper.png");
    background-repeat:no-repeat;
    background-size: cover;
    background-position: center;
    filter: brightness(0.8);
}

.control{
    display: flex;
    flex-direction: column;
    align-items: center;
    background-color: white;
    padding: 50px 100px;
    border-radius: 10px;
    box-shadow: 0 0 20px rgba(0, 0, 0, 0.4)
}

h1 {
    color: #333;
    font-size: 2rem;
    margin-bottom: 20px;
}

.joystick-container {
    display: flex;
    flex-direction: column;
    align-items: center;
}

.joystick-row {
    display: flex;
    justify-content: center;
    margin: 5px 0;
}

.joystick-button {
    background-color: #5d9cec;
    color: white;
    font-size: 1.5rem;
    padding: 20px;
    margin: 5px;
    border: none;
    border-radius: 10px;
    cursor: pointer;
    transition: background-color 0.3s ease;
}

.joystick-button:hover {
    background-color: #4a8ad4;
}

.action-buttons {
    display: flex;
    margin-top: 20px;
}

.action-button {
    background-color: #ff7f50;
    color: white;
    font-size: 1.2rem;
    padding: 15px 25px;
    margin: 0 10px;
    border: none;
    border-radius: 10px;
    cursor: pointer;
    transition: background-color 0.3s ease;
}

.action-button:hover {
    background-color: #ff6347;
}

.switch {
    position: relative;
    display: inline-block;
    width: 50px;
    height: 25px;
    margin-top: 15px;
}

.switch input {
    opacity: 0;
    width: 0;
    height: 0;
}

.slider {
    position: absolute;
    cursor: pointer;
    top: 0;
    left: 0;
    right: 0;
    bottom: 0;
    background-color: #ccc;
    transition: .4s;
    border-radius: 25px;
}

.slider:before {
    position: absolute;
    content: "";
    height: 18px;
    width: 18px;
    left: 4px;
    bottom: 4px;
    background-color: white;
    transition: .4s;
    border-radius: 50%;
}

input:checked + .slider {
    background-color: #4CAF50;
}

input:checked + .slider:before {
    transform: translateX(24px);
}



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