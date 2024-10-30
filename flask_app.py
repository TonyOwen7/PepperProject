# flask_app.py

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
    app.run(debug=True)
