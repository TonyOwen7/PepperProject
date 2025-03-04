import sys
import os

module_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ''))
sys.path.append(module_path)

from flask import Flask, render_template, request, jsonify, redirect, url_for
from best_response import wiki_response
from robotProcessManager import RobotProcessManager
from move import execute_move
from pepper_speach import pepper_speak
from guide import guide


app = Flask(__name__, template_folder="../templates", static_folder="../static")
wiki_mode_enabled = False

robot_process_manager = RobotProcessManager()

driver = "naoqi_driver"

@app.route('/')
def index():
    return render_template('form.html')

@app.route('/control')
def control_page():
    return render_template('control.html')

@app.route('/submit', methods=['POST'])
def submit_robot_data():
    global driver
    data = request.json
    robot_ip = data['robot_ip']
    network_interface = data['network_interface']
    chosen_driver = data['driver']
    
    robot_process_manager.start_roscore()
    if chosen_driver == "naoqi_driver":
        robot_process_manager.start_naoqi_driver(robot_ip, network_interface)
    else: 
        if chosen_driver == "pepper_dcm":
            driver = chosen_driver
            robot_process_manager.start_pepper_dcm_bringup(robot_ip, network_interface)

    return jsonify({
        'redirect_url': url_for('control_page', 
                                robot_ip=robot_ip, 
                                network_interface=network_interface,
                                driver=driver
                                )
    })

@app.route('/move', methods=['POST'])
def handle_move():
    global driver
    data = request.get_json()
    command_move = data['command_move']

    if command:
        move(driver, command_move)


@app.route('/move', methods=['POST'])
def handle_guiding():
    global driver
    data = request.get_json()
    location = data['location']

    if command:
        guide(driver, location)


@app.route('/question', methods=['POST'])
def handle_question():
    data = request.get_json()
    question = data['question']
    
    if question:
        response = wiki_response(question)
        response = response["text"]
        pepper_speak(response)
        

@app.route('/stop_processes', methods=['POST'])
def stop_processes():
    driver = data.get('driver')
    if driver == "naoqi_driver":
        robot_process_manager.stop_naoqi_driver()
    else:
        if driver == "pepper_dcm":
            robot_process_manager.stop_pepper_dcm_bringup()

    robot_process_manager.stop_roscore()
    return jsonify({'message': 'Tous les processus ont été arrêtés avec succès'})

if __name__ == '__main__':
    app.run(debug=True)
