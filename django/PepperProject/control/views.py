from django.shortcuts import render
from django.http import JsonResponse

import sys
import os

# Get the base directory of the project
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Add the paths to the modules to sys.path
module_paths = [
    os.path.join(BASE_DIR, 'control', 'module', 'ia'),
    os.path.join(BASE_DIR, 'control','module', 'robotProcessManager'),
    os.path.join(BASE_DIR, 'control', 'module', 'move'),
    os.path.join(BASE_DIR, 'control', 'module', 'pepper_speach'),
    os.path.join(BASE_DIR, 'control', 'module', 'guide'),
]

for path in module_paths:
    if path not in sys.path:
        sys.path.append(path)

from best_response import wiki_response
from robotProcessManager import RobotProcessManager
# from move import move
# from pepper_speach import pepper_speak
# from guide import guide

robot_process_manager = RobotProcessManager()
language = "fr"

def robot_configuration(request):
    return render(request, 'control/form.html')

def control_page(request):
    return render(request, 'control/control.html')

def submit_robot_data(request):
    if request.method == 'POST':
        data = request.POST
        robot_ip = data.get('robot_ip')
        network_interface = data.get('network_interface')
        language = data.get('language')
        
        robot_process_manager.start(robot_ip, network_interface)

        return JsonResponse({
            'redirect_url': '/control/',
            'language': language,
        })

def handle_move(request):
    if request.method == 'POST':
        data = request.POST
        command_move = data.get('command_move')

        if command_move:
            move("pepper_dcm_bringup", command_move)
            return JsonResponse({'response': 'Le robot est en train de se déplacer'})
        else:
            return JsonResponse({'response': 'Aucune commande n\'a été donnée'})

def handle_guiding(request):
    if request.method == 'POST':
        data = request.POST
        location = data.get('destination')

        if location:
            guide("pepper_dcm_bringup", location)
            return JsonResponse({'response': 'Le robot est en train de se déplacer vers la destination'})
        else:
            return JsonResponse({'response': 'Aucune commande n\'a été donnée'})

def handle_question(request):
    if request.method == 'POST':
        data = request.POST
        question = data.get('question')
        language = data.get('language')
        
        if question:
            response = wiki_response(question, language)
            response = response["text"]
            pepper_speak(response)
            return JsonResponse({'response': response})
        else:
            return JsonResponse({'response': 'Aucune question n\'a été posée'})

def handle_speech(request):
    if request.method == 'POST':
        data = request.POST
        speech = data.get('speech')
        
        if speech:
            pepper_speak(speech)
            return JsonResponse({'response': speech})    
        else:
            return JsonResponse({'response': 'Aucune question n\'a été posée'})

def stop_processes(request):
    if request.method == 'POST':
        robot_process_manager.stop()
        return JsonResponse({'message': 'Tous les processus ont été arrêtés avec succès'})