from django.shortcuts import render
from django.http import JsonResponse
import sys
import os
from map.models import Map

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
    if request.user.is_authenticated:
        # Fetch the user's current map
        current_map = Map.objects.filter(user=request.user, is_current=True).first()
        if current_map:
            matrix = current_map.matrix
            rooms = current_map.rooms
        else:
            # If no current map, use the default map
            matrix = [
                [0, 2, 2, 0, 1, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 1, 0, 0, 1, 2],
                [0, 0, 0, 0, 2, 0],
                [2, 1, 0, 1, 1, 2]
            ]
            rooms = {
                "Accueil": (0, 1),
                "Bureau des enseignants": (4, 4),
                "Classe 1": (0, 2),
                "Classe 2": (2, 5),
                "Classe 3": (4, 0),
                "Toilette": (2, 5),
                "Escalier": (4, 0),
                "Bureau du directeur": (4, 5)
            }
    else:
        # Use the default map for unauthenticated users
        matrix = [
            [0, 2, 2, 0, 1, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 1, 0, 0, 1, 2],
            [0, 0, 0, 0, 2, 0],
            [2, 1, 0, 1, 1, 2]
        ]
        rooms = {
            "Accueil": (0, 1),
            "Bureau des enseignants": (4, 4),
            "Classe 1": (0, 2),
            "Classe 2": (2, 5),
            "Classe 3": (4, 0),
            "Toilette": (2, 5),
            "Escalier": (4, 0),
            "Bureau du directeur": (4, 5)
        }

    return render(request, 'control/control.html', {
        'matrix': matrix,
        'rooms': rooms,
    })

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
    else:
        return JsonResponse({'response': 'Cette vue ne supporte que les requêtes POST.'}, status=405)
            

def stop_processes(request):
    if request.method == 'POST':
        robot_process_manager.stop()
        return JsonResponse({'message': 'Tous les processus ont été arrêtés avec succès'})