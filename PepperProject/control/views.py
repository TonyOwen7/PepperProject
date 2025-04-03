from django.shortcuts import get_object_or_404, redirect, render
from django.http import JsonResponse
import sys
import os
import json
from map.models import Map
from robots.models import Robot


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
from move import move
from pepper_speach import pepper_speak
from guide import guide

robot_process_manager = RobotProcessManager()
language = "fr"

def robot_configuration(request):
    return render(request, 'control/form.html')


def control_page(request):
    if request.user.is_authenticated:
        # Fetch the user's current map
        current_map = get_object_or_404(Map, user=request.user, is_current=True)
        if not current_map:
            current_map = get_object_or_404(Map, user=request.user).first()          
        
        if current_map:
            matrices = current_map.matrices  # Use the matrices field
            rooms = current_map.rooms  # Use the rooms field
        else:
            # If no current map, use the default map
            matrices = [
                [
                    [0, 2, 2, 0, 1, 0],
                    [0, 0, 0, 0, 1, 0],
                    [0, 1, 0, 0, 1, 2],
                    [0, 0, 0, 0, 2, 0],
                    [2, 1, 0, 1, 1, 2]
                ]
            ]

            rooms = {
                "Accueil": [1, 0, 1],  # [matrix_index, row_index, col_index]
                "Bureau des enseignants": [1, 4, 4],
                "Classe 1": [1, 0, 2],
                "Classe 2": [1, 2, 5],
                "Classe 3": [1, 4, 0],
                "Toilette": [1, 2, 5],
                "Bureau du directeur": [1, 4, 5]
            }
    else:
        # Use the default map for unauthenticated users
        matrices = [
            [
                [0, 2, 2, 0, 1, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 1, 0, 0, 1, 2],
                [0, 0, 0, 0, 2, 0],
                [2, 1, 0, 1, 1, 2]
            ]
        ]

        rooms = {
            "Accueil": [1, 0, 1],  # [matrix_index, row_index, col_index]
            "Bureau des enseignants": [1, 4, 4],
            "Classe 1": [1, 0, 2],
            "Classe 2": [1, 2, 5],
            "Classe 3": [1, 4, 0],
            "Toilette": [1, 2, 5],
            "Bureau du directeur": [1, 4, 5]
        }

    return render(request, 'control/control.html',
        { 
            'matrices':json.dumps(matrices),  
            'rooms':rooms,  
            'rooms_dumps':json.dumps(rooms),  

        }
    )

from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
import json

@csrf_exempt  # Temporarily disable CSRF for debugging
def submit_robot_data(request):
    print("submit")
    try:
        if request.method == 'GET':
            # Handle GET request
            data = request.GET  # Extract query parameters
            robot_ip = data.get('robot_ip')
            network_interface = data.get('network_interface')
            language = data.get('language')

            # Debugging: Print the extracted data
            print(f"GET request received with: robot_ip={robot_ip}, network_interface={network_interface}, language={language}")

            if not robot_ip or not network_interface or not language:
                return JsonResponse({'error': 'Missing required fields: robot_ip, network_interface, or language'}, status=400)

            robot_process_manager.start(robot_ip, network_interface)  # Uncomment if needed

            return redirect("/control/")

        elif request.method == 'POST':
            # Handle POST request
            data = json.loads(request.body)  # Parse JSON data from the request body
            robot_ip = data.get('robot_ip')
            network_interface = data.get('network_interface')
            language = data.get('language')

            # Debugging: Print the extracted data
            print(f"POST request received with: robot_ip={robot_ip}, network_interface={network_interface}, language={language}")
            
            if not robot_ip or not network_interface or not language:
                return JsonResponse({'error': 'Missing required fields: robot_ip, network_interface, or language'}, status=400)

            robot_process_manager.start(robot_ip, network_interface)  # Uncomment if needed

            return JsonResponse({
                'message': 'Data received successfully',
                'robot_ip': robot_ip,
                'network_interface': network_interface,
                'language': language,
                'redirect_url' : '/control/',
            })


        else:
            # Handle unsupported methods
            return JsonResponse({'error': 'Unsupported request method'}, status=405)
        
    except json.JSONDecodeError:
        return JsonResponse({'error': 'Invalid JSON data'}, status=400)
    except Exception as e:
        return JsonResponse({'error': f'An error occurred: {str(e)}'}, status=500)

def handle_move(request):
    if request.method == 'POST':
        try:
            data = json.loads(request.body)
            command_move = data.get('command_move')

            if command_move:
                move("pepper_dcm_bringup", command_move)  # Uncomment if needed
                return JsonResponse({'message': 'Le robot est en train de se déplacer'})
            else:
                return JsonResponse({'message': 'Aucune commande n\'a été donnée'}, status=400)
        except json.JSONDecodeError:
            return JsonResponse({'message': 'Invalid JSON data'}, status=400)
        except Exception as e:
            return JsonResponse({'message': str(e)}, status=500)
    else:
        return JsonResponse({'message': 'Invalid request method'}, status=405)


def handle_guiding(request):
    if request.method == 'POST':
        try:
            data = json.loads(request.body)
            location = data.get('location')

            if location:
                current_robot = get_object_or_404(Robot, user=request.user, is_current=True)
                current_map = get_object_or_404(Map, user=request.user, is_current=True)
                rooms = {}
                for key, value in current_map.rooms.items():
                    rooms[key] =  value
                guide("pepper_dcm_bringup", location, current_map.matrices, current_robot, rooms)  # Uncomment if needed
                return JsonResponse({'message': 'Le robot est en train de se déplacer vers la destination'})
            else:
                return JsonResponse({'message': 'Aucune commande n\'a été donnée'}, status=400)
        except json.JSONDecodeError:
            return JsonResponse({'message': 'Invalid JSON data'}, status=400)
        except Exception as e:
            return JsonResponse({'message': str(e)}, status=500)
    else:
        return JsonResponse({'message': 'Cette vue ne supporte que les requêtes POST.'}, status=405)

def handle_question(request):
    global language
    if request.method == 'POST':
        try:
            data = json.loads(request.body)
            question = data.get('question')

            if question and language:
                response = wiki_response(question, language)  # Replace with your logic
                response_text = response.get("text", "No response found.")
                pepper_speak(response_text)  # Uncomment if needed
                return JsonResponse({'message': response_text})
            else:
                return JsonResponse({'message': 'Aucune question n\'a été posée'}, status=400)
        except json.JSONDecodeError:
            return JsonResponse({'message': 'Invalid JSON data'}, status=400)
        except Exception as e:
            return JsonResponse({'message': str(e)}, status=500)
    else:
        return JsonResponse({'message': 'Cette vue ne supporte que les requêtes POST.'}, status=405)

def handle_speech(request):
    if request.method == 'POST':
        try:
            data = json.loads(request.body)
            speech = data.get('speech')

            if speech:
                pepper_speak(speech)  # Uncomment if needed
                return JsonResponse({'message': speech})
            else:
                return JsonResponse({'message': 'Aucune phrase n\'a été fournie'}, status=400)
        except json.JSONDecodeError:
            return JsonResponse({'message': 'Invalid JSON data'}, status=400)
        except Exception as e:
            return JsonResponse({'message': str(e)}, status=500)
    else:
        return JsonResponse({'message': 'Cette vue ne supporte que les requêtes POST.'}, status=405)

def stop_processes(request):
    if request.method == 'POST':
        robot_process_manager.stop()
        return JsonResponse({'message': 'Tous les processus ont été arrêtés avec succès'})