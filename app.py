from flask import Flask, request, jsonify, session, redirect, url_for
import sqlite3

app = Flask(__name__)
app.secret_key = 'supersecretkey'  # Nécessaire pour utiliser les sessions

# Configuration des sessions avec Flask
app.config['SESSION_TYPE'] = 'filesystem'

# Fonction pour initialiser la base de données (si nécessaire)
def init_db():
    with sqlite3.connect('commands.db') as conn:
        cursor = conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS robot_data (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                robot_ip TEXT NOT NULL,
                network_interface TEXT NOT NULL
            )
        ''')
        conn.commit()

# Initialiser la base de données au démarrage
init_db()

# Route pour gérer la soumission du formulaire
@app.route('/submit', methods=['POST'])
def submit_robot_data():
    data = request.get_json()  # Récupérer les données JSON envoyées

    robot_ip = data['robot_ip']
    network_interface = data['network_interface']

    # Sauvegarder les données dans la session Flask pour les réutiliser
    session['robot_ip'] = robot_ip
    session['network_interface'] = network_interface

    # Sauvegarder les données dans la base de données (optionnel, pour traçabilité)
    with sqlite3.connect('commands.db') as conn:
        cursor = conn.cursor()
        cursor.execute("INSERT INTO robot_data (robot_ip, network_interface) VALUES (?, ?)", (robot_ip, network_interface))
        conn.commit()

    # Retourner une réponse JSON au client
    return jsonify({'message': 'Données du robot sauvegardées avec succès !', 'redirect_url': url_for('control_page')})

# Route vers la page de contrôle du robot (redirection)
@app.route('/control')
def control_page():
    # On vérifie que les données de session sont présentes
    if 'robot_ip' in session and 'network_interface' in session:
        return jsonify({
            'robot_ip': session['robot_ip'],
            'network_interface': session['network_interface']
        })
    else:
        return jsonify({'error': 'Les informations du robot ne sont pas disponibles.'}), 400

# Lancer le serveur Flask
if __name__ == '__main__':
    app.run(debug=True)
