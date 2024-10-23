const robotIp = "{{ robot_ip }}";
const networkInterface = "{{ network_interface }}";

// Gérer les commandes du joystick
const buttons = document.querySelectorAll('.joystick-button');
buttons.forEach(button => {
    button.addEventListener('click', function() {
        const command = this.id;

        // Envoyer la commande avec l'IP et l'interface réseau du robot
        fetch('/command', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                command: command,  // Le nom de la commande (par exemple 'Avancer')
                robot_ip: robotIp,
                network_interface: networkInterface
            })
        })
        .then(response => response.json())
        .then(result => {
            console.log(result.message);
        })
        .catch(error => {
            console.error('Erreur:', error);
        });
    });
});
