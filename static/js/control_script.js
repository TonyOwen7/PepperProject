// Récupérer les informations du robot depuis le backend
fetch('/control')
    .then(response => response.json())
    .then(data => {
        if (data.error) {
            alert(data.error);
        } else {
            // Afficher les informations du robot
            const robotInfo = document.getElementById('robotInfo');
            robotInfo.innerHTML = `<p>Robot IP: ${data.robot_ip}</p><p>Network Interface: ${data.network_interface}</p>`;
        }
    })
    .catch(error => {
        console.error('Erreur:', error);
    });

// Ajouter des événements aux boutons du joystick pour envoyer les commandes
const buttons = document.querySelectorAll('.joystick-button');
buttons.forEach(button => {
    button.addEventListener('click', function() {
        const command = this.id;

        // Envoyer la commande au backend
        fetch('/command', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ command: command })
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
