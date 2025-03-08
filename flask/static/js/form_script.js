document.addEventListener("DOMContentLoaded", function () {
        const form = document.getElementById("pepperForm");
    
        form.addEventListener("submit", function (event) {
            event.preventDefault(); // Empêche l'envoi du formulaire classique
    
            // Récupérer les valeurs du formulaire
            const formData = new FormData(form);
            const robotIp = formData.get("robot_ip");
            const networkInterface = formData.get("network_interface");
            const language = formData.get("language");
    
            // Affichez les valeurs pour vérification (console log)
            console.log("Robot IP:", robotIp);
            console.log("Network Interface:", networkInterface);
            console.log("Language:", language);

    // Envoyer les données au serveur Flask
    fetch('/submit', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            robot_ip: robotIp,
            network_interface: networkInterface,
            language: language
        })
    })
    .then(response => {
        console.log('Réponse reçue:', response);  // Vérifier si la réponse est bien reçue
        return response.json();
    })
    .then(result => {
        console.log('Résultat du serveur:', result);  // Voir la réponse du serveur
        // Rediriger vers la page de contrôle avec l'IP et l'interface du robot
        if (result.redirect_url) {
            window.location.href = result.redirect_url;
        } else {
            console.error('Pas de redirection trouvée dans la réponse');
        }
    })
    .catch(error => {
        console.error('Erreur:', error);  // Gérer les erreurs
        });
    })
})

