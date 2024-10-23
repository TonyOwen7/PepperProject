document.getElementById('pepperForm').addEventListener('submit', function(e) {
    e.preventDefault();  // Empêcher la soumission classique du formulaire

    console.log('Formulaire soumis');  // Vérifier si la soumission fonctionne

    const robotIp = document.getElementById('robot_ip').value;
    const networkInterface = document.getElementById('network_interface').value;

    console.log('Données du robot:', { robotIp, networkInterface });  // Vérifier les données récupérées

    // Envoyer les données au serveur Flask
    fetch('/submit', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            robot_ip: robotIp,
            network_interface: networkInterface
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
});
