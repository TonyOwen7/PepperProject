// Sélectionner le formulaire
const form = document.getElementById('pepperForm');

// Ajouter un événement de soumission au formulaire
form.addEventListener('submit', function(e) {
    // Empêcher le comportement par défaut (rechargement de la page)
    e.preventDefault();

    // Récupérer les valeurs des inputs
    const robotIp = document.getElementById('robot_ip').value;
    const networkInterface = document.getElementById('network_interface').value;

    // Créer un objet de données à envoyer au backend
    const data = {
        robot_ip: robotIp,
        network_interface: networkInterface
    };

    // Envoyer les données au serveur Flask via POST
    fetch('/submit', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify(data) // Convertir l'objet JavaScript en JSON
    })
    .then(response => response.json()) // Attendre la réponse en JSON
    .then(result => {
        // Rediriger vers la page de contrôle après la soumission réussie
        window.location.href = result.redirect_url;
    })
    .catch(error => {
        // Gérer les erreurs ici
        console.error('Erreur:', error);
    });
});
