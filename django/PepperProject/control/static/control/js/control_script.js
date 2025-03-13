const language = "{{ language }}";
{{ rooms }};
-
function toggleInputZone() {
    const inputZone = document.querySelector('.input-zone');
    inputZone.classList.toggle('hidden');
    inputZone.classList.toggle('show');
}

const questionButton = document.querySelector('.send-question');
questionButton.addEventListener('click', sendQuestion);

document.getElementById('question-input').addEventListener('keypress', function(event) {
    if (event.key === 'Enter') {
        sendQuestion();
    }
});


const speechButton = document.querySelector('.send-speech')
speechButton.addEventListener('click', sendSpeech);
document.getElementById('speech-input').addEventListener('keypress', function(event) {
    if (event.key === 'Enter') {
        sendSpeech();
    }
});


document.querySelector('.destination-button').addEventListener('click', function() {
    const destination = document.getElementById('destination-input').value.trim();
    if (destination) {
        sendDestination(destination);
    } else {
        alert("Veuillez choisir une destination !");
    }
});
function getCookie(name) {
    let cookieValue = null;
    if (document.cookie && document.cookie !== '') {
        const cookies = document.cookie.split(';');
        for (let i = 0; i < cookies.length; i++) {
            const cookie = cookies[i].trim();
            if (cookie.substring(0, name.length + 1) === (name + '=')) {
                cookieValue = decodeURIComponent(cookie.substring(name.length + 1));
                break;
            }
        }
    }
    return cookieValue;
}

const csrftoken = getCookie('csrftoken');

function sendMove(move) {
    fetch('/move/', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'X-CSRFToken': csrftoken,  // Include the CSRF token in the headers
        },
        body: JSON.stringify({
            command_move: move,
        })
    })
    .then(response => response.json())
    .then(result => console.log(result.message))
    .catch(error => console.error('Error:', error));
}

function switchTab(tabName) {
    document.querySelectorAll('.tab-button').forEach(button => {
        button.classList.remove('active');
    });

    document.querySelectorAll('.tab-content').forEach(content => {
        content.classList.add('hidden');
    });

    document.getElementById(`${tabName}-tab`).classList.remove('hidden');
    document.querySelector([onclick="switchTab('${tabName}')"]).classList.add('active');
}

function sendQuestion() {
    const input = document.getElementById('question-input');
    const text = input.value.trim();
    if (text) {
        fetch('/question/', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'X-CSRFToken': csrftoken,  // Include the CSRF token
            },
            body: JSON.stringify({
                question: text,
                language: language,
            })
        })
        .then(response => response.json())
        .then(data => {
            alert(data.message || "Commande envoyée !");
            input.value = '';
        })
        .catch(error => console.error('Erreur:', error));
    } else {
        alert("Veuillez écrire quelque chose pour que Pepper parle !");
    }
}

function sendSpeech() {
    const input = document.getElementById('speech-input');
    const text = input.value.trim();
    if (text) {
        fetch('/speech/', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'X-CSRFToken': csrftoken,  // Include the CSRF token
            },
            body: JSON.stringify({
                speech: text,
            })
        })
        .then(response => response.json())
        .then(data => {
            alert(data.message || "Commande envoyée !");
            input.value = '';
        })
        .catch(error => console.error('Erreur:', error));
    } else {
        alert("Veuillez écrire quelque chose pour que Pepper parle !");
    }
}

function sendDestination(destination) {
    fetch('/destination/', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'X-CSRFToken': csrftoken,  // Include the CSRF token
        },
        body: JSON.stringify({
            location: destination,
            rooms: rooms
        })
    })
    .then(response => response.json())
    .then(result => console.log(result.message))
    .catch(error => console.error('Erreur:', error));
}