const driver = "{{ driver }}";
const language = "{{ language }}";

function toggleInputZone() {
    const inputZone = document.querySelector('.input-zone');
    inputZone.classList.toggle('hidden');
    inputZone.classList.toggle('show');
}

const buttons = document.querySelectorAll('.joystick-button');
buttons.forEach(button => {
    button.addEventListener('click', function() {
        sendMove(this.getAttribute('data-command'));
    });
});

const questionButton = document.querySelector('.send-question');
questionButton.addEventListener('click', sendQuestion);

document.getElementById('question-input').addEventListener('keypress', function(event) {
    if (event.key === 'Enter') {
        sendQuestion();
    }
});

const destinationButtons = document.querySelectorAll('.destination-button');
destinationButtons.forEach(button => {
    button.addEventListener('click', function() {
        sendDestination(this.textContent);
    });
});

function sendMove(command_move) {
    fetch('/move', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            driver: driver,
            command_move: command_move,
        })
    })
    .then(response => response.json())
    .then(result => console.log(result.message))
    .catch(error => console.error('Erreur:', error));
}


function switchTab(tabName) {
    document.querySelectorAll('.tab-button').forEach(button => {
        button.classList.remove('active');
    });

    document.querySelectorAll('.tab-content').forEach(content => {
        content.classList.add('hidden');
    });

    document.getElementById(`${tabName}-tab`).classList.remove('hidden');
    document.querySelector(`[onclick="switchTab('${tabName}')"]`).classList.add('active');
}


function sendQuestion() {
    const input = document.getElementById('question-input');
    const text = input.value.trim();
    if (text) {
        fetch('/move', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
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
        fetch('/move', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
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
    fetch('/move', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            driver: driver,
            location: destination,
        })
    })
    .then(response => response.json())
    .then(result => console.log(result.message))
    .catch(error => console.error('Erreur:', error));
}
