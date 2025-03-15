const mapElement = document.getElementById('map');

const matrices = JSON.parse(mapElement.getAttribute('data-matrices'));
const number_of_floors = JSON.parse(mapElement.getAttribute('data-number-of-floors'));


const mapId = parseInt(mapElement.getAttribute('data-id'));
let rooms;
try {
    rooms = JSON.parse(mapElement.getAttribute('data-rooms')) || {};
    if (typeof rooms !== "object" || Array.isArray(rooms)) {
        throw new Error("Invalid rooms format");
    }
} catch (error) {
    console.error("Failed to parse rooms data:", error);
    rooms = {}; // Default to an empty dictionary
}
console.log('Matrices:', matrices);  // Debugging: Log the matrices
console.log('Map ID:', mapId);       // Debugging: Log the map ID
console.log('Rooms:', rooms);        // Debugging: Log the rooms
document.getElementById('number_of_floors').innerHTML = `Number of floors: ${matrices.length}`;

let currentMatrixIndex = 0; // Track the currently displayed matrix
let rows = matrices[0].length;
let cols = matrices[0][0].length;

let selectedValue = 2; // Default to room

// Render the current matrix
function renderMap() {
    const matrix = matrices[currentMatrixIndex];
    mapElement.innerHTML = '';
    mapElement.style.gridTemplateColumns = `repeat(${cols}, 50px)`;

    matrix.forEach((row, rowIndex) => {
        row.forEach((cell, colIndex) => {
            const cellElement = document.createElement('div');
            cellElement.classList.add('cell');
            if (cell === 0) cellElement.classList.add('empty');
            else if (cell === 1) cellElement.classList.add('obstacle');
            else if (cell === 2) cellElement.classList.add('room');
            else if (cell === 3) cellElement.classList.add('stairs'); // Add stairs/doors class
            cellElement.addEventListener('click', () => handleCellClick(rowIndex, colIndex));
            mapElement.appendChild(cellElement);
        });
    });
}


function updateCarousel() {
    document.getElementById('matrix-index').textContent = `Floor ${currentMatrixIndex + 1}`;
}

document.getElementById('prev-matrix').addEventListener('click', () => {
    if (currentMatrixIndex > 0) {
        currentMatrixIndex--;
        renderMap();
        updateCarousel();
    }
});

document.getElementById('next-matrix').addEventListener('click', () => {
    if (currentMatrixIndex < matrices.length - 1) {
        currentMatrixIndex++;
        renderMap();
        updateCarousel();
    }
});

function addMatrix() {
    const newMatrix = Array.from({ length: rows }, () => Array(cols).fill(0));
    matrices.push(newMatrix);
    currentMatrixIndex = matrices.length - 1;
    document.getElementById('number_of_floors').innerHTML = `Number of floors: ${matrices.length}`;
    renderMap();
    updateCarousel();
}

function removeMatrix() {
    if (matrices.length > 1) {
        matrices.splice(currentMatrixIndex, 1);
        currentMatrixIndex = Math.min(currentMatrixIndex, matrices.length - 1);
        renderMap();
        updateCarousel();
    }
}
function handleCellClick(rowIndex, colIndex) {
    if (selectedValue === 2) {
      // Adding a new room
      const roomName = prompt('Enter room name:');
      console.log(roomName);
      if (roomName) {
        // Remove this console.log(type) - 'type' isn't defined here
        matrices[currentMatrixIndex][rowIndex][colIndex] = 2; // Mark as room
        rooms[roomName] = [currentMatrixIndex + 1, rowIndex, colIndex]; // Store room position
      }
    } else if (selectedValue === 3) {
      // Adding stairs/doors
      const type = prompt('Enter type (stairs/elevator):');
      if (type && (type.toLowerCase() === 'stairs' || type.toLowerCase() === 'elevator')) {
        matrices[currentMatrixIndex][rowIndex][colIndex] = 3; // Mark as stairs/doors
        rooms[type] = [currentMatrixIndex + 1, rowIndex, colIndex]; // Store stairs/doors position
      }
    } else {
      // Removing or updating a cell
      if (matrices[currentMatrixIndex][rowIndex][colIndex] === 2 || matrices[currentMatrixIndex][rowIndex][colIndex] === 3) {
        // If the cell was a room or stairs/doors, remove it from the rooms dictionary
        const roomName = Object.keys(rooms).find(
          key => rooms[key][0] === currentMatrixIndex + 1 && rooms[key][1] === rowIndex && rooms[key][2] === colIndex
        );
        if (roomName) {
          delete rooms[roomName]; // Remove the room or stairs/doors from the dictionary
        }
      }
      matrices[currentMatrixIndex][rowIndex][colIndex] = selectedValue; // Update the cell value
    }
    console.log(rooms)
    renderMap();
  }

// Add a new matrix (floor)
function addMatrix() {
    const newMatrix = new Array(rows).fill(0).map(() => new Array(cols).fill(0));
    matrices.push(newMatrix);
    currentMatrixIndex = matrices.length - 1;
    renderMap();
    updateCarousel();
}

// Remove the current matrix (floor)
function removeMatrix() {
    if (matrices.length > 1) {
        matrices.splice(currentMatrixIndex, 1);
        currentMatrixIndex = Math.min(currentMatrixIndex, matrices.length - 1);
        renderMap();
        updateCarousel();
    }
}

// Add a new row to all matrices
function addRow() {
    matrices.forEach(matrix => matrix.push(new Array(cols).fill(0)));
    rows++;
    renderMap();
}

// Add a new column to all matrices
function addColumn() {
    matrices.forEach(matrix => matrix.forEach(row => row.push(0)));
    cols++;
    renderMap();
}

// Remove the last row from all matrices
function removeRow() {
    if (rows > 1) {
        matrices.forEach(matrix => matrix.pop());
        rows--;
        renderMap();
    }
}

// Remove the last column from all matrices
function removeColumn() {
    if (cols > 1) {
        matrices.forEach(matrix => matrix.forEach(row => row.pop()));
        cols--;
        renderMap();
    }
}

// Handle color legend clicks
document.querySelectorAll('.legend-item').forEach(item => {
    item.addEventListener('click', () => {
        // Remove the 'selected' class from all legend items
        document.querySelectorAll('.legend-item').forEach(i => i.classList.remove('selected'));
        // Add the 'selected' class to the clicked legend item
        item.classList.add('selected');
        // Update the selected value
        selectedValue = parseInt(item.getAttribute('data-value'));
    });
});

// Set the default selected legend item (room)
document.querySelector('.legend-item[data-value="2"]').classList.add('selected');

// Save the map
document.getElementById('save-map-form').addEventListener('submit', function (event) {
    event.preventDefault(); // Prevent the default form submission

    // Prepare the data to send
    const data = {
        id: mapId,
        matrices: matrices,
        rows: rows,
        cols: cols,
        rooms: rooms,
        name: document.getElementById('map-name').value,
    };

    // Send the data as JSON
    fetch(this.action, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'X-CSRFToken': getCookie('csrftoken'), // Include CSRF token
        },
        body: JSON.stringify(data),
    })
    .then(response => {
        if (response.redirected) {
            // Handle redirect
            window.location.href = response.url;
        } else {
            return response.json(); // Parse the response as JSON
        }
    })
    .then(data => {
        if (data && data.error) {
            console.error('Error saving matrix:', data.error);
        }
    })
    .catch(error => {
        console.error('Error during form submission:', error);
    });
});

// Get CSRF token from cookies
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

// Initial render
renderMap();

let unsavedChanges = false;

// Track changes in matrices or rooms
function trackChanges() {
    unsavedChanges = true;
}

// Mark changes when user modifies the map
document.getElementById('map').addEventListener('click', trackChanges);

// Warn the user if they try to leave without saving
window.addEventListener('beforeunload', (event) => {
    if (unsavedChanges) {
        event.preventDefault();
        event.returnValue = 'You have unsaved changes. Do you really want to leave?';
    }
});

// Reset flag when saving
document.getElementById('save-map-form').addEventListener('submit', () => {
    unsavedChanges = false;
});
