
// Access the element
const mapElement = document.getElementById('map');
const matrixString = mapElement.getAttribute('data-matrix'); // Get the JSON string*
const mapId = parseInt(mapElement.getAttribute('data-id')); // Get the JSON string*

let matrix;
try {
    matrix = JSON.parse(matrixString || '[]');
} catch (error) {
    console.error('Failed to parse JSON:', error);
}

let selectedValue = 2; // Default to room
let currentMapId = null; // Track the current map ID

console.log(mapElement, "test")

// Render the map grid
function renderMap() {
  mapElement.innerHTML = '';
  mapElement.style.gridTemplateColumns = `repeat(${matrix[0].length}, 50px)`;

  matrix.forEach((row, rowIndex) => {
      row.forEach((cell, colIndex) => {
          const cellElement = document.createElement('div');
          cellElement.classList.add('cell');
          if (cell === 0) cellElement.classList.add('empty');
          else if (cell === 1) cellElement.classList.add('obstacle');
          else if (cell === 2) cellElement.classList.add('room');
          cellElement.addEventListener('click', () => handleCellClick(rowIndex, colIndex));
          mapElement.appendChild(cellElement);
      });
  });
}

// Handle cell clicks
function handleCellClick(rowIndex, colIndex) {
  if (selectedValue === 2) {
      const roomName = prompt('Enter room name:');
      if (roomName) {
          matrix[rowIndex][colIndex] = 2;
          renderMap();
          saveMatrixToBackend(); // Sync with backend
      }
  } else {
      matrix[rowIndex][colIndex] = selectedValue;
      renderMap();
      saveMatrixToBackend(); // Sync with backend
  }
}

// Add a new row
function addRow() {
  const newRow = new Array(matrix[0].length).fill(0); // Default to empty
  matrix.push(newRow);
  renderMap();
}

// Add a new column
function addColumn() {
  matrix = matrix.map(row => [...row, 0]); // Default to empty
  renderMap();
}

// Remove the last row
function removeRow() {
  if (matrix.length > 1) {
      matrix.pop();
      renderMap();
  }
}

// Remove the last column
function removeColumn() {
  if (matrix[0].length > 1) {
      matrix = matrix.map(row => row.slice(0, -1));
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
// Get the map element and matrix data

// Function to save the matrix to the backend
function saveMatrixToBackend() {
    console.log('Map id:', mapId);
    console.log('Matrix being sent:', matrix);

    fetch(`/map/save-map/`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'X-CSRFToken': getCookie('csrftoken'), // Include CSRF token
        },
        body: JSON.stringify({
            id: mapId,
            matrix: matrix,
        })
    })
    .then(response => {
        if (!response.ok) {
            throw new Error(`HTTP error! Status: ${response.status}`);
        }
        return response.json();
    })
    .then(data => {
        if (data.message === 'Matrix saved successfully') {
            console.log('Matrix saved successfully.');
        } else {
            console.error('Error saving matrix:', data.error);
        }
    })
    .catch(error => {
        console.error('Error during saveMatrixToBackend:', error);
    });
}

// Handle form submission
document.getElementById('save-map-form').addEventListener('submit', function (event) {
    event.preventDefault(); // Prevent the default form submission

    // Update the hidden input fields with the current matrix and map ID
    document.getElementById('map-matrix').value = JSON.stringify(matrix);
    document.getElementById('map-id').value = mapId;

    // Submit the form data
    const formData = new FormData(this);
    fetch(this.action, {
        method: 'POST',
        body: formData,
        headers: {
            'X-CSRFToken': getCookie('csrftoken'), // Include CSRF token
        },
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
        if (data && data.message === 'Matrix saved successfully') {
            console.log('Matrix saved successfully.');
        } else if (data) {
            console.error('Error saving matrix:', data.error);
        }
    })
    .catch(error => {
        console.error('Error during form submission:', error);
    });
});

// Fetch matrix from backend
function fetchMatrixFromBackend(mapId) {
  fetch(`/get-matrix/${mapId}/`)
      .then(response => response.json())
      .then(data => {
          if (data.matrix) {
              matrix = data.matrix;
              renderMap();
          } else {
              console.error('Error fetching matrix:', data.error);
          }
      })
      .catch(error => {
          console.error('Error:', error);
      });
}

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