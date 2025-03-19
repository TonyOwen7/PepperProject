document.getElementById('pepperForm').addEventListener('submit', function(event) {
    event.preventDefault();  // Prevent the default form submission

    // Create an object to store the form data
    const formData = {
        robot_ip: document.getElementById('robot_ip').value,
        network_interface: document.getElementById('network_interface').value,
        language: document.querySelector('input[name="language"]:checked').value,
    };

    // Send the JSON data to the server using fetch
    fetch('/submit/', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',  // Set the content type to JSON
            'X-CSRFToken': document.querySelector('[name=csrfmiddlewaretoken]').value,  // Include the CSRF token
        },
        body: JSON.stringify(formData),  // Convert the object to a JSON string
    })
    .then(response => {
        if (response.ok) {
            return response.json();  // Parse the JSON response
        } else {
            throw new Error('Network response was not ok.');
        }
    })
    .then(data => {
        if (data.redirect_url) {
            window.location.href = data.redirect_url;  // Redirect to the provided URL
        }
    })
    .catch(error => {
        console.error('Error:', error);  // Log any errors
        alert('An error occurred. Please try again.');
    });
});