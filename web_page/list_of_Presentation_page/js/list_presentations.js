import rosWeb from './ROS_WEB.js';

// Sample presentation data - replace this with actual data from your backend if available
const PRESENTATIONS = [
    { id: "test_page", displayName: "Test" },
    { id: "GLaDOS", displayName: "Glados Sound Board" },
    { id: "cyberwest_v5", displayName: "Cyber West" }
];

// Function to generate presentation buttons
function generatePresentationButtons(presentations) {
    const grid = document.getElementById('presentation-grid');
    grid.innerHTML = ''; // Clear existing content
    
    presentations.forEach(presentation => {
        const button = document.createElement('button');
        button.className = 'presentation-button';
        button.id = presentation.id || presentation.internalName;
        
        // Create display name element
        const displayNameElement = document.createElement('div');
        displayNameElement.className = 'display-name';
        displayNameElement.textContent = presentation.displayName || presentation.title || presentation.id;
        
        // Create internal name element (smaller text below)
        const internalNameElement = document.createElement('div');
        internalNameElement.className = 'internal-name';
        internalNameElement.style.fontSize = '12px';
        internalNameElement.style.opacity = '0.7';
        internalNameElement.style.marginTop = '5px';
        internalNameElement.textContent = presentation.id || presentation.internalName;
        
        // Add elements to button
        button.appendChild(displayNameElement);
        button.appendChild(internalNameElement);
        
        // Add click event
        button.addEventListener('click', function() {
            window.Power_point_slected(this);
        });
        
        grid.appendChild(button);
    });
	console.log("Presentation buttons generated:", presentations);
}
// Check if document is already loaded, otherwise wait for DOMContentLoaded
if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", initApp);
} else {
    // Document already loaded, run initialization immediately
    initApp();
}

function initApp() {
    console.log('onDomLoaded list');
    rosWeb.init();
    generatePresentationButtons(PRESENTATIONS);
    console.log("Presentation navigator loaded!");
}