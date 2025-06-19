import rosWeb from './ROS_WEB.js';

// Sample presentation data - replace this with actual data from your backend if available
const PRESENTATIONS = [
    { id: "test_page", displayName: "Test", hidden: true },
    { id: "GLaDOS", displayName: "Glados Sound Board", hidden: true },
    { id: "cyberwest_v6", displayName: "Cyber West" },
    { id: "joondulup_v2", displayName: "Joondulup" },
    { id: "murray_selfie", displayName: "Selfie Cam" },
];

// Hidden presentations state
let showHiddenPresentations = false;
let secretClickCount = 0;
const REQUIRED_CLICKS = 20;

// Function to generate presentation buttons
function generatePresentationButtons(presentations) {
    const grid = document.getElementById('presentation-grid');
    grid.innerHTML = ''; // Clear existing content
    
    presentations.forEach(presentation => {
        // Skip hidden presentations if they shouldn't be shown
        if (presentation.hidden && !showHiddenPresentations) {
            return;
        }
          const button = document.createElement('button');
        button.className = 'presentation-button';
        button.id = presentation.id || presentation.url || presentation.internalName;
        
        // Store presentation data on the button for easy access
        button.presentationData = presentation;
        
        // Add special styling for URL presentations
        if (presentation.url) {
            button.classList.add('url-presentation');
        }
        
        // Add special styling for hidden presentations
        if (presentation.hidden) {
            button.classList.add('hidden-presentation');
        }
        
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
        
        // Add hidden indicator for hidden presentations
        if (presentation.hidden) {
            const hiddenIndicator = document.createElement('div');
            hiddenIndicator.className = 'hidden-indicator';
            hiddenIndicator.style.fontSize = '10px';
            hiddenIndicator.style.opacity = '0.5';
            hiddenIndicator.style.marginTop = '3px';
            hiddenIndicator.textContent = 'Devmode';
            button.appendChild(hiddenIndicator);
        }
        
        // Add elements to button
        button.appendChild(displayNameElement);
        button.appendChild(internalNameElement);
          // Add click event
        button.addEventListener('click', function() {
            const presentation = this.presentationData;
            if (presentation.url) {
                // Handle URL presentations
                rosWeb.goToPage(presentation.url, true);
            } else {
                // Handle regular ID-based presentations
                rosWeb.goToPage(presentation.id, false);
            }
        });
        
        grid.appendChild(button);
    });
	console.log("Presentation buttons generated:", presentations.filter(p => !p.hidden || showHiddenPresentations));
}

// Function to create the secret unlock button
function createSecretButton() {
    const secretButton = document.createElement('div');
    secretButton.id = 'secret-unlock-button';
    secretButton.style.position = 'fixed';
    secretButton.style.bottom = '20px';
    secretButton.style.right = '20px';
    secretButton.style.width = '35px';
    secretButton.style.height = '35px';
    secretButton.style.backgroundColor = 'transparent';
    secretButton.style.border = 'none';
    secretButton.style.cursor = 'pointer';
    secretButton.style.zIndex = '1000';
    secretButton.style.borderRadius = '50%';
    secretButton.title = `Click ${REQUIRED_CLICKS} times to unlock hidden presentations`;
    
    // Add click handler
    secretButton.addEventListener('click', function() {
        secretClickCount++;
        console.log(`Secret clicks: ${secretClickCount}/${REQUIRED_CLICKS}`);
        
        // Visual feedback for clicks
        this.style.backgroundColor = 'rgba(255,255,255,0.1)';
        setTimeout(() => {
            this.style.backgroundColor = 'transparent';
        }, 100);
        
        // Show progress feedback
        if (secretClickCount >= REQUIRED_CLICKS - 5 && secretClickCount < REQUIRED_CLICKS) {
            this.style.border = '1px solid rgba(255,255,255,0.3)';
            this.textContent = REQUIRED_CLICKS - secretClickCount;
            this.style.color = 'white';
            this.style.display = 'flex';
            this.style.alignItems = 'center';
            this.style.justifyContent = 'center';
            this.style.fontSize = '12px';
        }
        
        // Unlock hidden presentations
        if (secretClickCount >= REQUIRED_CLICKS) {
            unlockHiddenPresentations();
        }
    });
    
    document.body.appendChild(secretButton);
}

// Function to unlock hidden presentations
function unlockHiddenPresentations() {
    if (!showHiddenPresentations) {
        showHiddenPresentations = true;
        console.log('Dev presentations unlocked!');
        
        // Show notification
        showUnlockNotification();
        
        // Regenerate buttons to include hidden ones
        generatePresentationButtons(PRESENTATIONS);
        
        // Update secret button to show it's unlocked
        const secretButton = document.getElementById('secret-unlock-button');
        if (secretButton) {
            secretButton.style.backgroundColor = 'rgba(0,255,0,0.3)';
            secretButton.style.border = '1px solid rgba(0,255,0,0.5)';
            secretButton.textContent = '✓';
            secretButton.style.color = 'lightgreen';
            secretButton.title = 'Dev presentations unlocked!';
        }
    }
}

// Function to show unlock notification
function showUnlockNotification() {
    const notification = document.createElement('div');
    notification.style.position = 'fixed';
    notification.style.top = '50%';
    notification.style.left = '50%';
    notification.style.transform = 'translate(-50%, -50%)';
    notification.style.backgroundColor = 'rgba(0,0,0,0.9)';
    notification.style.color = 'white';
    notification.style.padding = '20px 30px';
    notification.style.borderRadius = '10px';
    notification.style.fontSize = '18px';
    notification.style.zIndex = '2000';
    notification.style.boxShadow = '0 4px 20px rgba(0,0,0,0.5)';
    notification.innerHTML = `
        <div style="text-align: center;">
            <div>Hidden Presentations Unlocked!</div>
        </div>
    `;
    
    document.body.appendChild(notification);
    
    // Auto-remove notification after 3 seconds
    setTimeout(() => {
        document.body.removeChild(notification);
    }, 3000);
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
    try{
        rosWeb.init();
    }catch(error){
        console.error('Failed to initialize ROS:', error);
    }
    generatePresentationButtons(PRESENTATIONS);
    createSecretButton();
    console.log("Presentation navigator loaded!");
}