// Try to import RRLIB, but continue if it fails
let RRLIB = null;
let importFailed = false;

try {
    RRLIB = await import('../../js/modules/rrlib.js');
    console.log('Imported RRLIB successfully')
} catch (error) {
    console.warn('RRLIB import failed. Running in mock mode:', error);
    importFailed = true;
}

// ROS connection and utilities for web interface
class ROS_WEB {
    constructor() {
        this.ros = null;
        this.web_go_to = null;
        this.volume_pram = null;
        this.mockMode = importFailed;
    }

    // Initialize ROS connection
    init() {
        if (this.mockMode) {
            console.info('🤖 Running in MOCK MODE - No ROS functionality available');
            this._initMock();
        } else {
            console.log('running in realmode')
            this._initReal();
        }
        
        // Initialize volume slider
        this.initVolumeSlider();
        
        return this;
    }
    
    // Initialize with real ROS
    _initReal() {
        try {
            this.ros = new RRLIB.Ros({
                host: 'http://' + window.location.hostname
            });
            
            this.web_go_to = new RRLIB.Topic({
                ros: this.ros,
                name: "web_go_to"
            });
            
            this.volume_pram = new RRLIB.Param({
                ros: this.ros,
                name: "volume"
            });
            
            console.info('🤖 ROS connection initialized successfully');
        } catch (error) {
            console.error('Failed to initialize ROS:', error);
            // Fall back to mock mode if initialization fails
            this.mockMode = true;
            this._initMock();
        }
    }

    // Initialize volume slider with current value
    initVolumeSlider() {
        const volumeSlider = document.getElementById("volume_slider");
        if (volumeSlider) {
            this.volume_pram.get((current_vol) => { 
                volumeSlider.value = current_vol;
            });
        }
    }

    // Set the volume
    setVolume(value) {
        console.log("Setting volume to:", value);
        this.volume_pram.set(value);
    }

    // Send presentation selection command
    selectPresentation(buttonId) {
        console.log("Selected presentation:", buttonId);
        this.web_go_to.publish({
            type: 4,
            value: buttonId
        });
        
        // In mock mode, simulate a presentation change
        if (this.mockMode) {
            this._simulatePresentation(buttonId);
        }
    }
}

// Create global instance
const rosWeb = new ROS_WEB();

// Export functions for global access
window.Power_point_slected = ((Button_pressed) => {
    rosWeb.selectPresentation(Button_pressed.id);
});

window.set_vol = ((volume_slider) => {
    rosWeb.setVolume(volume_slider.value);
});

export default rosWeb;
