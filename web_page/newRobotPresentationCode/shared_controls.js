async function goToPage(page) {
    if (!page) return;
    
    try {
        const response = await fetch('http://ari-20c/topic/web_go_to', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                type: "4",
                value: page,
            })
        });
        
        console.log('Go to page response:', await response.json());
    } catch (error) {
        console.error('Error calling goToPage:', error);
    }
}

// Function to call the motion endpoint
async function doMotion(motion) {
    if (!motion) return;
    
    try {
        const response = await fetch('http://ari-20c/action/motion_manager', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                filename: motion
            })
        });
        
        console.log('Motion response:', await response.json());
    } catch (error) {
        console.error('Error calling motion:', error);
    }
}

// Function to call the TTS endpoint
async function sayText(text) {
    if (!text) return;

    const replaceList = [ {'joondalup': 'joondalahpppp'}, {'tafe': 'tayfe'} ];
    for (const replace of replaceList) {
        const [key, value] = Object.entries(replace)[0];
        //replace all occurrences of the key with the value (dont be case sensitive)
        const regex = new RegExp(key, 'gi');
        text = text.replace(regex, value);
    }

    //en_GB, de_DE
    
    try {
        const response = await fetch('http://ari-20c/action/tts', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                rawtext: {
                    text: text,
                    lang_id: "en_GB"
                }
            })
        });
        
        console.log('TTS response:', await response.json());
    } catch (error) {
        console.error('Error calling TTS:', error);
    }
}