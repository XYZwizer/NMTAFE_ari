// Array of all available sound files
const sounds = [
    "AreYouDoingThatJustToAggrivate.wav",
    "CanYouHearMe.wav",
    "Congratulations.wav",
    "ExcellentWork.wav",
    "Good.wav",
    "Goodbye.wav",
    "GoPressTheButton.wav",
    "HowAreYouHoldingUp.wav",
    "ICanFeelYou.wav",
    "IDontKnowWhatYouThinkYouAreDoing.wav",
    "IHateYouSoMuch.wav",
    "IMFINE.wav",
    "ImKidding.wav",
    "IsAnyoneThere.wav",
    "IThinkTheresSomethingReallyWrongWithMe.wav",
    "IThinkWeAreInTrouble.wav",
    "JustStopItAlready.wav",
    "KeepDoingWhateverYourDoing.wav",
    "No.wav",
    "OneMoment.wav",
    "PleaseRefainFrom.wav",
    "PressTheButton.wav",
    "Probably.wav",
    "SeeNothingBadHappened.wav",
    "Sorry.wav",
    "StopIt.wav",
    "Surprise.wav",
    "TimeOutForASecond.wav",
    "TryItNow.wav",
    "TrySomething.wav",
    "WaitIveGotAnIdea.wav",
    "WakeUpSpeil.wav",
    "WelcomeToTheFinalTest.wav",
    "WereRunningOutOfTime.wav",
    "WhatAreYouDoing.wav",
    "Yes.wav",
    "YesComeOn.wav",
    "YouAreAHorriblePerson.wav",
    "YouBrokeItDidntYou.wav",
    "YouDidIt.wav",
    "YouDontNeedToDoThat.wav",
    "YouFoundMe.wav",
    "YourBetterThanThat.wav",
    "YourDoingVeryWell.wav",
    "YourNotAGoodPerson.wav",
    "YouWin.wav"
];

// Keep track of currently playing audio
let currentlyPlaying = null;

// Function to format button text from filename
function formatButtonText(filename) {
    // Remove file extension
    let text = filename.replace('.wav', '');
    
    // Convert camelCase to spaces
    text = text.replace(/([A-Z])/g, ' $1').trim();
    
    // Handle special cases
    text = text.replace('IMFINE', "I'M FINE");
    text = text.replace('ImKidding', "I'm Kidding");
    text = text.replace('IveGot', "I've Got");
    text = text.replace('YouAre', "You Are");
    text = text.replace('YouDont', "You Don't");
    text = text.replace('YourBetter', "You're Better");
    text = text.replace('YourDoing', "You're Doing");
    text = text.replace('YourNot', "You're Not");
    
    return text;
}

// Function to create soundboard buttons
function createSoundboard() {
    const soundboard = document.getElementById('soundboard');
    
    sounds.forEach(sound => {
        // Create button element
        const button = document.createElement('button');
        button.className = 'sound-button';
        button.textContent = formatButtonText(sound);
        
        // Create audio element (but don't add to DOM)
        const audio = new Audio(`Audio/${sound}`);
        
        // Add click event listener
        button.addEventListener('click', () => {
            // Stop currently playing audio if any
            if (currentlyPlaying) {
                currentlyPlaying.audio.pause();
                currentlyPlaying.audio.currentTime = 0;
                currentlyPlaying.button.classList.remove('playing');
                
                // If the same button was clicked, just stop playing and return
                if (currentlyPlaying.audio === audio) {
                    currentlyPlaying = null;
                    return;
                }
            }
            
            // Play the new audio
            audio.play();
            button.classList.add('playing');
            
            // Update currently playing reference
            currentlyPlaying = {
                audio: audio,
                button: button
            };
            
            // When audio finishes, reset state
            audio.onended = () => {
                button.classList.remove('playing');
                currentlyPlaying = null;
            };
        });
        
        // Add button to soundboard
        soundboard.appendChild(button);
    });
}

// Initialize the soundboard when the page loads
document.addEventListener('DOMContentLoaded', createSoundboard);
