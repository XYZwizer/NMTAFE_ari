// to able-disable Arrow button
function changeArrowClass() {
	$("#arrow_icon").removeClass("non-active-icon").addClass("go-icon");
	$("#arrow-right-btn").removeClass("non-active-btn").addClass("go-btn");
}

function animatedOpenKeyboard(input) {
	switch (input) {
		case 'wifi-name':
			/* Open keyboard */
			$("#wifi-keyboard").removeClass("d-none fadeOutDown").addClass("animated animatedFadeInUp fadeInUp");
			/* Expand active input */
			$("#wifi-name-keyboard-input").removeClass("keyboard-input-closed").addClass("keyboard-input-opened input-large");
			/* Hide another input */
			$("#pass-keyboard-input").addClass("d-none");
			$("#inputs-column").removeClass("mt-10");
			break;
		case 'pass':
			/* Open keyboard */
			$("#wifi-keyboard").removeClass("d-none fadeOutDown").addClass("animated animatedFadeInUp fadeInUp");
			/* Expand active input */
			$("#pass-keyboard-input").removeClass("keyboard-input-closed").addClass("keyboard-input-opened input-large");
			/* Hide another input */
			$("#wifi-name-keyboard-input").addClass("d-none");
			$("#inputs-column").removeClass("mt-10");
			/* Pass eye icon */
			$("#pin-visibility").removeClass("d-none").fadeIn(800);
			break;
		case 'robot-name':
			/* Open keyboard */
			$("#robot-name-keyboard").removeClass("d-none fadeOutDown").addClass("animated animatedFadeInUp fadeInUp");
			/* Expand active input */
			$("#robot-name-input").removeClass("keyboard-input-closed").addClass("keyboard-input-opened input-large");
			/* Edit margins */
			$("#robot-name-title").removeClass("mt-15").addClass("mt-10");
			$("#input-box").removeClass("mt-10");
			break;
		case 'pin-first':
			/* Open keyboard */
			$("#pin-keyboard").removeClass("d-none fadeOutDown").addClass("animated animatedFadeInUp fadeInUp");
			/* Hide elements */
			$("#pin-input-title").addClass("d-none");
			$("#pin-container-second").addClass("d-none");
			/* Edit margins */
			$("#pin-notice").addClass("mb-33px");
			$("#pin-input-box").removeClass("mt-4");
			break;
		case 'pin-second':
			/* Open keyboard */
			$("#pin-keyboard").removeClass("d-none fadeOutDown").addClass("animated animatedFadeInUp fadeInUp");
			/* Hide elements */
			$("#pin-input-title").addClass("d-none");
			$("#pin-container-first").addClass("d-none");
			/* Edit margins */
			$("#pin-notice").addClass("mb-33px");
			$("#pin-input-box").removeClass("mt-4");
			break;
	}
}


function changePassVisibility(location) {

	//Define the page
	if (location == 'network') {
		var pin_input = document.getElementById("pass-keyboard-input");
		var pass_eye_icon = document.getElementById("pin-visibility");
	} else if (location == 'master-pin-first') {
		var pin_input = document.getElementById("pin-input-first");
		var pass_eye_icon = document.getElementById("pin-eye-first");
	} else if (location == 'master-pin-second') {
		var pin_input = document.getElementById("pin-input-second");
		var pass_eye_icon = document.getElementById("pin-eye-second");
	}
	// Change PIN visibility
	if (pin_input.type === "password") {
		pin_input.type = "text";
		pass_eye_icon.src = "images/icons/eye_closed.svg";
	} else {
		pin_input.type = "password";
		pass_eye_icon.src = "images/icons/eye_opened.svg";
	}
}