import * as RRLIB from '../../../js/modules/rrlib.js'
import '../lib/keyboard.min.js'

// ***** SLOW START *****
$(document).ready(function() {
    $('#main-div').fadeIn(400);
});

class ggPasswordInputManager {
	constructor() {
		this.ros = new RRLIB.Ros({
			host: 'http://' + window.location.hostname
		});
		this.web_input_pub = new RRLIB.Topic({
			ros: this.ros,
			name: '/NMTAFE_gandalf_game/password_attempt'
		});
	}

	try_password(password_text) {
		this.web_input_pub.publish({data:password_text})
	}
	/*setCfg() {
		let param = new RRLIB.Param({
			ros: this.ros,
			name: 'preferences_setup'
		});
		param.set({
			robot_name_preferences: $("#gg-password-input").val()
		}, () => {
			this.web_input_pub.publish({
				action: 'INPUT_ACCEPT'
			});
		});
	}*/
}

let pass_input_mgr = new ggPasswordInputManager();
$(document).ready(function() {
	$("#arrow-right-btn").on('touchend', () => {
		if (!$("#arrow-right-btn").hasClass("non-active-btn")) {
			$("#arrow-right-btn").removeClass("go-btn").addClass("non-active-btn");
			pass_input_mgr.try_password();			
		}
	});
});

/********************************************
 ************ Keyboard config ****************
 ********************************************/

let Keyboard = window.SimpleKeyboard.default;

let keyboard = new Keyboard({
	onChange: input => onChange(input),
	onKeyPress: button => onKeyPress(button)
});

function onChange(input) {
	document.querySelector(".input").value = input;
	if (input != "") {
		$("#arrow_icon").removeClass("non-active-icon").addClass("go-icon");
		$("#arrow-right-btn").removeClass("non-active-btn").addClass("go-btn");
	} else {
		$("#arrow_icon").removeClass("go-icon").addClass("non-active-icon");
		$("#arrow-right-btn").removeClass("go-btn").addClass("non-active-btn");
	}
}

function onKeyPress(button) {
	if (button == "{enter}") {
		animatedCloseKeyboard();
	}

	/**
	 * If you want to handle the shift and caps lock buttons
	 */
	if (button === "{shift}" || button === "{lock}") handleShift();
}

function handleShift() {
	let currentLayout = keyboard.options.layoutName;
	let shiftToggle = currentLayout === "default" ? "shift" : "default";

	keyboard.setOptions({
		layoutName: shiftToggle
	});
}

document.addEventListener("click", calculateCoordinates);

function calculateCoordinates(event) {
	// For Wifi keyboard visible
	if (document.getElementById("robot-name-keyboard").classList.contains("fadeInUp")) {
		// Conditional rules for indice zone out of keyboard
		if (event.clientY < 215 || event.clientY > 630) {
			animatedCloseKeyboard();
		} else if (event.clientX < 105 || event.clientX > 1180) {
			if (event.clientY > 215 && event.clientY < 630) {
				animatedCloseKeyboard();
			}
		}
	}
}

function animatedCloseKeyboard() {
	/* Close keyboard */
	$("#robot-name-keyboard").addClass("fadeOutDown d-none").removeClass("animatedFadeInUp fadeInUp");
	/* Reduce inputs */
	$("#gg-password-input").addClass("keyboard-input-closed").removeClass("keyboard-input-opened input-large");
	/* Edit margins */
	$("#robot-name-title").removeClass("mt-10").addClass("mt-15");
	$("#input-box").addClass("mt-10");
	$("#pin-visibility").css("display", "none");
}



