import * as RRLIB from '../../../js/modules/rrlib.js'
import '../lib/keyboard.min.js'

// ***** SLOW START *****
$(document).ready(function() {
	$('#main-div').fadeIn(400);
});

class PinManager {
	constructor() {
		this.ros = new RRLIB.Ros({
			host: 'http://' + window.location.hostname
		});
		this.web_input_pub = new RRLIB.Topic({
			ros: this.ros,
			name: 'user_input'
		});
	}

	setCfg() {
		let param = new RRLIB.Param({
			ros: this.ros,
			name: 'preferences_setup'
		});
		param.set({
			admin_pin_preferences: $("#pin-input-first").val()
		}, () => {
			this.web_input_pub.publish({
				action: 'INPUT_ACCEPT'
			});
		});
	}

	animatedOpenKeyboard(input) {
		switch (input) {
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
		// Disable cursor to avoid text select
		$(".input").css("pointer-events", "none");
	}
}

let pin_mgr = new PinManager();

$(document).ready(function() {
	$("#arrow-info").on('touchend', () => {
		goToNextScreen();
	});
	$("#back-arrow-btn").on("touchend", () => {
	    goToPreviousScreen();
	  });
	$("#arrow-pin").on('touchend', () => {
		if (!$("#arrow-pin").hasClass("non-active-btn")) {
			$("#arrow-pin").removeClass("go-btn").addClass("non-active-btn");
			pin_mgr.setCfg();
		}
	});
	$("#pin-input-first").on('touchend', () => {
		pin_mgr.animatedOpenKeyboard('pin-first');
		selectedInput = '#pin-input-first';
		keyboard.setOptions({
			inputName: 'pin-input-first'
		})
	});	
	$("#pin-input-second").on('touchend', () => {
		pin_mgr.animatedOpenKeyboard('pin-second');
		selectedInput = '#pin-input-second';
		keyboard.setOptions({
			inputName: 'pin-input-second'
		})
	});

	
	
})

/********************************************
 ************ Keyboard config ****************
 ********************************************/

let Keyboard = window.SimpleKeyboard.default;

let keyboard = new Keyboard({
	onChange: input => onChange(input),
	onKeyPress: button => onKeyPress(button),
	layout: {
		default: ["1 2 3", "4 5 6", "7 8 9", "{bksp} 0 {enter}"]
	},
	display: {
		'{enter}': 'OK',
		'{bksp}': 'Back'
	},
	theme: "hg-theme-default hg-layout-numeric numeric-theme"
});

let selectedInput;
/**
 * Function to able arrow btn if...
 */
function onChange(input) {
	document.querySelector(selectedInput || ".input").value = input;
	// PIN validation
	checkValidPIN();
}

function activeArrowBtn() {
	$("#arrow_icon_pin").removeClass("non-active-icon").addClass("go-icon");
	$("#arrow-pin").removeClass("non-active-btn").addClass("go-btn");
}

function disableArrowBtn() {
	$("#arrow_icon_pin").removeClass("go-icon").addClass("non-active-icon");
	$("#arrow-pin").removeClass("go-btn").addClass("non-active-btn");
}

function onKeyPress(button) {
	if (button === "{enter}") {
		animatedCloseKeyboard();
	}
}

document.addEventListener("click", calculateCoordinates);


function calculateCoordinates(event) {
	// If the keyboard is visible
	if (document.getElementById("pin-keyboard").classList.contains("fadeInUp")) {
		// Conditional rules for indice zone out of keyboard
		if (event.clientY < 175 || event.clientY > 630) {
			animatedCloseKeyboard();
		} else if (event.clientX < 380 || event.clientX > 880) {
			animatedCloseKeyboard();
		}
	}
}

function animatedCloseKeyboard() {
	/* Close keyboard */
	$("#pin-keyboard").addClass("fadeOutDown d-none").removeClass("animatedFadeInUp fadeInUp");
	/* Edit margins */
	$("#pin-input-title").removeClass("d-none mt-7").addClass("mt-13");
	$("#pin-input-box").addClass("mt-4");
	/* Show hidded elemets */
	$("#pin-notice").removeClass("d-none mb-33px");
	$("#pin-container-first").removeClass("d-none");
	$("#pin-container-second").removeClass("d-none");
	// Able cursor to click and open keyboard
		$(".input").css("pointer-events", "auto");
	// PIN validation
	checkValidPIN();
}

function checkValidPIN() {
	let input_first = $("#pin-input-first").val();
	let input_second = $("#pin-input-second").val();

	// FIRST input opened
	if ($("#pin-container-second").hasClass("d-none")) {
		if (input_first.length < 4) {
			$("#pin-notice").text("Choose a PIN code with at least 4 digits").css("color", "#6f6f6f");
		} else {
			$("#pin-notice").text("PIN code is valid").css("color", "#008000");
		}
		// SECOND opened
	} else if ($("#pin-container-first").hasClass("d-none")) {
		if (input_first !== input_second) {
			$("#pin-notice").text("Repeat the PIN code").css("color", "#6f6f6f");
		} else if (input_first == input_second && input_first.length > 3) {
			$("#pin-notice").text("The PIN code is valid").css("color", "#008000");
		}
		// BOTH closed
	} else if (!$("#pin-container-first").hasClass("d-none") && !$("#pin-container-second").hasClass("d-none")) {
		if (input_first && !input_second) {
			if (input_first.length < 4) {
				$("#pin-notice").text("The PIN code must have at least 4 digits").addClass("shake-effect").css("color", "#bf0000");
				setTimeout(function() {
					$("#pin-notice").removeClass("shake-effect");
				}, 1000);
				disableArrowBtn()
			} else {
				$("#pin-notice").text("Repeat the PIN code").css("color", "#6f6f6f");
			}
		} else if (input_first !== input_second) {
			if (input_first.length > 3) {
				$("#pin-notice").text("The PIN codes are not equal").addClass("shake-effect").css("color", "#bf0000");
				setTimeout(function() {
					$("#pin-notice").removeClass("shake-effect");
				}, 1000);
				disableArrowBtn()
			} else {
				$("#pin-notice").text("The PIN code must have at least 4 digits").addClass("shake-effect").css("color", "#bf0000");
				setTimeout(function() {
					$("#pin-notice").removeClass("shake-effect");
				}, 1000);
				disableArrowBtn()
			}

		} else if (input_first == input_second) {
			if (input_first.length > 3) {
				$("#pin-notice").text("The PIN code is valid").css("color", "#008000");
				activeArrowBtn();
			} else if (input_first.length < 4) {
				$("#pin-notice").text("The PIN code must have at least 4 digits").addClass("shake-effect").css("color", "#bf0000");
				setTimeout(function() {
					$("#pin-notice").removeClass("shake-effect");
				}, 1000);
				disableArrowBtn()

			}
		}
	}
}

function goToNextScreen() {
	// Hide current page
	$("#pin-info-page").css("display", "none");
	$("#arrow-info").css("display", "none");
	// Show Master PIN page
	$("#master-pin-page").fadeIn(400);
	$("#arrow-pin").fadeIn(400);
	$("#back-arrow-btn").fadeIn(400);
}
function goToPreviousScreen() {
	// Hide current page
	$("#master-pin-page").css("display", "none");
	$("#arrow-pin").css("display", "none");
	$("#back-arrow-btn").css("display", "none");
	// Show Info page
	$("#pin-info-page").fadeIn(400);
	$("#arrow-info").fadeIn(400);
}