import * as RRLIB from '../../../js/modules/rrlib.js'
import '../lib/keyboard.min.js'

// ***** SLOW START *****
$(document).ready(function() {
  $('#main-div').fadeIn(400);
});

class NetworkManager {
  constructor() {
    this.ros = new RRLIB.Ros({
      host: 'http://' + window.location.hostname
    });
    this.web_input_pub = new RRLIB.Topic({
      ros: this.ros,
      name: 'user_input'
    });
    this.config = {
      mode: '',
      ssid: '',
      pwd: ''
    };
  }

  goToNextScreen(screen) {
    switch (screen) {
      case 'choice-page':
        // Hide current page
        $("#net-info-page").css("display", "none");
        $("#net-arrow-btn").css("display", "none");
        // Show next page
        $("#choice-page").fadeIn(400);
        $("#choice-btns").fadeIn(400);
        break;
      case 'access-point-input':
        this.config.mode = "AP";
        // Hide current page
        $("#choice-page").css("display", "none");
        $("#choice-btns").css("display", "none");
        // Show next page
        $("#network_inputs").fadeIn(400);
        $("#inputs-arrow-btn").fadeIn(400);
        $("#back-arrow-btn").fadeIn(400);
        $("#ap-bg-icon").fadeIn(400);
        break;
      case 'wifi-input':
        this.config.mode = "Client";
        // Hide current page
        $("#choice-page").css("display", "none");
        $("#choice-btns").css("display", "none");
        // Show next page
        $("#network_inputs").fadeIn(400);
        $("#inputs-arrow-btn").fadeIn(400);
        $("#back-arrow-btn").fadeIn(400);
        $("#wifi-bg-icon").fadeIn(400);
        break;
    }
  }
  goToPreviousScreen() {
    // Hide current page
    $("#network_inputs").css("display", "none");
    $("#inputs-arrow-btn").css("display", "none");
    $("#back-arrow-btn").css("display", "none");
    $("#ap-bg-icon").css("display", "none");
    $("#wifi-bg-icon").css("display", "none");
    // Show next page
    $("#choice-page").fadeIn(400);
    $("#choice-btns").fadeIn(400);
  }
  checkValidConfig() {
    if ($("#wifi-name-keyboard-input").val() != "" && $("#pass-keyboard-input").val() != "") {
      $("#inputs-arrow-btn").removeClass("non-active-btn").addClass("go-btn");
      $("#inputs_arrow_icon").removeClass("non-active-icon").addClass("go-icon");
    }
  }
  setConfig() {
    // Set param
    this.config.ssid = $("#wifi-name-keyboard-input").val();
    this.config.pwd = $("#pass-keyboard-input").val();
    let param = new RRLIB.Param({
      ros: this.ros,
      name: 'preferences_setup'
    });
    param.set({
      wifi_preferences: this.config
    }, () => {
      this.web_input_pub.publish({
        action: 'INPUT_ACCEPT'
      });
    });

    // Disable buttons
    $("#inputs-arrow-btn").removeClass("go-btn").addClass("non-active-btn");
    $("#inputs_arrow_icon").removeClass("go-icon").addClass("non-active-icon");
  }
};

let net_mgr = new NetworkManager();
$(document).ready(function() {
  $("#net-arrow-btn").on("touchend", () => {
    net_mgr.goToNextScreen('choice-page');
  });
  $("#access-point-btn").on("touchend", () => {
    net_mgr.goToNextScreen('access-point-input');
  });
  $("#wifi-btn").on("touchend", () => {
    net_mgr.goToNextScreen('wifi-input');
  });
  $("#back-arrow-btn").on("touchend", () => {
    net_mgr.goToPreviousScreen();
  });
  $("#inputs-arrow-btn").on("touchend", () => {
    if (!$("#inputs-arrow-btn").hasClass("non-active-btn"))
      net_mgr.setConfig();
  });

  $("#wifi-name-keyboard-input").on('touchend', () => {
    animatedOpenKeyboard('wifi-name');
    selectedInput = '#wifi-name-keyboard-input';
    keyboard.setOptions({
      inputName: 'wifi-name-keyboard-input'
    });
  })
  $("#pass-keyboard-input").on('touchend', () => {
    animatedOpenKeyboard('pass');
    selectedInput = '#pass-keyboard-input';
    keyboard.setOptions({
      inputName: 'pass-keyboard-input'
    });
  })
});

/********************************************
 ************ Keyboard config ****************
 ********************************************/

let Keyboard = window.SimpleKeyboard.default;

let keyboard = new Keyboard({
  onChange: input => onChange(input),
  onKeyPress: button => onKeyPress(button)
});

let selectedInput;

/**
 * Update simple-keyboard when input is changed directly
 */
document.querySelectorAll(".input").forEach(input => {
  input.addEventListener("focus", onInputFocus);
});

function onInputFocus(event) {
  selectedInput = `#${event.target.id}`;

  keyboard.setOptions({
    inputName: event.target.id
  });
}

function onChange(input) {
  // console.log("Input changed", input);
  document.querySelector(selectedInput || ".input").value = input;
  net_mgr.checkValidConfig();
}

function onKeyPress(button) {
  console.log(button);
  if (button == "{enter}") {
    animatedCloseKeyboard();
  }

  /**
   * Shift functionality
   */
  if (button === "{lock}" || button === "{shift}") handleShiftButton();
}

function handleShiftButton() {
  let currentLayout = keyboard.options.layoutName;
  let shiftToggle = currentLayout === "default" ? "shift" : "default";

  keyboard.setOptions({
    layoutName: shiftToggle
  });
}

document.addEventListener("click", calculateCoordinates);

function calculateCoordinates(event) {
  // For Wifi keyboard visible
  if (document.getElementById("wifi-keyboard").classList.contains("fadeInUp")) {
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
  $("#wifi-keyboard").addClass("fadeOutDown d-none").removeClass("animatedFadeInUp fadeInUp");
  /* Reduce inputs */
  $("#pass-keyboard-input").addClass("keyboard-input-closed").removeClass("keyboard-input-opened input-large");
  $("#wifi-name-keyboard-input").addClass("keyboard-input-closed").removeClass("keyboard-input-opened input-large");

  /* Show all inputs */
  $("#wifi-name-keyboard-input").removeClass("d-none");
  $("#pass-keyboard-input").removeClass("d-none");
  $("#inputs-column").addClass("mt-10");
  $("#pin-visibility").css("display", "none");
}

//Developer test
/*
 $('body').on('touchend', (event) => {
    console.log("Event.target.id: " + event.target.id);
  })
  */