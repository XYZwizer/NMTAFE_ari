import * as RRLIB from '../../../js/modules/rrlib.js'

// ***** SLOW START *****
$(document).ready(function() {
  $('#main-div').fadeIn(400);
});

class LanguageMgr {
  constructor() {
    this.ros = new RRLIB.Ros({
      host: 'http://' + window.location.hostname
    });
    this.language_list_param = new RRLIB.Param({
      ros: this.ros,
      name: 'language_list'
    });
    this.web_input_pub = new RRLIB.Topic({
      ros: this.ros,
      name: 'user_input'
    });
    this.on_touchmove = false;
  }



  loadLangs() {
    this.language_list_param.get((param) => {
      param.forEach((lang) => {
        $("#list").append("<div class='option' id='" + lang + "-op'>" + lang + "</div>");
        $("#" + lang + "-op").on('touchend', () => {
          if (!this.on_touchmove) {
            $("#selection").html(lang);
            $("#list").toggleClass('list-display');
            $("#arrow-menu").toggleClass('arrow-menu-rotate');
            $("#choice").val(lang);
            hideOverflow();
            changeArrowClass();
          }
        });
      });
    });
  }
  setCfg() {
    let param = new RRLIB.Param({
      ros: this.ros,
      name: 'preferences_setup'
    });
    param.set({
      language_preferences: $("#choice").val()
    }, () => {
      this.web_input_pub.publish({
        action: 'INPUT_ACCEPT'
      });
    });
  }
  touchMoved() {
    this.on_touchmove = true;
  }
  resetTouch() {
    this.on_touchmove = false;
  }
}

let language_mgr = new LanguageMgr();

$(document).ready(function() {

  // Load languages
  language_mgr.loadLangs();
  $("#arrow-right-btn").on('touchend', () => {
    if (!$("#arrow-right-btn").hasClass("non-active-btn")) {
      $("#arrow-right-btn").removeClass("go-btn").addClass("non-active-btn");
      language_mgr.setCfg();
    }
  });

  $("#list").on('touchmove', () => {
    language_mgr.touchMoved();
  });
  $("#list").on('touchstart', () => {
    language_mgr.resetTouch();
  });
  $('#select-header').on('touchend', () => {
    $("#list").toggleClass('list-display');
    $("#arrow-menu").toggleClass('arrow-menu-rotate');
    hideOverflow();

  });

  // Close language menu with any click 
  $('body').on('touchend', (event) => {
    if (!$(event.target).hasClass('select') && !$(event.target).hasClass('selection') &&
      !$(event.target).hasClass('option') && !$(event.target).hasClass('arrow-menu')) {
      if ($('#list').hasClass("list-display")) {
        $("#list").removeClass('list-display');
        $("#arrow-menu").toggleClass('arrow-menu-rotate');
        hideOverflow();
      }

    }
    console.log("Event.target.id: " + event.target.id);
  })
});


function hideOverflow() {
  // Hidden overflow-y during slideDown animation
  if ($('#list').hasClass("list-display")) {
    $('#list').css('overflow', 'hidden');
    document.getElementById("list").addEventListener("transitionend", showOverflow);
  } else {
    $('#list').css('overflow', 'hidden');
  }
}

function showOverflow() {
  $('#list').css('overflow', 'auto');
}
