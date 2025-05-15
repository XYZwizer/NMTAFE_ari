import * as RRLIB from '../../../js/modules/rrlib.js'

// ***** SLOW START *****
$(document).ready(function() {
    $('#main-div').fadeIn(400);
});

class TermsCondMgr {
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
      terms_of_usage_preferences: true
    }, () => {
      this.web_input_pub.publish({
        action: 'INPUT_ACCEPT'
      });
    });
  }
}

let terms_cond_mgr = new TermsCondMgr();

$(document).ready(function() {
  $("#term_cond_chk").on("touchend", () => {
    $("#input_term_cond").click();
    if (document.getElementById("input_term_cond").checked == true) {
      $("#check-btn").removeClass("non-active-btn").addClass("go-btn");
      $("#check-icon").removeClass("grey-check").addClass("green-check");
    } else {
      $("#check-btn").removeClass("go-btn").addClass("non-active-btn");
      $("#check-icon").removeClass("green-check").addClass("grey-check");
    }
  });

  $("#check-btn").on('touchend', () => {
    if (!$("#check-btn").hasClass("non-active-btn")) {
      $("#check-btn").removeClass("go-btn").addClass("non-active-btn");
      terms_cond_mgr.setCfg();
    }
  })
});