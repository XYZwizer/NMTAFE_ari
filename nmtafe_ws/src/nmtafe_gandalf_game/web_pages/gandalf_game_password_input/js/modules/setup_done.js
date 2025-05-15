import * as RRLIB from '../../../js/modules/rrlib.js'

// ***** SLOW START *****
$(document).ready(function() {
    $('#main-div').fadeIn(400);
});


/* Copy from index.js*/
class DoneMgr {
	constructor() {
		this.ros = new RRLIB.Ros({
			host: 'http://' + window.location.hostname
		});
		this.web_input_pub = new RRLIB.Topic({
			ros: this.ros,
			name: 'user_input'
		});
	}

	done() {
		this.web_input_pub.publish({
			action: 'INPUT_ACCEPT'
		});
	}
}
let done_mgr = new DoneMgr();
$(document).ready(function() {
	$("#main-div").on('touchend', () => {
		done_mgr.done();
	});
});