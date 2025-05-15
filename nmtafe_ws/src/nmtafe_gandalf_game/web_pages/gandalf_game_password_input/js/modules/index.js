import * as RRLIB from '../../../js/modules/rrlib.js'

class IndexMgr {
	constructor() {
		this.ros = new RRLIB.Ros({
			host: 'http://' + window.location.hostname
		});
		this.web_input_pub = new RRLIB.Topic({
			ros: this.ros,
			name: 'user_input'
		});
	}

	start() {
		this.web_input_pub.publish({
			action: 'INPUT_ACCEPT'
		});
	}
}
let index_mgr = new IndexMgr();
$(document).ready(function() {
	$("#main").on('touchend', () => {
		index_mgr.start();
	});
});