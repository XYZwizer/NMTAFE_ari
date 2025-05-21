import * as RRLIB from '../../js/modules/rrlib.js'
/*
function get_pages() {
	fetch('http://' + window.location.hostname + '/touch_web_mgr').then(response => {
                    if (!response.ok) {
                        throw new Error(`HTTP error! Status: ${response.status}`);
                    }
                    return response.json();  
                })
                .then(data => console.log(data))  
                .catch(error => console.error('Failed to fetch data:', error)); 
}
*/
window.Power_point_slected = ((Button_pressed) => {
	console.log(Button_pressed)
	window.web_go_to.publish({
		type: 4,
		value: Button_pressed.id
	})
});

window.set_vol = ((volume_slider) => {
	console.log(volume_slider);
	window.volume_pram.set(volume_slider.value);
});

document.addEventListener("DOMContentLoaded", () => {
	var i_ros = new RRLIB.Ros({
		host: 'http://' + window.location.hostname
	});
	
	window.web_go_to = new RRLIB.Topic({
			ros: i_ros,
			name: "web_go_to"
	});
	
	window.volume_pram = new RRLIB.Param({
		ros: i_ros,
		name: "volume"
	});
	
	window.volume_pram.get((curent_vol) => { 
		document.getElementById("volume_slider").value = curent_vol;
	});
	
	console.log("Hello World!");
});


	//var motion_list_param = new RRLIB.Param({
	//	ros: i_ros,
	//	name: "motion_list"
	//});
	
	//motion_list_param.get(function(e) {
	//	console.log("got pram")
	//	console.log(e)
	//	document.getElementById("a_p").innerHTML = "someTHING DID!!!!";
	//});
	
	/*const container = document.getElementById('a_p');
	
	for (page in get_pages().page_list) {
	var newDiv = document.createElement('div');
	newDiv.className = 'formatted-div'; // Add a class for styling

// Add formatted HTML content
newDiv.innerHTML = `
  <p style="font-size: 16px; line-height: 1.5;">
    open ${page.title}
  </p>
`;

container.appendChild(newDiv);
	}*/