---
layout: default
title: Dashboard
permalink: /dashboard/
---
{::options parse_block_html="true" /}

benchmark_ChBody Test Results:

<html>
<body>
<div id='metrics' style="width: 900px; height: 500px;">
</div>
<script type="text/javascript" src="https://www.gstatic.com/charts/loader.js"></script>
<script src="https://code.jquery.com/jquery-2.2.4.min.js" integrity="sha256-BbhdlvQf/xTY9gja0Dq3HiwQF8LaCRTXxZKRutelT44=" crossorigin="anonymous"></script>

<script type="text/javascript">
var HTML_base = "http://localhost:5000/chrono_test/api";
google.charts.load('current', {'packages':['corechart']});

var first = true;
var charts = [];
var tests;

function showTestNames(test_list) {
	console.log(test_list);
	for (test in test_list) {
		var div = document.createElement("a");
        div.setAttribute('id', test.name); 
        div.setAttribute('href', HTML_base + "/test.name");
        div.innerHTML = test.name;

	}
}


function drawCharts(run_name) {
    var run_names = []; // Number of machines + name
    var runs = []; //  2D Array containing. Cols =  Number of machines, Rows = Run
    var metrics =[];

    if(first){
        run_names = tests['run_names'];

        /* Load runs for each machine */
        for(var i = 0; i < run_names.length; i++){
            run = run_names[i];
            runs.push(tests[run]);
        }

        /* Load metric names */
        for(metric in runs[0][0]['metrics']){
            metrics.push(metric);
        }  
    }
    else{
        run_name = run_name.getAttribute('id');
        run_names.push(run_name);
        runs.push(tests[run_name]);
    }
    console.log(run_names, runs, metrics);     

	plot(metrics, run_names, runs);

}


function plot(metrics, run_names, runs){
	for(var i = 0; i < metrics.length; i++){

		metric_name = metrics[i];
		table = [['x']];

		for(var j = 0; j < run_names.length; j++){
			table[0].push(run_names[j]);
		}

		for(var j = 0; j < runs[0].length; j++){
			date = new Date(runs[0][j].timestamp);
			row = [date];

			for(var k = 0; k < runs.length; k++){
				try{
					data_point = runs[k][j].metrics[metric_name];
				}catch(TypeError){
					console.log(runs[k][j]);
					data_point = 0;
				}
				row.push(data_point);
			}
			table.push(row);
		}

		var data = google.visualization.arrayToDataTable(table);
		var options = { 
						title: metric_name, 
						legend: { position: 'bottom' },
						vAxis: {title: metric_name},
          				hAxis: {title: "Timestamp"},
						explorer: {axis: 'horizontal',
								   actions: ['dragToZoom','rightClickToReset'],
								   maxZoomIn: 0}
					   };

		if(first){
			var div = document.createElement("div");
        	div.setAttribute('id', metric_name); // and make sure myclass has some styles in css
        	div.setAttribute('class', 'metric');
        	document.getElementById('metrics').appendChild(div);

        	var chart = new google.visualization.ScatterChart(
        									document.getElementById(metric_name));
        	chart.draw(data, options);
        	charts.push(chart);
    	}
    	else{
    		var div = document.getElementById(metric_name);
    		charts[i].draw(data, options)

    	}

	}
	if(first){
		first = false;
	}
}



$.ajaxSetup({
	crossDomain: true,
	xhrFields: {
		withCredentials: true
	},
	headers: {
		'Access-Control-Allow-Credentials': true,
		'Authorization': "Basic " + btoa("User:Password")
	}
});

$.ajax({
		url: HTML_base + "/tests/benchmark_ChBody",
		method: "GET",
		data: "{};",
		dataType:"json",
		success: function (response, status, xhr) {
			console.log(response);
			tests = response;
			google.charts.setOnLoadCallback(drawCharts(tests));
		},
		error: function (xhr, status, error_code) {
			console.log("Error:" + status + ": " + error_code);
		}
})
$.ajax({
		url: HTML_base + "/tests",
		method: "GET",
		data: "{};",
		dataType:"json",
		success: function (response, status, xhr) {
			console.log(response);
			showTestNames(response);
			},
		error: function (xhr, status, error_code) {
			console.log("Error:" + status + ": " + error_code);
		}
})


function parseJSON(result) {
	test_names = result['run_names'];
	//Initializes test_data
	var test_data = [];
	for (i = 0; i < 3; i++) {
		test_data[i]=[];

	}
	// Runs through each Test Name 
	for (i = 0; i < test_names.length; i++) {
		test_runs = result[test_names[i]];

		// Run data for one run test
		var test_run_data = [];
		// Different timesteps for this test
		var timestamps = []; 
		var metric_names = [];
		// Holds array of test results for each test
		var metric_results = [];
		// Finds all metrics corresponding to the current test
		for (var metric in test_runs[0]['metrics']) {
			metric_names.push(metric);
		}
		for (i = 0; i < test_runs.length; i++) {
			run = test_runs[i];				
			metrics = run['metrics'];
			// Records Timestamp for each run
			timestamps.push( formatTimestamp(run['timestamp']) );

			for (metric in metrics) {
			test_run_data.push(metrics[metric])
			}
			metric_results.push(test_run_data);
		}
		test_data[0].push(timestamps);
		test_data[1].push(metric_names);
		test_data[2].push(metric_results);
	}
	return test_data;
}

function formatTimestamp(date) {
	var new_date = new Date(date);
	return new_date;
}
</script>
</body>
</html>

