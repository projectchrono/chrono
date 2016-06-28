var HTML_base = "http://projectchrono.org/metrics/api";
google.charts.load("current", {"packages":["corechart"]});

var charts = [];

// Shows dropdown menu of all tests available
function showTestNames(test_list) {
    for (i = 0; i < test_list.length; i++) {
        test = test_list[i];
        // Makes a dropdown list of each available test
        var x = document.getElementById("test_names");
        var option = document.createElement("option");
        option.text = test.name;
        option.value = test.name;
        x.add(option);
    }
}
// Parses data for a given test and plots charts for each metric
function drawCharts(test_runs) {
    var run_names = []; // Number of machines + name
    var runs = []; //  2D Array containing. Cols =  Number of machines, Rows = Run
    var metrics =[];

    run_names = test_runs["run_names"];
    var len = run_names.length;

    /* Load runs for each machine */
    for (var i = 0; i < len; i++){
        run = run_names[i];
        runs.push(test_runs[run]);
    }
    /* Load metric names */
    for (metric in runs[0][0]["metrics"]) {
        metrics.push(metric);
    }  
    console.log(run_names, runs, metrics);
    
    plotProps(metrics, run_names, runs);

}
function makeChart(data, prop_name) {
    var options = { 
            title: prop_name, 
            legend: "bottom",
            // vAxis: {title: metric_name},
            // hAxis: {title: "timestamp"},
            explorer: {axis: "horizontal",
                actions: ["dragToZoom","rightClickToReset"]},
            chartArea: {
                height: "50%",
                width: "90%",
            },
        };

    var div = document.createElement("div");
    div.setAttribute("id", prop_name); 
    div.setAttribute("class", "metric");
    document.getElementById("metrics").appendChild(div);
    var chart = new google.visualization.ScatterChart(document.getElementById(prop_name));
    chart.draw(data, options);
    charts.push(chart);
}

// Plots charts for each property (metrics and execution time) of a given test
function plotProps(metrics, run_names, runs) {
    var timestamps = [];
    var base_table = [["x"]];
    // Sets up table of timestamps
    for (var n = 0; n < run_names.length; n++) {
        // Iterate through each run for that name
        base_table[0].push(run_names[n]);
        for (var m = 0; m < runs[n].length; m++) {
            var test_run = runs[n][m];
            var ts = new Date(test_run["timestamp"]);
            // Adds timestamp to list of timestamps iff not already in array
            var index = getObjectIndex(ts, timestamps);
            if ( index == -1) {
                timestamps.push(ts);
                base_table.push([ts]);
                // table[timestamps.length][n + 1] = test_run["metrics"][metric_name];
            } //else {
                // table[index + 1][n + 1].push(test_run["metrics"][metric_name]);
            // }            
        }
    }

    // Ensures each row has same length
    for (var n = 0; n < base_table.length; n++) {
        base_table[n].length = run_names.length + 1;
    }
    var table = base_table;
    // Plots execution times
    for (var n = 0; n < run_names.length; n++) {
        // Iterate through each run for that name
        for (var m = 0; m < runs[n].length; m++) {
            var test_run = runs[n][m];
            var ts = new Date(test_run["timestamp"]);
            var index = getObjectIndex(ts, timestamps);
            table[index + 1][n + 1] = test_run["execution_time"];
        }
    }
    console.log(table);
    var data = google.visualization.arrayToDataTable(table);

    makeChart(data, "Execution Times");
    console.log(metrics);
    for (var i = 0; i < metrics.length; i++) {
        var metric = metrics[i];
        console.log(i + ": " + metric);
        table = base_table;
        // Plots a chart for each metric
        for (var n = 0; n < run_names.length; n++) {
            // Iterate through each run for that name
            for (var m = 0; m < runs[n].length; m++) {
                var test_run = runs[n][m];
                console.log(test_run['metrics'][metric]);
                var ts = new Date(test_run["timestamp"]);
                var index = getObjectIndex(ts, timestamps);
                table[index + 1][n + 1] = test_run["metrics"][metric];
            }
        }
            // console.log(table);
    data = google.visualization.arrayToDataTable(table);
    makeChart(data, metric);
    }

}

function getObjectIndex(obj, arr) {
    for (var test_idx = 0; test_idx < arr.length; test_idx++) {
        if (arr[test_idx].valueOf() == obj.valueOf()) {
            return test_idx;
        }
    } 
    return -1;
}


// Sets up authentication info
$.ajaxSetup({
    crossDomain: true,
    xhrFields: {
        withCredentials: true
    },
    headers: {
        "Access-Control-Allow-Credentials": true,
        "Authorization": "Basic " + btoa("User:Password")
    }
});
// Gets list of test names
$.ajax({
        url: HTML_base + "/tests",
        method: "GET",
        data: "{};",
        dataType:"json",
        success: function (response, status, xhr) {
            console.log(response);
            // Changes "Test Loading!" text
            $("#test_names option:selected").html(" --- Select A Test --- ");
            showTestNames(response);
            },
        error: function (xhr, status, error_code) {
            console.log("Error:" + status + ": " + error_code);
        }
})

// Shows a test when selected from the dropdown menu
function showTest(test_name) {
    $("#metrics").empty(); // Clears metrics div so new charts can be shown
    if (test_name == "default") {
        return; //If the "Select A Test" option is selected
    }
    $.ajaxSetup({
        crossDomain: true,
        xhrFields: {
            withCredentials: true
        },
        headers: {
            "Access-Control-Allow-Credentials": true,
            "Authorization": "Basic " + btoa("User:Password")
        }
    });
    // Gets all test data for a given test
    $.ajax({
        url: HTML_base + "/tests/" + test_name,
        method: "GET",
        data: "{};",
        dataType:"json",
        success: function (response, status, xhr) {
            console.log(response);
            google.charts.setOnLoadCallback(drawCharts(response));
        },
        error: function (xhr, status, error_code) {
            console.log("Error:" + status + ": " + error_code);
        }
    })
}