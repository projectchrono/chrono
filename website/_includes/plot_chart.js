google.charts.setOnLoadCallback(drawChart);
function drawChart() {
  var data = google.visualization.arrayToDataTable({{% include.chart %}});

  var options = {
    title: 'Age of sugar maples vs. trunk diameter, in inches',
    hAxis: {title: 'Diameter'},
    vAxis: {title: 'Age'},
    legend: 'none',
    trendlines: { 0: {} }    // Draw a trendline for data series 0.
  };

  var chart = new google.visualization.ScatterChart(document.getElementById('chart_div'));
  chart.draw(data, options);
}