---
layout: default
title: Status
permalink: /status/
---
{::options parse_block_html="true" /}

### Project Chrono Benchmark Test Results:

** NOTICE ** 
---
The benchmark API feature is still in development and may have bugs. All data is for testing purposes only and does not reflect any real information about Project Chrono's performance. 
<html>
<body>

<script src="https://code.jquery.com/jquery-2.2.4.min.js" integrity="sha256-BbhdlvQf/xTY9gja0Dq3HiwQF8LaCRTXxZKRutelT44=" crossorigin="anonymous"></script>

<script type="text/javascript" src="https://www.gstatic.com/charts/loader.js"></script>

<select id="test_names" onchange="showTest(value);">
    <option value="default"> Tests Loading! </option>
</select>

<div id="metrics" ></div>
<script>
{% include plot_charts.js %}
</script>
</body>
</html>

### Stats

<script type="text/javascript" src="https://www.openhub.net/p/projectchrono/widgets/project_languages?format=js"></script>
<script type="text/javascript" src="https://www.openhub.net/p/projectchrono/widgets/project_factoids_stats?format=js"></script>

<br><br>

