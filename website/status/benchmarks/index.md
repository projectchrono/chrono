---
layout: default
title: Benchmarks
permalink: /status/benchmarks/
---
{::options parse_block_html="true" /}

### Project Chrono Benchmark Test Results:

<html>
<body>

<select id='test_names' onchange="showTest(value);">
    <option value='default'> --- Select A Test --- </option>
</select>

<div id='metrics' ></div>
{% include plot_charts.js %}

</body>
</html>

