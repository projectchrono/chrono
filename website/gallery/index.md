---
layout: default
title: Gallery
permalink: /gallery/
---

In this page you can see some demo animations showing the main features of Chrono::Engine. 

{% for link in site.data.gallery.main %}
<div class="media">
	{% if link.host =='youtube' %}
		<a class="media-left" href="#"> 
			<iframe width="420" height="315" src="http://www.youtube.com/embed/{{ link.url }}" frameborder="0" allowfullscreen></iframe>
		</a>
	{% elsif link.host =='vimeo' %}
	<a class="media-left" href="#"> 
		<iframe src="https://player.vimeo.com/video/{{ link.url }}?byline=0&portrait=0" width="420" height="315" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe> 
	</a>
	{% endif %}
	<div class="media-body">
		<h3 class="media-heading">{{ link.title }}</h3> 
		<p>{{ link.description }}</p>
	</div>
</div>
{% endfor %}
