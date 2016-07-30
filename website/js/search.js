---
---
jQuery(function() {
  // Initialize lunr with the fields to be searched, plus the boost.
  console.log("Initializing lunr");
  window.idx = lunr(function () {
    this.field('id');
    this.field('title');
    this.field('content', { boost: 10 });
  });

  // Get the generated search_data.json file so lunr.js can search it locally.
  window.data = $.getJSON('/search_data.json');

  console.log("Loading site data into lunr");
  // Wait for the data to load and add it to lunr
  window.data.then(function(loaded_data){
    $.each(loaded_data, function(index, value){
      window.idx.add(
        $.extend({ "id": index }, value)
        );
    });
  });

  // Event when the form is submitted
  $("#site_search").submit(function(event){
    event.preventDefault();
    console.log("Search box triggered");

      // If query was passed from search bar
      console.log("Reading from search box");
      var query = $("#search_box").val(); // Get the value for the text field
      search_documentation(query);

      console.log("query is " + query);

      var results = window.idx.search(query); // Get lunr to perform a search
      console.log("results is " + results);
      $("#search_bar").val(query);
      display_search_results(results); // Hand the results off to be displayed
    });

  function display_search_results(results) {
    var $search_results = $("#search_results");

    // Wait for data to load
    window.data.then(function(loaded_data) {

      // Are there any results?
      if (results.length) {
        $search_results.empty(); // Clear any old results

        // Iterate over the results
        results.forEach(function(result) {
          var item = loaded_data[result.ref];

          // Build a snippet of HTML for this result
          var appendString = '<li><a href="' + item.url + '">' + item.title + '</a></li>';

          // Add the snippet to the collection of results.
          $search_results.append(appendString);
        });
      } else {
        // If there are no results, let the user know.
        $search_results.html('<li>No website results found.<br/>Please check spelling, spacing, etc...</li>');
      }
    });
  }
  // Searches through doxygen site using embedded doxysearch capabilities
  // See https://www.stack.nl/~dimitri/doxygen/manual/extsearch.html for more information
  function search_documentation(query) {
    var doc_url = "https://projectchrono.org";
    var doc_search_url = "/doxygen";
    var page = "0";
    var number = "40";
    var callback = "docshow"
    var search_string = "?q=" + query +"&n=" + number + "&p=" + page + "&cb=" + callback;
    $.ajax({
      url: doc_url + doc_search_url + search_string,
      method: "GET",
      data: "",
      dataType:"jsonp",
      jsonpCallback: "docshow",
      success: function (response, status, xhr) {
        docshow(response);
        console.log(response);
      },
      error: function (xhr, status, error_code) {
        console.log("Error:" + status + ": " + error_code);
      }
    })
  }
  function docshow(result) {
    console.log("result is: " + JSON.stringify(result, null, 4));
    var hits = result["items"];
    var num_hits = result["hits"];
    console.log(num_hits + " hits.");
    $hits_div = $("#num_hits_div") 
    $hits_div.html(num_hits + " results found. ");
    if (num_hits > 40) {
      $hits_div.append("Showing the first 40...");
    }
    

    var ul = document.getElementById("doc_results");
    ul.innerHTML="";
    for (var i = 0; i < hits.length; i++) {
      var hit = hits[i];
      // console.log(hit);
      var name = hit["name"];
      var url = hit["url"];
      var url_base = "https://api.projectchrono.org/";
      url = url_base + url;
      var li = document.createElement("li");
      ul.appendChild(li);
      var a = document.createElement("a");
      li.appendChild(a);
      a.setAttribute("href", url);
      a.innerHTML = name;
    }
  }

});


