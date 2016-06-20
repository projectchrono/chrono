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

  // // If search originates from search bar
  // $("#search_bar_triggered").submit(function(event){
  //     event.preventDefault();
  //     console.log("Search bar triggered");
  //     var query = $("#search_bar").val(); // Get the value for the text field
  //     console.log("query is " + query);
      
  //     var results = window.idx.search(query); // Get lunr to perform a search
  //           console.log("results is " + results);
  //     $("#search_box").val(query);
  //     display_search_results(results); // Hand the results off to be displayed
  // });

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
        $search_results.html('<li>No results found.<br/>Please check spelling, spacing, etc...</li>');
      }
    });
  }

});


