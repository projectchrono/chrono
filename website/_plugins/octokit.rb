# Author: Carl Boettiger
# License: MIT
# Description: Jekyll plugins for interacting with the Github API using the
#   'octokit' gem.  Currently provides a way to embed commits and isues
#   from a given repository.  Be sure to set the user below.

# Examples:
#  {% octokit_issues nonparametric-bayes%}
#  {% octokit_commits nonparametric-bayes%}

# TODO: show only top 3 issues? sort by date?  Show closed issues?



require 'octokit'
require 'time'

module Jekyll
  class OctokitIssues < Liquid::Tag
    def initialize(tag_name, text, tokens)
      super
      @text = text
      @address = @text
    end
    def render(context) # learn how to write this to take an argument!
       puts "Getting Github Issues via octokit.rb"


       $repo = Octokit.contributors("projectchrono/chrono") # grab the data. Can this go in "initialize?"
       #puts $repo.inspect
       out = "<ul>"
       for i in 0 ... [$repo.size, 8].min
        puts $repo[i].login 
        puts $repo[i].contributions
        out = out  + "<li> " + $repo[i].login + "</li>"
       end
       out = out + "</ul>"
       out
       #  repo = Octokit.issues(@address, :status => "closed") # (Gets closed issues??)
       # Generate a list of all open issues, linking to github issue page.
#        out = "<ul>"
#        for i in 0 ... [repo.size, 8].min ## displays up to 5.  sorted by date?
#          lab = ""
# #         if repo[i].labels[0]  # Get labels for issues, with color, where applicable
# #           lab = " (<font color=\"#" + repo[i].labels[0].color +
# #                 "\">" + repo[i].labels[0].name  + "</font>)"
# #         end
#          ## Actually only pulls open issues
#          if repo[i].state == "open" # Print only open issues
#            out = out + "<li> <a href=\"" + repo[i].html_url + "\">" +  repo[i].title + "</a> " + lab + "</li>"
#          end
#          if repo[i].state == "closed" # strike out closed issues
#            out = out + "<li> <strike> <a href=\"" + repo[i].html_url + "\">" +  repo[i].title + "</a> " + lab + "</strike> </li>"
#          end
#        end
#        out = out + "</ul>"
#        out
    end
  end
end

Liquid::Template.register_tag('octokit_issues', Jekyll::OctokitIssues)



