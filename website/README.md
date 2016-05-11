# redesign.projectchrono.org
A replacement for the current project chrono wiki

to compile/generate this website locally:

install ruby 
- if using choclatey on windows: "choco install ruby"
- linux: use the package manager to install ruby
- osx should have ruby by default

install the jekyll ruby gem
- "gem install jekyll"
install the octokit ruby gem
- "gem install octokit"

in this directory run "jekyll serve", this does several things:
- compiles the website into a "_site" directory (DO NOT COMMIT THIS INTO GIT, the .gitignore should take care of this)
- executes a webserver at "localhost:4000" that you can see in your web browser
- watches this directory for any changes and regenerates the website as needed

jekyll will keep watching and regenerating the website until it is closed


For projectchrono.org and api.projectchrono.org:

hookshot is running under sbel_bot
'forever list' to check, look at the listed log file to see whats up

To reset the hook:

forever restart hookshot.js

hookshot.js will execute build_chrono_website.sh which will pull the latest changes, compile the website and then copy it to the site directory. build_chrono_website.sh will also run doxygen and update that on every push to the repository. Note that only the "develop" branch is currently watched

For more information see [here](http://hamelot.co.uk/other/jekyll-github-hook/)