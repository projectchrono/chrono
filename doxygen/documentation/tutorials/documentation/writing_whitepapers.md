Writing white papers       {#tutorial_writing_whitepapers}
==========================

The documentation of the more 'technical' and theorietical parts
of Chrono::Engine can be put in LaTeX documents to generate _white papers_
like the one that you can see [in the white papers page](@ref whitepaper_root).

To this end, we provide a **chrono.cls** LaTeX class that you can use instead of
the article class.


## Find the chrono.cls LaTeX class

- you can find ```chrono.cls``` and ```logo_chrono_engine_h200.png``` in the 
  ```chrono\docs\latex_white_papers``` directory of the code in the GIT repository.
  
- move those files in a directory where you have your LaTeX article, say ```myarticle.tex```

## Use the chrono.cls LaTeX class

- In your ```myarticle.tex``` file, start with 
  ~~~
  \documentclass{chrono}
  ~~~
  
- include additional packages if needed:
  ~~~
  \usepackage{graphicx}
  \usepackage{amsmath, amssymb} 
  \usepackage[english]{alg}
  \usepackage{empheq}
  ~~~

- The rest of the paper should have a structure like this:
  ~~~
  \begin{document}
  
  \title{My title here}

  \author{John Foo}
  
  \maketitle
  \thispagestyle{fancy}
  
  \begin{abstract} 
  Bla bla..
  \end{abstract}

  # put sections here...
  
  \end{document}
  ~~~

## References and links

The ```chrono.cls``` class provides some macros to create links to the Chrono::Engine API.

- For links to a C++ class of Chrono::Engine: ```\urlChronoAPIclass```. 
  For example
  ```\urlChronoAPIclass{chrono::ChMatrix}``` or ```\urlChronoAPIclass{chrono::fea::ChElement}```.
  Note the namespaces must be added. This macro turns the class name into the decorated
  name of the .html page of the Chrono API documentation generated with Doxygen.

- For links to a C++ namespace of Chrono::Engine: ```\urlChronoAPInamespace```. 
  For example ```\urlChronoAPInamespace{fea}```
  
- For links to a C++ module of Chrono::Engine: ```\urlChronoAPImodule```. Pass the name 
  of the Chrono module in upper case.  
  For example ```\urlChronoAPImodule{FEA}``` or ```\urlChronoAPIclass{MATLAB}```.
  
- For typing the Chrono::Engine name with custom font, use ```\Chrono```.

- For typing the Chrono::Engine name with custom font that can be clicked as a 
  link to www.projectchrono.org, use ```\urlChrono```.
  
## Where to put the PDF?

Once you generated the PDF, you can put it on some online repository, or in a FTP site. 
Then, add the link to such document in the ```whitepapers.md```  document in 
```chrono\doxygen\documentation\whitepapers```.

<div class="ce-warning">
Compiling these white paper LaTeX sources is not integrated in the automatic Doxygen toolchain, 
so it is up to you to build, check the cross links, upload, etc.
<div>

 
