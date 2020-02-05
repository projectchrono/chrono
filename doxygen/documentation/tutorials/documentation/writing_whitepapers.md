Writing white papers       {#tutorial_writing_whitepapers}
==========================

A 'white paper' is a document that provides technical/analytical details that anchor the software implementation in Chrono.
In order to maintain some sense of uniformity between these white paper documents it is highly recommended that the source document be generated in LaTeX using a Chrono specific document type. Specifically, we provide a **chrono.cls** LaTeX class that you can use instead of the article class. This is how the docs in the [white papers page](http://projectchrono.org/whitepapers/) have been generated.


## Finding the chrono.cls LaTeX class

- you can find ```chrono.cls``` and ```logo_projectchrono_h200.png``` in the ```docs\latex_white_papers``` directory of the repository.
  
- copy those files to a directory where you generate your LaTeX document (say ```mywhitepaper.tex```)

## Use the chrono.cls LaTeX class

- In your ```mywhitepaper.tex``` file, start with 
~~~{.tex}
\documentclass{chrono}
~~~

- include additional packages if needed:
~~~{.tex}
\usepackage{graphicx}
\usepackage{amsmath, amssymb} 
\usepackage[english]{alg}
\usepackage{empheq}
~~~

- The rest of the document should have a structure like this:
~~~{.tex}
\begin{document}

\title{My title here}

\author{John Doe}

\maketitle
\thispagestyle{fancy}

\begin{abstract} 
Blah blah..
\end{abstract}

# add sections here...

\end{document}
~~~

## References and links

The ```chrono.cls``` class provides some macros to create links to the Chrono API.

- For links to a C++ class of Chrono: ```\urlChronoAPIclass```. <br>
  For example: ```\urlChronoAPIclass{chrono::ChMatrix}``` or ```\urlChronoAPIclass{chrono::fea::ChElement}```.<br>
  Note the namespaces must be added. This macro turns the class name into the decorated
  name of the .html page of the Chrono API documentation generated with Doxygen.

- For links to a C++ namespace of Chrono: ```\urlChronoAPInamespace```. 
  For example: ```\urlChronoAPInamespace{fea}```
  
- For links to a C++ module of Chrono: ```\urlChronoAPImodule```. Pass the name 
  of the Chrono module in upper case.  
  For example: ```\urlChronoAPImodule{FEA}``` or ```\urlChronoAPIclass{MATLAB}```.
  
- For typing the Chrono name with custom font, use ```\Chrono```.

- For typing the Chrono name with custom font that can be clicked as a 
  link to www.projectchrono.org, use ```\urlChrono```.
  
## Where should the PDF be stored?

Once you generated the PDF, you can store it in a repository or in an FTP site. 
Then, add the link to your document in the ```whitepapers.md``` file in ```chrono\doxygen\documentation\whitepapers```.

<div class="ce-warning">
Compiling these white paper LaTeX sources is not integrated in the automatic Doxygen toolchain, 
so it is up to you to build, check the cross links, upload, etc.
</div>

 
