Chrono Frequently Asked Questions {#faq_root}
==========================


### Does Chrono have a graphical user interface?

No, it doesn't. However, if you need a CAD-like interface, consider the use of the 
[Chrono::SolidWorks](@ref tutorial_table_of_content_chrono_solidworks) plug-in. It allows a work flow in which you can develop a model in SolidWorks&copy; and then export it as a model that can be subsequently imported and simulated in Chrono.

### Is Chrono free?
The entire {% include module.html first="PROJECT" second="CHRONO" %} software infrastructure is open source and released under a permissive BSD3 license. As such, you can use, modify, redistribute, sell, etc., the software as you wish. 

### Who owns Chrono?
Chrono is owned and copyrighted by {% include module.html first="PROJECT" second="CHRONO" %}, a nonprofit set up in the US. 

### I want to use Chrono. Do I have to be a good programmer?
No. You only need to be able to install the software and go through a couple of [examples/tutorials](@ref tutorial_root) to get the gist of it. Moreover, if you need a jump start to your project you might want to take a look at the [Chrono model repository](@ref model_root). If you are lucky, you'll find a model close to what you need, in which case you'll get a jump start to your project.

### How can I contribute?
You can contribute in many ways:
- If you want to add to the {% include module.html first="PROJECT" second="CHRONO" %} software infrastructure, make a pull request in [GitHub](https://github.com/projectchrono/chrono)
- If you put together a model and want to make it part of the [Chrono model repository](@ref model_root) you can make a pull request in [GitHub](https://github.com/projectchrono/chrono)
- If you put together a tutorial or generated a well documented Chrono example, follow the same [GitHub](https://github.com/projectchrono/chrono) pull request path
- If you feel like Chrono is the best thing since slice bread and you want to support its development, make a tax deductible donation to the University of Wisconsin-Madison or University of Parma, Italy. Let us know if you follow this  path since we want to  channel your financial contribution into our labs to fund the development of Chrono.
- If you can't make a donation but still like Chrono, you can still contribute by letting us know you used Chrono and sharing your experience with us

### Can Chrono be compiled on platform X with compiler Y?
That's pretty likely to happen. Currently we build Chrono under Windows (32 and 64
bit with the MingW GNU compiler,  Microsoft VisualC++, and Intel
compilers) and Linux (32 and 64 bit, GNU and Intel compilers)

### Should I need to build all sub units?

No, to keep dependencies as few as possible, Chrono::Engine is designed
in a way so that advanced features, that might depend on additional
libraries or tools, are grouped into
[units](ChronoEngine:Units "wikilink") that can be selected in the CMake
interface when setting up the build.

##Programming

### I see that CHRONO::ENGINE implements a 'fake RTTI' mechanism, mostly used by the class factory. Why doesn't you use the ready-to-use typeid mechanism of default C++ RTTI?

Well, though C symbol decoration is always the same, different compilers
implement different schemes for name decoration of C++ class members.
Therefore, if you use the RTTI name decoration for a polimorphic
serialization mechanism, your files won't be platform-independent. Since
our class factory is based on a custom RTTI, the class factory of the
persistent/transient deserialization will always work, regardless of
compiler/platform.

##Chrono Engine Workflow

The ProjectChrono ecosystem includes tools, plugins and libraries, most
of them revolving around the Chrono::Engine middleware.

The Chrono::Engine middleware is a modular set of libraries for physical
simulations, organized in [units](Units "wikilink"). Some of those units
allow interfacing Chrono::Engine to external software, for example for
pre-processing and post-processing data, or for displaying real-time
simulations with OpenGL, or for parsing Python commands.

This means that there are many options for the assets workflow. In a
broad context, the following picture shows some of these possibilities:

![](/images/Workflow.png "workflow.png")
