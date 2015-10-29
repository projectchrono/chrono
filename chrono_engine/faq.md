---
layout: default
title: Frequently Asked Questions
permalink: /chrono_engine/faq/
---
General
-------

### Where can I start to read the documentation?

If you are a new user, we suggest you to start from [the main page of
this WIKI](Main_Page "wikilink"), or directly go to [this quick
installation guide](Installation "wikilink").

### Does CHRONO::ENGINE have a graphical user interface?

No, it is a C++ library and you must be a software programmer in order
to take advantage of it. However, if you need a CAD-like interface, you
may give a look at
[Chrono::SolidWorks](ChronoSolidWorks:Introduction "wikilink"), our
add-in for the SolidWorks CAD; this allows you to create the assets for
Chrono::Engine simulations using powerful graphical-user interface
tools.

### Is CHRONO::ENGINE free, or should I pay a fee?

Since 2013, Chrono::Engine got a permissive license of BSD style, so you
can use it freely in your projects (read the details in the license.txt
file in the repository).

### I want to use CHRONO::ENGINE, but I have few knowledge about C++ language..

Using CHRONO::ENGINE requires adequate knowledge about the C++ language.
If you don't know what's 'templating', 'polimorphism' or such, please
learn some basic lessons about C++ (for example see [these
links](http://www.deltaknowledge.com/chronoengine/links.html)).

### How can I contribute to the development of Chrono::Engine?

Look at the instructions in the page about the [GIT
repository](ChronoEngine:GIT_repository "wikilink"). If you feel like
contributing to the development, just fork the chrono GIT, then submit
us a pull request when you think you have interesting contributions.

Build
-----

### Can CHRONO::ENGINE be compiled on platform XXX with compiler YYY?

Yes, probably it can, but currently we test it under Windows (32 and 64
bit, XP and Vista, with MingW GNU compiler and Microsoft VisualC++
compilers) and Linux (32 and 64 bit, GNU compiler)

### Should I need to build all sub units?

No, to keep dependencies as few as possible, Chrono::Engine is designed
in a way so that advanced features, that might depend on additional
libraries or tools, are grouped into
[units](ChronoEngine:Units "wikilink") that can be selected in the CMake
interface when setting up the build.

Programming
-----------

### I see that CHRONO::ENGINE implements a 'fake RTTI' mechanism, mostly used by the class factory. Why doesn't you use the ready-to-use typeid mechanism of default C++ RTTI?

Well, though C symbol decoration is always the same, different compilers
implement different schemes for name decoration of C++ class members.
Therefore, if you use the RTTI name decoration for a polimorphic
serialization mechanism, your files won't be platform-independent. Since
our class factory is based on a custom RTTI, the class factory of the
persistent/transient deserialization will always work, regardless of
compiler/platform.
