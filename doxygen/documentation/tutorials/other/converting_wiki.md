Converting from MediaWiki to Doxygen {#tutorial_converting_wiki}
==========================

##Reference Links

[Doxygen Manual](https://www.stack.nl/~dimitri/doxygen/manual/)



###Links

A mediawiki link ```[http://www.povray.org POVray]``` becomes ```[POVray](http://www.povray.org)``` in markdown

###Referencing a page in doxygen

If the page starts with

~~~
About Chrono::Solidworks {#introduction_chrono_solidworks}
==========================
~~~

In the header you can reference it with the ```@ref``` command

~~~
[Chrono::SolidWorks](@ref introduction_chrono_solidworks)
~~~

###Headings 
In mediawiki ```===``` specified headings, in markdown ```###``` specifies a heading, the more pound symbols there are the lower the heading level. Example: ```===Create a column===``` becomes ```###Create a column```


### Images
Images must be in the documentation/images/ folder to be picked up by doxygen, if you would like to add more folders modify the ```IMAGE_PATH             = documentation/images/``` setting in the doxfile.

Images can be added as follows

~~~
![](SWaddin.jpg)
~~~


###Converting Details Blocks

~~~
{{Details|content=
We suggest to install also the following third party packages for expanding the capabilities of Python in mathematical ad plotting areas:
* [http://numpy.scipy.org/ Numpy]
* [http://matplotlib.sourceforge.net/ Matplotlib].

NOTE. Precompiled binaries of Numpy and Matplotlib for Python 3.2 can be downloaded  from the unofficial [http://www.lfd.uci.edu/~gohlke/pythonlibs/ repository]. A faster option is to install the entire [http://www.lfd.uci.edu/~gohlke/pythonlibs/#scipy-stack SciPy stack] that includes both.
Otherwise there is a custom Python distribution called [http://enthought.com/products/epd.php Enthough] that already includes the two packages.
}}
~~~


~~~
<div class=well>
We suggest to install also the following third party packages for expanding the capabilities of Python in mathematical ad plotting areas:
* [http://numpy.scipy.org/ Numpy]
* [http://matplotlib.sourceforge.net/ Matplotlib].

NOTE. Precompiled binaries of Numpy and Matplotlib for Python 3.2 can be downloaded  from the unofficial [repository](http://www.lfd.uci.edu/~gohlke/pythonlibs/). A faster option is to install the entire [stack](http://www.lfd.uci.edu/~gohlke/pythonlibs/#scipy-stack SciPy) that includes both.
Otherwise there is a custom Python distribution called [Enthough](http://enthought.com/products/epd.php) that already includes the two packages.
</div>
~~~


~~~
{{Details|content=
The files for this demo can be found in the directory ''C:\Program Files\SolidWorks Corp\SolidWorks\chronoengine\examples\collisions''. The directory contains all the parts needed for this assembly.
}}
~~~

~~~
<div class=well>
The files for this demo can be found in the directory ```C:\Program Files\SolidWorks Corp\SolidWorks\chronoengine\examples\collisions```. The directory contains all the parts needed for this assembly.
</div>
~~~



### Code blocks

~~~
<source lang="py">
brick_material = chrono.ChMaterialSurfaceShared()
brick_material.SetFriction(0.6)
brick_material.SetDampingF(0.05)
brick_material.SetCompliance (0.000000003)
brick_material.SetComplianceT(0.000000001)
</source>
~~~

Becomes

```
~~~{.py}
brick_material = chrono.ChMaterialSurfaceShared()
brick_material.SetFriction(0.6)
brick_material.SetDampingF(0.05)
brick_material.SetCompliance (0.000000003)
brick_material.SetComplianceT(0.000000001)
~~~
```

### Emphasis

~~~
''but it won't produce any collisions yet!''
~~~

~~~
**but it won't produce any collisions yet!**
~~~



### Lists

~~~
*First, start SolidWorks.

*Use menu File/New... and create a new Part.

*In the Part editor, create a doric column (like those that you can find in Greek temples) by using the [[File:Tutorial_collshapes_01.jpg]] '''Revolved boss / base''' tool in the toolbar. (Just create a profile like a rectangle, where one of the vertical sides is rather an almost flat arc, and the opposite side is the axis of revolution).

*You should obtain this:
~~~
Becomes (spacing important!)

~~~
* First, start SolidWorks.
* Use menu File/New... and create a new Part.
* In the Part editor, create a doric column (like those that you can find in Greek temples) by using the [[File:Tutorial_collshapes_01.jpg]] '''Revolved boss / base''' tool in the toolbar. (Just create a profile like a rectangle, where one of the vertical sides is rather an almost flat arc, and the opposite side is the axis of revolution).
* You should obtain this:
~~~


