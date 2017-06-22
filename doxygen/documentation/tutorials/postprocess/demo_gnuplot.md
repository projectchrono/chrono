GNUplot example (demo_POST_gnuplot.cpp)  {#tutorial_demo_gnuplot}
==========================

Tutorial that teaches how to use the 
[POSTPROCESS module](group__postprocess__module.html)
to create graphs with [GNUPLOT](http://www.gnuplot.info). 

There is a @ref chrono::postprocess::ChGnuPlot class that helps you to create .gpl 
gnuplot scripts directly from your cpp program.

Of course you could create the gpl script by yourself, save a .dat file 
from your program, and then launch gnuplot by hand, 
but this class generates the script for you and makes the graph 
creation easier. Also, it automatically calls gnuplot.

There are two prerequisites:

- [GNUPLOT](http://www.gnuplot.info) must be installed on your computer. 
  We support releases from v.4.6. You can install either 32 or 64 bit versions from their site.

- the `gnuplot` command must be accessible from the shell. 
  On Linux, Unix etc. this is ok by default. 
  On Windows, you must remember to add the bin/ directory of `gnuplot.exe` to your PATH. 
  That is, if all is ok, when you type `gnuplot` in your cmd window, 
  the gnuplot should start.

In the following you can see some examples of using the ChGnuPlot class.


# Example 1

The most low-level way of using the ChGnuPlot class: use the the 
SetCommand() or alternatively the << operator, to build the .gpl script.

Such script will be saved on disk with the specified .gpl name. 

~~~{.cpp}
ChGnuPlot mplot("__tmp_gnuplot_1.gpl");
mplot << "set contour";
mplot << "set title 'Demo of specifying discrete contour levels'";
mplot << "splot x*y";
~~~

When the mplot object goes out of scope and is deleted 
at the end of this example, the script is saved on disk and GNUplot is launched.

You should see the following windows that opens:

![](http://projectchrono.org/assets/manual/Tutorial_gnuplot1.png)

<div class="ce-info">
Troubleshooting. If you do not see the window opening, check the following: 
<br>
... if you open a shell (a DOS cmd window in Windows) and type `gnuplot`, 
  does gnuplot start? If the program is not found, you should install gnuplot.
<br>
... make sure you can access it from the shell (add it to your PATH in windows)
<br>
... in case the .gpl command script contains errors, you can see which is 
  the offending statement by opening a shell, go to the current working directory 
  of the demo_POST_gnuplot executable using cd, and entering 
  `gnuplot __tmp_gnuplot_1.gpl`  where __tmp_gnuplot_1.gpl is the name 
  of the gpl file, here is for this example. 
  When executed it will prompt script errors, if any.
</div>


# Example 2

Learn how to use the Open... functions to define the output terminal.
The gnuplot utility can create plots in windows, or it can save them 
as EPS or JPG or PNG or other formats. 

This is a demo that opens two plots in two windows and save the 
second to an EPS file too.

Note that we use `SetGrid()`, `SetLabelX()`, `SetLabelY()`. 
There are other functions like these. They are shortcuts to alleviate 
you from entering complex gnuplot commands. For instance 
`mplot.SetLabelX("x");` is equivalent to 
`mplot << "set xlabel \"x\"";`, etc.

~~~{.cpp}
ChGnuPlot mplot("__tmp_gnuplot_2.gpl");
mplot.SetGrid();
~~~

Then we redirect a first plot to a window n.0:

~~~{.cpp}
mplot.OutputWindow(0);
mplot.SetLabelX("x");
mplot.SetLabelY("y");
mplot << "plot [-30:20] besj0(x)*0.12e1 with impulses, (x**besj0(x))-2.5 with points";
~~~

We redirect a following plot to a window n.1:

~~~{.cpp}
mplot.OutputWindow(1);
mplot.SetLabelX("v");
mplot.SetLabelY("w");
mplot << "plot [-10:10] real(sin(x)**besj0(x))";
~~~

Finally we redirect a plot to an .EPS Postscript file, 
that can be used, for example, in LaTeX documents. 
Note that here we do not issue another plot command, 
but we rather use `Replot()`, that repeats the last plot command (the one of the window n.1).

~~~{.cpp}
mplot.OutputEPS("test_eps.eps");
mplot.Replot(); // repeat last plot
~~~

You should see the following two plots in two separate windows:

![](http://projectchrono.org/assets/manual/Tutorial_gnuplot2.png)

![](http://projectchrono.org/assets/manual/Tutorial_gnuplot3.png)


# Example 3

Learn how to use the `Plot()` shortcut functions, for easy plotting 
from a .dat file (an ascii file with column ordered data). 

Of course you could do this by just adding gnuplot commands such as 

~~~{.cpp}
	mplot << "plot \"test_gnuplot_data.dat\" 1:2 
~~~

etc., but the `Plot()` functions make this easier.

Step 1: create a .dat file with three columns of demo data:

~~~{.cpp}
ChStreamOutAsciiFile mdatafile("test_gnuplot_data.dat");
for (double x = 0; x<10; x+=0.1)
   mdatafile << x << ", " << sin(x) << ", "  << cos(x) << "\n";
~~~

Step 2: Create the plot. 

Note the Plot() shortcut: In this case you pass the .dat 
filename, the columns IDs, title and custom settings that 
define the plotting line type, width, etc. (see the gnuplot documentation)

~~~{.cpp}
ChGnuPlot mplot("__tmp_gnuplot_3.gpl");
mplot.SetGrid();
mplot.SetLabelX("x");
mplot.SetLabelY("y");
mplot.Plot("test_gnuplot_data.dat", 1,2, "sine",   " with lines lt -1 lw 2");
~~~

Note that you can have multiple Plot() calls for a single Output, 
they will be overlapped as when you use commas in gnuplot:  `plot ... , ... , ...`.
For instance, here overlap a 2nd plot to the previous one:

~~~{.cpp}
mplot.Plot("test_gnuplot_data.dat", 1,3, "cosine", " with lines lt 2 lw 2");
~~~	

You should see the following plot:

![](http://projectchrono.org/assets/manual/Tutorial_gnuplot4.png)


# Example 4

To make things even simpler, the Plot() shortcut function can be used directly 
on embedded data, without needing to save a .dat file. 

One can use, for instance:
- a pair of x,y vectors (use @ref chrono::ChVectorDynamic column matrices),
- @ref chrono::ChFunction y(x) objects,
- @ref chrono::ChFunction_Recorder and @ref chrono::ChFunction_Oscilloscope objects, that 
  are optimized for recording xy pairs,
- columns from a @ref chrono::ChMatrix, etc.

The data values will be saved _embedded_ in the .gpl file.

Note, the Replot() command does not work with embedded data.

Let's begin with creating some example data: 

~~~{.cpp}
//create demo data in a pair of x,y vectors
ChVectorDynamic<> mx(100);
ChVectorDynamic<> my(100);
for (int i=0; i<100; ++i)
{
	double x = ((double)i/100.0)*12;
	double y = sin(x)*exp(-x*0.2);
	mx(i)=x;
	my(i)=y;
}
// ..or create demo data in a ChFunction_Recorder
ChFunction_Recorder mfun;
for (int i=0; i<100; ++i)
{
	double x = ((double)i/100.0)*12;
	double y = cos(x)*exp(-x*0.4);
	mfun.AddPoint(x,y);
}
// ..or create demo data in two columns of a ChMatrix
ChMatrixDynamic<> matr(100,10);
for (int i=0; i<100; ++i)
{
	double x = ((double)i/100.0)*12;
	double y = cos(x)*exp(-x*0.4);
	matr(i,2)=x;
	matr(i,6)=y*0.4;
}
~~~

Now create the plot. 
Note the Plot() shortcuts. 

~~~{.cpp}
ChGnuPlot mplot("__tmp_gnuplot_4.gpl");
mplot.SetGrid();
mplot.Plot(mx,my, "from x,y ChVectorDynamic",  " every 5 pt 1 ps 0.5");
mplot.Plot(mfun,  "from ChFunction_Recorder",  " with lines lt -1 lc rgb'#00AAEE' ");
mplot.Plot(matr,   2,6, "from ChMatrix", " with lines lt 5");
~~~

You should see the following plot:

![](http://projectchrono.org/assets/manual/Tutorial_gnuplot5.png)



\include demo_POST_gnuplot.cpp

