Install the MATLAB module {#module_matlab_installation}
==========================

[TOC]

Chrono::Matlab is a simple module that allows exchanging matrices and variables from/to the Matlab environment with simple C++ function calls in Chrono.

## Features

The **Matlab module** is used to provide an easy way to call Matlab 
functions from your Chrono -based application. Basically, you can 

- call Matlab commands from your C++ program,
- send/retrieve data to/from Matlab (the Chrono C++ matrices are converted to Matlab, and vice-versa)
- use the Matlab powerful visualization tools, to show simulation data in 2D/3D plots, etc.


## Requirements

- To **run** applications based on this module:
	- you must have a licensed copy of Matlab(TM) installed on your system. 
	  We tested it from the 2006 revision up to the most recent.
	- your PATH environment variable must contain the path to the libeng.dll (see below)
- To **build** applications based on this module:
	- you must have a licensed copy of Matlab(TM) installed on your system. 
	  We tested it from the 2006 revision up to the most recent.



## Building instructions

1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono).
  
2. Set `CH_ENABLE_MODULE_MATLAB` to 'on'.

3. Set `Matlab_ROOT_DIR` to the Matlab installation root directory.
   This changes depending on where you installed Matlab. For example, it could be `C:/Program Files/MATLAB/R2019a`
 
4. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

<div class="ce-warning">
In most cases, when you run an executable that uses the Matlab module, you will get an 
error message from Windows, telling that the libeng.dll library cannot be found. 
This means that you must tell Windows the path where to 
find it; so you need to modify the PATH system variable. Do the following:
<br><br>
1.Open your Matlab editor, type <br>
  <tt>[matlabroot "\bin\win64"]</tt><br>
  (or <tt>[matlabroot "\bin\win32"]</tt> on a 32 bit platform).
  You get the path to the dll on your system.
<br><br>
2.To set an environment variable in a Windows system, select  
  Start > Settings > Control Panel > System.  
  The System Properties dialog box is displayed. Click the Advanced tab, 
  and then the Environment Variables button. In the System variables panel 
  scroll down until you find the Path variable. 
  Click this variable to highlight it, and then click the Edit button to
  open the Edit System Variable dialog box. At the end of the path string, 
  enter a semicolon and then the path string returned by evaluating the expression shown above in MATLAB. 
  Click OK in the Edit System Variable dialog box, and in all remaining dialog boxes.
</div>

<div class="ce-info">
In some cases, in Windows, the Matlab engine cannot be started 
because it was not registered in COM during its installation. 
In this case, when launching the "demo_MTLB_matlab.exe", 
you get a message like "Can't start MATLAB engine". 
To fix this problem: <br><br>
1.Open your Matlab editor,  <br>
2.type `!matlab -regserver`
</div>


## How to use it

- Look at the [API section](group__matlab__module.html) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_table_of_content_chrono_matlab) to learn how to use the functions of this module.
