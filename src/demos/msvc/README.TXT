-------------------------------------------------------------

   Examples of Microsoft Visual C++ 2005 Express projects
   using Chrono::Engine 

   HOW TO COMPILE THESE EXAMPLES
   
-------------------------------------------------------------

 - for each demo project (racing, oscillator, etc.), go into its
   directory and:
 
 - open the file   chrono_proj_config.vsprops
 
 - change the Value= field into the Macros section so that the 
   paths of ChronoEngineDir and IrrlichtDir correspond to the 
   directories where you installed them.
   For example, if you installed Irrlicht SDK in C:\, you set it as:
   
   ...
	<UserMacro
		Name="IrrlichtDir"
		Value="C:\irrlicht-1.7.3"
	/>
	
- now double click on the xxxxxx.sln file to open the Microsoft
  Visual C++ Express IDE and load the solution & project. 
  
- Note: when you recompile the example (ex. press key F7 in IDE), the 
  binaries (the .exe executable) will be put into the directory
    \ChronoEngine\bin\Win32_VisualStudio 
    
    