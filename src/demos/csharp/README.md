Chrono C# demos
================

A few of the Chrono modules are SWIG-wrapped for use in C# programs.  
Currently, these are:
- the Chrono core module (no FEA support at this time)
- the Chrono::Irrlicht run-time visualization system
- the Chrono::Vehicle module

To generate the necessary C# scripts and associated libraries, set `CH_ENABLE_MODULE_CSHARP` to `ON` and also enable any other desired Chrono module (Irrlicht and/or Vehicle). The resulting C# scripts and associated libraries can then be used in external C# programs that leverage the exposed Chrono functionality, or else embedded in a Unity project.

When using MS Visual Studio, in order to build the C# demos included in the main Chrono solution, firstly build the core modules. When complete, attempt to build a C# demo to trigger Cmake GLOB update and a regeneration of all dependencies (i.e. the SIG generated .cs builds). Reload again and then proceed to build demos.

NB. If using the irrlicht module, ensure you manually copy the Irrlicht.dll to the build folder (or point to it in the system PATH), otherwise the demos will present an "unhandled exception error" stating an inability to load the dll 'Chrono_csharp_vehicle' for example. This is somewhat misleading, as it is not an error with Chrono - check you have correctly placed the Irrlicht (or any other required component dll) where the program can find it.