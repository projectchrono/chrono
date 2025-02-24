Chrono C# demos
================

A few of the Chrono modules are SWIG-wrapped for use in C# programs.  
Currently, these are:
- the Chrono core module (no FEA support at this time)
- the Chrono::Irrlicht run-time visualization system
- the Chrono::Vehicle module

To generate the necessary C# scripts and associated libraries, set `CH_ENABLE_MODULE_CSHARP` to `ON` and also enable any other desired Chrono module (Irrlicht and/or Vehicle). The resulting C# scripts and associated libraries can then be used in external C# programs that leverage the exposed Chrono functionality, or else embedded in a Unity project.
