Chrono C# demos
================

A few of the Chrono modules are SWIG-wrapped for use in C# programs.  
Currently, these are:
- the Chrono core module (no FEA support at this time)
- the Chrono::Irrlicht run-time visualization system
- the Chrono::Vehicle module

To generate the necessary C# scripts and associated libraries, set `ENABLE_MODULE_CSHARP` to `ON` and also enable any other desired Chrono module (Irrlicht and/or Vehicle). The resulting C# scripts and associated libraries can then be used in external C# programs that leverage the exposed Chrono functionality, or else embedded in a Unity project.

The demos in this directory are meant for illustration only, as well as a mechanism for verifying correct operation of the Chrono::Csharp module.

To prevent mixing different languages (C++ and C#) in the same build, these demos can not be enabled, configured, and built simultaneously with the rest of the Chrono libraries, C++ demos, and tests.  Instead, we provide a separate CMakeLists.txt hierarchy to generate a C# CMake project that includes all C# demos in the various sub-directories.  These C# demos assume an existing Chrono installation (with the Chrono::Csharp module enabled) and are configured as external projects depending on Chrono.  This setup is very similar to the simpler example provided in `template_project_csharp`, except that the CMake scripts here account for multiple sub-projects. 

