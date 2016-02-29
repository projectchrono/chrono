Creating A System In Chrono (demo_buildsystem.cpp) {#demo_buildsystem}
==========================

\section ex1 Example 1

\dontinclude demo_buildsystem.cpp

Start with creating the physical system: 

\skipline my_system

Create a bunch of rigid bodies..

Note that we use shared pointers, so you don't have to care about the deletion (never use delete.. for objects managed with shared pointers! it will be automatic!) 

\skip ChSharedBodyPtr
\until my_body_C

_Note_
The ChSharedBodyPtr is a shortcut for ChSharedPtr<ChBody>, as well as there are other shortcuts like ChSharedMarkerPtr for ChSharedPtr<ChMarker> etc. Use the second form if you prefer. 

Create some markers..

Markers are 'auxiliary coordinate systems' to be added to rigid bodies. Again, note that they are managed by shared pointers. 

\skip ChSharedMarkerPtr
\until my_marker_b2

You can create some forces too... 