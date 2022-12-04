Slider crank example (demo_MBS_crank.cpp)  {#tutorial_demo_crank}
==========================

The simpliest way to integrate Chrono::Engine in the Irrlicht 3D visualization library: in fact the coordinates of the joints are simply used as end-points of simple polygonal lines which are drawn in the 3D space for each frame redraw, to show a very simplified 'skeleton' of a slider-crank mechanism. The demo also teaches how to:

- create constraints and 'engine' objects

- make a real-time application, where 
  Chrono::Engine adapts the integration step to the speed of the CPU. 


\include demo_MBS_crank.cpp

