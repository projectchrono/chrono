Basic suspensions example (demo_MBS_suspension.cpp)  {#tutorial_demo_suspension}
==========================

Simulate a simplified vehicle with double-wishbone suspensions and drive it using the mouse over some sliders in the interface. 

This tutorial shows how to:

- use the ChLinkDistance to represent massless rods, saving computational efforts.
- change the friction coefficient depending on the contact zone (here, half of the ground plane has lower friction).
- use the Irrlicht interface system to interact with a model.
- use ChLinkSpring to make spring-dampers   
 
\include demo_MBS_suspension.cpp

