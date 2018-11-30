The idea of a ChGd node is to have an inherited node/nodes that can be added
to the godot scene and have special functionality.

Example nodes:
-HUD that has information pulled from the ChSystem and can be updated each frame
-Camera sensor that can process data on each frame, and can make use of shaders
-Arguably mesh nodes that represent ChBodies should be nodes that have a pointer
    back to the ChBody and can then just update themselves each frame.
-interactive camera that can take mouse inputs and move the view around.
