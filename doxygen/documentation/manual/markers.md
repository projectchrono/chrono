
Markers        {#markers}
============

Markers are auxiliary frames that can be 
attached to [rigid bodies](@ref rigid_bodies).

This is achieved by using the ChMarker class. 

![](pic_ChMarker.png)

- Markers are [coordinate systems](@ref coordinate_transformations) that 
  inherit the features of ChFrameMoving because in the most general 
  case one can also assign motion respect to the owner body

- They can be used to get position/speed/acceleration of 
  a given reference attached to a [rigid body](@ref rigid_bodies)

- They can be used to build [mechanical constraints](@manual_ChLink) via ChLink objects
  (using couple of ChMarker from two bodies)

Example:

~~~{.cpp}
ChSharedPtr<ChMarker> marker_c(new ChMarker);

marker_c->Impose_Abs_Coord(X_ca); // or.. marker_c->Impose_Rel_Coord(X_cb);

body_b->AddMarker(marker_c);
~~~


