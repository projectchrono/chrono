
Markers        {#markers}
============

Markers, which are implemented in Chrono in the ChMarker class, are auxiliary frames that can be 
attached to [rigid bodies](@ref rigid_bodies).

![](http://www.projectchrono.org/assets/manual/pic_ChMarker.png)

- Markers are [coordinate systems](@ref coordinate_transformations) that 
  inherit from ChFrameMoving because in the most general 
  case one can also assign a motion to a marker with respect to the owner body

- Markers can be used to get the position/velocity/acceleration of 
  a given reference frame attached to a [rigid body](@ref rigid_bodies)

- They can be used to build [mechanical constraints](@ref links) via ChLink objects
  by using a couple of ChMarker objects from two bodies

Example:

~~~{.cpp}
auto marker_c = chrono_types::make_shared<ChMarker>();

marker_c->Impose_Abs_Coord(X_ca); // or.. marker_c->Impose_Rel_Coord(X_cb);

body_b->AddMarker(marker_c);
~~~



