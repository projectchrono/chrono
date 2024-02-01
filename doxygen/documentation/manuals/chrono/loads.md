
Loads {#loads}
========

Chrono offers different options to apply loads to objects.

As the name suggests, objects that can carry a load are those inheriting from the @ref chrono::ChLoadable "ChLoadable" class; for example @ref chrono::ChBody "ChBody", finite elements and nodes or @ref chrono::ChShaft "ChShaft"s.

Loads can be applied to these objects through different approaches:
+ @ref chrono::ChForce "ChForce":
  - available only for @ref chrono::ChBody "ChBody"
  - applies either a force or a torque with respect to some pre-defined frame (body or world);
+ @ref chrono::ChLoad<Tloader> "ChLoad<Tloader>" + @ref chrono::ChLoader "ChLoader":
  - available for any @ref chrono::ChLoadable "ChLoadable" inherited object;
  - the specific implementation must be provided through a @ref chrono::ChLoader "ChLoader" object;
  - it is more tightly coupled with the Chrono system;
  - requires the computation of the generalized loads;
+ loads inheriting from @ref chrono::ChLoadCustom "ChLoadCustom"|@ref chrono::ChLoadCustomMultiple "ChLoadCustomMultiple":
  - similar to the previous one, but they extended by inheritance rather than template;
  - they are applied to either one or multiple @ref chrono::ChLoadable "ChLoadable" objects;
  - it is the preferred choice for loads applied to pair of objects;
  - multiple pre-defined classes are available, simplifying the writing of the generalized loads.

While not being considered explicitely in the previous list, also the @ref chrono::ChLinkTSDA "ChLinkTSDA"|@ref chrono::ChLinkRSDA "ChLinkRSDA" can do similar jobs, also including the Jacobian for the generalized forces or also some advanced internal dynamic, by providing custom functors (@ref chrono::ChLinkTSDA::ForceFunctor "ChLinkTSDA::ForceFunctor"|@ref chrono::ChLinkRSDA::TorqueFunctor "ChLinkRSDA::TorqueFunctor") or even proper ODEs (only @ref chrono::ChLinkTSDA::ODE "ChLinkTSDA::ODE").

Other simplified approaches, limited to _ChBody_, allow to accumulate forces, by using @ref chrono::ChBody::Accumulate_force() "Accumulate_force()" and @ref chrono::ChBody::Accumulate_torque() "Accumulate_torque()".

The overall contributions of forces to a given _ChBody_ can be retrieved through @ref chrono::ChBody::GetAppliedForce() "GetAppliedForce()" and similarly for torques.

For FEA nodes, similary to the @ref chrono::ChForce "ChForce" for @ref chrono::ChBody "ChBody", it is possible to add a force directly to the node through @ref chrono::fea::ChNodeFEAxyz::SetForce() "ChNodeFEAxyz::SetForce()". However, in this case the options are even more limited, since the force is expressed as a simple @ref chrono::ChVector "ChVector", thus always assumed constant and expressed in absolute frame. The @ref chrono::fea::ChNodeFEAxyzrot "ChNodeFEAxyzrot" class implements also @ref chrono::fea::ChNodeFEAxyzrot::SetTorque() "ChNodeFEAxyzrot::SetTorque()".

Some more peculiar class has been excluded from this list: please look at @ref chrono::ChLoadBase "ChLoadBase" to have a full perspective on the load classes in Chrono.


### ChForce

The @ref chrono::ChForce "ChForce" can be applied directly to a @ref chrono::ChBody "ChBody" by calling:
~~~{.cpp}
  auto force = chrono_types::make_shared<ChForce>();
  body->AddForce(force)

  // AFTER calling AddForce
  force->SetMforce(10);
~~~

Please mind that:
- use `body->AddForce(force)` and not `force->SetBody(body)`: the latter it is not sufficient since the _ChForce_ wouldn't be considered by the _ChSystem_;
- always call `ChForce` methods **after** having called `body->AddForce(force)`

The application point, direction, position and modulus can be set either through constant values or through [ChFunctions](@ref ChFunction_objects). The reference system can be either relative to the body or absolute, but cannot be set to a generic frame.


### ChLoad and inherited

These sets of loads allows for the maximum freedom and coupling with the Chrono system.

Contrary to @ref chrono::ChForce "ChForce", these other _ChLoad_s requires the introduction of a @ref chrono::ChLoadContainer "ChLoadContainer" in order to be added to the system. For example:

~~~{.cpp}
  auto load_container = chrono_types::make_shared<ChLoadContainer>();
  sys.Add(load_container);

  auto load_bb = chrono_types::make_shared<ChLoadBodyBodyTorque>(bodyA, bodyB, ChVector<>(0,10.0,0), false);
  load_container->Add(load_bb);
~~~

For the case of the @ref chrono::ChLoad<Tloader> "ChLoad<Tloader>", the user is asked to either provide one of the pre-defined @ref chrono::ChLoader "ChLoader" objects or to write its own. Please refer to the documentation of each single _ChLoader_ to understand their use. This method considers the load applied to a single object.

A similar set of loads includes those inheriting from @ref chrono::ChLoadCustom "ChLoadCustom"|@ref chrono::ChLoadCustomMultiple "ChLoadCustomMultiple": while the features are similar compared to @ref chrono::ChLoad<Tloader> "ChLoad<Tloader>" type and also their usage might have big overlaps, they usually offere a wider set of pre-defined classes that might match the user needs.

These more advanced approaches allow for a tighter coupling with the Chrono system, allowing to introduce also entire stiffness matrix blocks (see @ref chrono::ChLoadBodyBodyBushingGeneric "ChLoadBodyBodyBushingGeneric" and others), providing Jacobians and much more. This come with the added price of having to implement some additional code.