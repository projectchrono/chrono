
Coordinate Systems        {#coordinate_systems}
==============================

Translation and rotations, together with their transformations, are key elements in the Chrono library.

While translations in the 3D space are _exclusively_ expressed as [vectors](#manual_ChVector3), the user might encounter either [quaternions](#manual_ChQuaternion) or, more rarely, [rotation matrices](#manual_ChMatrix33) to represent rotations (usually for performance reasons).  
The recommendation is to **rely on quaternions** wherever possible and to leverage the appropriate conversions to rotations matrices if required.

Chrono relies on different objects to represents coordinate systems ([ChCoordsys](#manual_ChCoordsys), [ChFrame](#manual_ChFrame), [ChFrameMoving](#manual_ChFrameMoving), @ref chrono::ChMarker "ChMarker", ...) in order to leverage different features and optimizations.  
However, the user is usually requested to deal with [ChCoordsys](#manual_ChCoordsys) or [ChFrame](#manual_ChFrame) only, on which we recommend the beginner user to focus on.


Some basic knowledge of the overall structure might help to better handle these objects:
- [ChCoordsys](#manual_ChCoordsys) and [ChFrame](#manual_ChFrame) are the most important base classes;
- [ChCoordsys](#manual_ChCoordsys) is the most basic object, consisting in just a pair of `ChVector3` plus a `ChQuaternion`;
- [ChFrame](#manual_ChFrame) is the most used throughout Chrono; it embeds a `ChCoordsys` and just adds a rotation matrix to improve the performance;
- both [ChCoordsys](#manual_ChCoordsys) and [ChFrame](#manual_ChFrame) easily allow all kind of transformations and change of coordinates at the position level (translation and rotation);
- [ChFrameMoving](#manual_ChFrameMoving) adds transformations at the velocity and acceleration levels;
- @ref chrono::ChMarker "ChMarker" offers similar capabilities to `ChFrameMoving` but it is attached to a body and follows it during motion;


All the classes discussed in this page are templated in order to hold different scalar types. However, in order to have a cleaner code, some specialization for `double` and `float` and sometimes `int` types are offered. Their names follow the pattern `ClassName[d|f|i]`.

\tableofcontents


# Vectors  {#manual_ChVector3}


Vectors that represent 3D points in space are defined by means of the @ref chrono::ChVector3<> "ChVector3" class. 

In mathematical notation:

\f[ 
\mathbf{p}=\{p_x, p_y, p_z\}
\f]


### Construction

The general construction of @ref chrono::ChVector3<> "ChVector3"s is through its constructor

~~~{.cpp}
ChVector3d vect1(1, 2, 3); // equivalent to: ChVector3<double> vect1(1, 2, 3);
ChVector3d vect1(VECT_X); // equivalent to: ChVector3<double> vect1 = VECT_X;
~~~

However, Chrono offers also a useful set of constant (double) vectors - `VNULL`, `VECT_X`, `VECT_Y`, `VECT_Z` - that represent the null and the axes unit vectors, respectively.

### Usage

`ChVector3`s hold three coefficients, accessible through @ref chrono::ChVector3<>::x(), `y()` or `z()`.

Since `ChVector3` does not inherit from Eigen vectors it is required to apply a conversion through @ref chrono::ChVector3<>::eigen() ".eigen()" to leverage Eigen capabilities.

`ChVector`s can be modified and used through:
- their methods, as defined in the class interface; including all the overloaded operators `+`, `-`, `*`, etc.
- free functions, starting with the `V` uppercase letter, `Vcross`



# Quaternions  {#manual_ChQuaternion}

Rotations in Chrono are mainly described by quaternions, as implemented in the @ref chrono::ChQuaternion<> "ChQuaternion" class, according to the axis-angle pair representation:

\f[
\mathbf{q}=\left\{
\begin{array}{c}
q_0\\
q_1\\
q_2\\
q_3
\end{array}
\right\}=\left\{
\begin{array}{c}
\cos(\theta / 2)\\
{u}_x \sin(\theta / 2)\\
{u}_y \sin(\theta / 2)\\
{u}_z \sin(\theta / 2)
\end{array}
\right\}
\f]

in which should be noted that the scalar part is at the first position.

Given the template nature of the class, it is required to specify the scalar type; however, useful predefined specializations are offered, e.g. `ChQuaterniond` for the double type.

![](http://www.projectchrono.org/assets/manual/coord_quaternions.png)

Please remember that:
- only unit-norm quaternions represent valid rotations;
- the neutral quaternion @ref chrono::QUNIT "QUNIT", representing no rotation, is defined by quadruplet `(1, 0, 0, 0)`.

### Construction

Quaternions can be built by explicitly providing their components in the constructor, e.g.
~~~{.cpp}
ChQuaterniond q(1, 0, 0, 0);
~~~

but in many cases it is convenient to build from:
- `QuatFrom___` free functions, defined in [ChRotation.h](https://github.com/projectchrono/chrono/blob/main/src/chrono/core/ChRotation.h).  
  e.g. @ref chrono::QuatFromAngleAxis "QuatFromAngleAxis" or @ref chrono::QuatFromAngleY "QuatFromAngleY"
- predefined quaternion, defined in [ChQuaternion.h](https://github.com/projectchrono/chrono/blob/main/src/chrono/core/ChQuaternion.h).  
  e.g. `Q_ROTATE_Z_TO_X` or @ref chrono::QUNIT "QUNIT"


### Usage

The `ChQuaternion` class allows to apply rotations on other items, being them @ref chrono::ChVector3<> "ChVector3", @ref chrono::ChFrame "ChFrame" or other `ChQuaternion`s as well.

The rotation of a point by a quaternion can be easily done by using the `.Rotate()` member function. The example below shows how to rotate a point by 20 degrees about the Y axis:

~~~{.cpp}
ChVector3d vA(1, 2, 3);
ChQuaterniond q = QuatFromAngleY(20 * CH_DEG_TO_RAD);
ChVector3d vB = q.Rotate(vA);
~~~

The `*` operator is used to perform the quaternion (Hamilton) product. From a kinematic perspective, this represents concatenation of rotations. For example, a rotation **qA** followed by a rotation **qB** can be condensed in a single rotation **qC** by pre-multiplication, as follows:

~~~{.cpp}
qC = qB * qA; // concatenate two rotations, first qA then qB
~~~

Alternatively, for convenience, the `>>` operator can be used to achieve the same result, but writing items from left to right:

~~~{.cpp}
qC = qA >> qB; // concatenate two rotations, first qA then qB (same result as before)
~~~


# Rotation matrices  {#manual_ChMatrix33}

As an alternative to quaternions, Chrono offers the use of 3x3 rotation matrices \f$ \mathbf{R} \in \mathsf{SO}(3) \f$ to describe the orientation of a reference frame with respect to another (consult @ref chrono::ChMatrix33 for specific information). Note that a rotation matrix is orthonormal, therefore \f$ \mathbf{R}^{-1} = \mathbf{R}^{T} \f$.

Several methods are available for the creation of a rotation matrix and its conversion to other representations. The example below illustrates some construction options:

~~~{.cpp}
ChMatrix33d rotmA(1); // build from value on diagonal (e.g. identity matrix)
ChMatrix33d rotmB(quat); // build from quaternion
ChMatrix33d rotmC(angle, axis); // build from angle and axis
ChMatrix33d rotmD(vecX, vecY, vecZ); // build from column vectors
~~~

The `*` operator can be used to multiply two rotation matrices. For instance, a rotation **rotmA** followed by a rotation **rotmB** is given by:

~~~{.cpp}
ChMatrix33d rotmC = rotmB * rotmA; // concatenate two rotations, first rotmA then rotmB
~~~

The `*` operator can also be used to multiply a ChVector3d; this corresponds to rotating the vector:

~~~{.cpp}
ChVector3d vB = rotm * vA;
~~~


# ChCoordsys   {#manual_ChCoordsys}

A @ref chrono::ChCoordsys represents a coordinate system in 3D space. It embeds both a vector (coordinate translation \f$ \mathbf{d} \f$ ) and a quaternion (coordinate rotation \f$ \mathbf{q} \f$ ):  

\f[
\mathbf{c}=\{\mathbf{d},\mathbf{q}\}
\f]

The ChCoordys is a lightweight version of a ChFrame, which is discussed next.


# ChFrame   {#manual_ChFrame}

A @ref chrono::ChFrame represents a coordinate system in 3D space like ChCoordsys, but it includes more advanced features.

![](http://www.projectchrono.org/assets/manual/coord_frame.png)

As shown in the picture above, a ChFrame object indicates how "rotated" and "displaced" a coordinate system **b** is with respect to another coordinate system **a**. Many time **a** is the absolute reference frame.

<div class="ce-info">
In what follows the notation
\f[
\mathbf{d}_{a,b(c)}
\f]
is used to define a vector \f$ \mathbf{d} \f$ ending in point \f$ a \f$, starting from point \f$ b \f$, 
expressed in the base \f$ c \f$ (that is, _measured_ along the x,y,z axes of the 
coordinate system \f$ c \f$ ).
If \f$ b \f$ is omitted, it is assumed to be the origin of the absolute reference frame.
</div>

As a ChCoordsys, a ChFrame object has a vector for the translation and a quaternion for the rotation:

\f[
\mathbf{c}=\{\mathbf{d},\mathbf{q}\}
\f]

However, the ChFrame class also stores an auxiliary 3x3 rotation matrix that can be used to speed up computations in the cases where quaternions would be less efficient. In this sense, ChCoordsys can be considered as a lightweight version of ChFrame, that may save memory if advanced features are not needed.

## Construction

There are several ways to build a ChFrame; for instance:

~~~{.cpp}
ChFramed Xa; // build default frame: zero translation, no rotation
ChFramed Xb(vec, quat); // build from given translation vector and rotation quaternion
ChFramed Xc(csys); // build from given ChCoordysd
ChFramed Xd(vec, theta, u); // build from translation vector and rotation about axis
~~~

## Usage

One of the most important features of the ChFrame class is the ability to apply coordinate transformations to:
- *directions*: the transformation is applied to a vector describing a _direction_:
  the vector is only rotated but no offset is applied due to the different location of the origin of the new reference system;
  \f[
	\mathbf{d}_{P,a(a)}=\mathbf{R}_{ba} \mathbf{d}_{P,b(b)}
  \f]
  The related methods are @ref chrono::ChFrame<>::TransformDirectionLocalToParent() "ChFrame<>::TransformDirectionLocalToParent()" and @ref chrono::ChFrame<>::TransformDirectionParentToLocal() "ChFrame<>::TransformDirectionParentToLocal()";
- *locations*: the transformation is applied to a vector describing a _location_:
  when transformed, the vector is rotated and the different location of the new reference system is taken into consideration
  \f[
	\mathbf{d}_{P,a(a)}=\mathbf{d}_{b,a(a)} + \mathbf{R}_{ba} \mathbf{d}_{P,b(b)}
  \f]
The related functions are @ref chrono::ChFrame<>::TransformPointLocalToParent() "ChFrame<>::TransformPointLocalToParent()" and @ref chrono::ChFrame<>::TransformPointParentToLocal() "ChFrame<>::TransformPointParentToLocal()";
- *wrenches* transformation: the transformation is applied to a wrench i.e. a pair of a force and a torque:
  a force expressed in a different reference system lead to an additional torque component, simply due to the change of frame.  
  The related methods are @ref chrono::ChFrame<>::TransformWrenchLocalToParent() "ChFrame<>::TransformWrenchLocalToParent()" and @ref chrono::ChFrame<>::TransformWrenchParentToLocal() "ChFrame<>::TransformWrenchParentToLocal()".

Direction and location transformations are by far the most used features thus being worth further details. As example, we can consider the transformation of a point expressed in a local coordinate system **b** into another (e.g. absolute) coordinate system **a**:

![](http://www.projectchrono.org/assets/manual/coord_trasf1_point.png)

This _affine_ transformation can be written as:

\f[
\mathbf{d}_{P,a(a)}=\mathbf{d}_{b,a(a)} + \mathbf{R}_{ba} \mathbf{d}_{P,b(b)}
\f]

In Chrono, this process can be expressed, without using `ChFrame<>`s as:

~~~{.cpp}
ChVector3d d_Pa_a; // vector with endpoint P, starting from point a, expressed in frame (a)
ChVector3d d_ba_a; // vector with endpoint b, starting from point a, expressed in frame (a)
ChVector3d d_Pb_b; // vector with endpoint P, starting from point b, expressed in frame (b)
ChMatrix33d R_ba; // rotation of frame (b) expressed with respect to frame (a)
...
d_Pa_a = d_ba_a + R_ba * d_Pb_b;
~~~

On the contrary, by using `ChFrame<>` methods, it is possible to simplify the notation, by leveraging one of these three similar approaches:
- using `Transform[Direction|Point]ParentToLocal`|`Transform[Direction|Point]LocalToParent`;
  more verbose, standard approach
- using the `*` operator;
  ~~~{.cpp}
  ChVector3d d_Pa_a; // vector with endpoint P, starting from point a, expressed in frame (a)
  ChVector3d d_Pb_b; // vector with endpoint P, starting from point b, expressed in frame (b)
  ChFramed X_ba; // frame (b) (i.e. position and rotation) expressed with respect to frame (a)
  ...
  d_Pa_a = X_ba * d_Pb_b;
  ~~~
- using the `>>` operator;

~~~{.cpp}
ChVector3d d_Pa_a; // vector with endpoint P, starting from point a, expressed in frame (a)
ChVector3d d_Pb_b; // vector with endpoint P, starting from point b, expressed in frame (b)
ChFramed X_ba; // frame (b) (i.e. position and rotation) expressed with respect to frame (a)
...
d_Pa_a = X_ba * d_Pb_b;
~~~

The same concept can be used to chain coordinate transformations. For instance, if the transformation from frames **c** to **b** and from **b** to **a** are known,
the overall frame rotation and displacement can be obtained as

![](http://www.projectchrono.org/assets/manual/coord_trasf3_frame.png)


~~~{.cpp}
ChFramed X_ba, X_cb, X_ca;
...
X_ca = X_ba * X_cb;
~~~

or, equivalently, by making use of the `>>` operator:

~~~{.cpp}
X_ca = X_cb >> X_ba;
~~~

In Chrono, most of the transformation between ChCoordsys<>, ChFrame<> and ChFrameMoving<> (which is defined below) can be expressed in two equivalent ways: <br>
  - using the * operator _RIGHT TO LEFT_ transformations, as in: 
    ```X_ca = X_ba * X_cb``` <br>
  - using the >> operator _LEFT TO RIGHT_ transformations, as in:
    ```X_ca = X_cb >> X_ba```  <br>

The latter has some advantages: <br>
- it is more 'intuitive' (see how the subscripts cb-ba follow a 'chain') <br>
- it leverages the default compiler operation precedence rules (from left to right) to improve computational performance <br>
For example if the first operand is a vector, as in  ```vnew = v >> X_dc >> X_cb >> X_ba```,
the default behavior of the compiler leads to a sequence of matrix-by-ChFrame operations returning temporary vectors. On the contrary, the * operator would create several `ChFrame<>` temporary objects, which would be slower.

One can also use the * or >> operators with other objects, 
for example using >> between a ChFrame Xa and a ChVector3 vb. The outcome of this 
operation is a new ChFrame object obtained by translating the old one by a vector vb:

~~~{.cpp}
ChVector3d vb;
ChFramed Xa, Xt; ...
Xt = Xa >> vb;    // also  Xt = vb * Xa;
~~~

The same holds for in-place operators *= or >>=, which can be used 
to translate or to rotate a ChFrame, or to transform entirely, 
depending on how it is used: with a ChVector3 or a ChQuaternion or another ChFrame. 
For example, to translate Xa by vector vb one writes:

~~~{.cpp}
Xa >>= vb;    
~~~

Note that while the operators * and >>  create temporary objects, 
that is not the case with *= or >>=, which leads to improved efficiency. In this context, a rule of thumb is to avoid:
 - Operators * and >> 
 - Use of low-level functions such as TransformLocalToParent, TransformParentToLocal, etc.


Both the * and >> operations support an inverse transformation. 
Example: in the relation 
X_ca = X_cb >> X_ba; 
supposed that X_ca and X_cb are known and one is interested in computing X_ba. 
Pre-multiplying both sides of the equation by the inverse of X_cb yields 

~~~{.cpp}
X_ba = X_cb.GetInverse() >> X_ca;    
~~~

Note that the GetInverse operation might be less efficient than 
the less readable low-level methods, in this case TransformParentToLocal(), that is:

~~~{.cpp}
X_ba = X_cb.TransformParentToLocal(X_ca);    
~~~

See @ref chrono::ChFrame for API details.


# ChFrameMoving    {#manual_ChFrameMoving}

The @ref chrono::ChFrameMoving object represents a coordinate system in 3D space, like a 
ChFrame, but it stores also information about the velocity and accelerations of the frame:

\f[
\mathbf{c}=\{\mathbf{p},\mathbf{q},\dot{\mathbf{p}}, \dot{\mathbf{q}}, \ddot{\mathbf{p}}, \ddot{\mathbf{q}} \}
\f]

Note that using quaternion derivatives to express angular velocity and angular 
acceleration can be cumbersome. Therefore, this class also has the possibility of 
setting and getting such data in terms of angular velocity  
\f$ \mathbf{\omega} \f$ 
and of angular acceleration 
\f$ \mathbf{\alpha} \f$ :

\f[
\mathbf{c}=\{\mathbf{p},\mathbf{q},\dot{\mathbf{p}}, \mathbf{\omega}, \ddot{\mathbf{p}}, \mathbf{\alpha}\}
\f]

This can be shown intuitively with the following picture:

![](http://www.projectchrono.org/assets/manual/coord_framemoving.png)

Note that 'angular' velocities and accelerations can be set/get both in the 
basis of the moving frame or in the absolute frame.

Example: A ChFrameMoving is created and assigned a non-zero angular velocity
and linear velocity, also linear and angular accelerations are assigned.


~~~{.cpp}
ChFrameMoving<> X_ba;
X_ba.SetPos(ChVector3d(2,3,5));
X_ba.SetRot(myquaternion);

// set velocity 
X_ba.SetPos_dt(ChVector3d(100,20,53)); 
X_ba.SetWvel_loc(ChVector3d(0,40,0)); // W in local frame, or..
X_ba.SetWvel_par(ChVector3d(0,40,0)); // W in parent frame

// set acceleration
X_ba.SetPos_dtdt(ChVector3d(13,16,22)); 
X_ba.SetWacc_loc(ChVector3d(80,50,0)); // a in local frame, or..
X_ba.SetWacc_par(ChVector3d(80,50,0)); // a in parent frame
~~~

A ChFrameMoving can be used to transform ChVector3 (points in space), a ChFrame, or ChFrameMoving objects. Velocities are also computed and transformed.

Example: The absolute velocity and angular velocity of **c** with respect to **a** can be computed if the transformation from **b** to **a** and from **c** to **b** is known:


![](http://www.projectchrono.org/assets/manual/coord_trasf5_framemoving.png)

The case above translates in the following equivalent expressions, 
using the two alternative formalisms (Left-to-right based on >> operator, 
or right-to-left based on * operator):

~~~{.cpp}
X_ca = X_cb >> X_ba;
X_ca = X_ba * X_cb;
~~~

This is _exactly the same algebra_ used in ChFrame and ChCoordsys, 
except that this time also the velocities and accelerations are also transformed.

Note that the transformation automatically takes into account the 
contributions of complex terms such as centripetal accelerations, 
relative accelerations, Coriolis acceleration, etc. 

The following is another example with a longer concatenation of transformations:

![](http://www.projectchrono.org/assets/manual/coord_trasf6_framemoving.png)

Note that one can also use the inverse of frame transformations, 
using GetInverse(), as seen for ChFrame.

Example: Computing the position, velocity and acceleration of the moving target 8
with respect to the gripper 6, expressed in the basis of the frame 6.


![](http://www.projectchrono.org/assets/manual/coord_robotexample.png)

How would one compute X_86 knowing all others? 
Start from two equivalent expressions of X_80: 

	X_86>>X_65>>X_54>>X_43>>X_32>>X_21>>X_10 = X_87>>X_70; 

also:

	X_86>>(X_65>>X_54>>X_43>>X_32>>X_21>>X_10) = X_87>>X_70;

Post multiply both sides by inverse of (...), remember that in general

- X >> X.GetInverse() = I
- X.GetInverse() >> X = I,

where I is the identity transformation that can be removed, to finally get:

~~~{.cpp}
X_86 = X_87 >> X_70 >> (X_65 >> X_54 >> X_43 >> X_32 >> X_21 >> X_10).GetInverse();
~~~

Example, based on the same figure:
Computing velocity and acceleration of the gripper with respect to the moving target 8,
expressed in the basis of reference frame 8.


~~~{.cpp}
X_68 = X_12 >> X_23 >> X_34 >> X_45 >> X_56 >> (X_87 >> X_70).GetInverse();
~~~

See @ref chrono::ChFrameMoving for API details.


# ChMarker    {#manual_ChMarker}

ChMarker objects are auxiliary frames that have the specific feature of following the [rigid body](@ref rigid_bodies) to which they are attached while potentially moving with respect to it.

![](http://www.projectchrono.org/assets/manual/pic_ChMarker.png)

ChMarker are used most often in the @ref chrono::ChLinkLock "ChLinkLock" link family.



# Theory

Additional details on the theoretical aspects of coordinate transformations in Chrono:
- [PDF whitepaper on rotations](http://projectchrono.org/assets/white_papers/rotations.pdf)
- [PDF whitepaper on coordinates](http://projectchrono.org/assets/white_papers/frame_kinematics.pdf)



