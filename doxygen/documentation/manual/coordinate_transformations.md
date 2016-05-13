
Coordinate transformations        {#coordinate_transformations}
==============================


In the following we introduce very important objects that are used through the entire Chrono::Engine API for managing coordinates, points, transformations and rotations.


# Vectors  {#manual_ChVector}

Vectors that represent 3D points in space, are defined by means of the 
@ref chrono::ChVector class. 
With mathematical notation:

\f[ 
\mathbf{p}=\{p_x,p_y,p_z\}
\f]

The ChVector<> class is templated, so you can have vectors with single 
precision, as ChVector<float>, or with double precision,
as ChVector<double>, etc. 
Anyway, if you leave the template void, as ChVector<>, 
by default the double floating point precision is used. 

For example, create a vector:

~~~{.cpp}
ChVector<> mvect1(2,3,4);
~~~

One can add, subtract, multiply vectors, using +,-,* operators. 

~~~{.cpp}
ChVector<double> mvect1(2,3,4);	/// create a vector with given x,y,z ‘double’ components

ChVector<float> mvect2(4,1,2); 	/// create a vector with given x,y,z ‘float’ components

ChVector<> mvect3(); 		/// create a 0,0,0, vector. The <> defaults to ‘double’ 

ChVector<> mvect4(mvect1 + mvect2);	/// create a vector by copying another (a result from +) 

mvect3 = mvect1 + mvect2; 	/// vector operators: +, - 

mvect3 += mvect1;		/// in-place operators

mvect3 = mvect2 * 0.003; 	/// vector product by scalar

mvect3.Normalize();		/// many member functions… 

mvect3 = mvect1 % mvect2;  	/// Operator for cross product: A%B means vector cross-product AxB

double val = mvect1 ^ mvect2;  	/// Operator for inner product (scalar product)
~~~


# Quaternions  {#manual_ChQuaternion}

Quaternions are used to represent rotations in 3D space, 
and are introduced by the @ref chrono::ChQuaternion class. 
Quaternions are four-dimensional complex numbers; with mathematical notation:

\f[
\mathbf{q}=\{q_0,q_1,q_2,q_3\}
\f]


The quaternion algebra has interesting properties when it 
comes to our applications to coordinate transformations:

- a real unit quaternion \f$ \mathbf{q}=\{1,0,0,0\} \f$ represents no rotation
- given a rotation  \f$ \theta_{u} \f$ about a generic unit vector \f$ \mathbf{u} \f$, 
the corresponding quaternion is 

\f[
\mathbf{q}=\left\{
\begin{array}{c}
\cos(\alpha_u / 2)\\
{u}_x \sin(\theta_u / 2)\\
{u}_y \sin(\theta_u / 2)\\
{u}_z \sin(\theta_u / 2)
\end{array}
\right\}
\f]

![](coord_quaternions.png)

Only quaternions with unit norm represent rotations. 
One can build unit-norm quaternions in different ways. 
For example the following are equivalent ways 
to build a quaternion that represent no rotation:

~~~{.cpp}
ChQuaternion<> qA(1,0,0,0);
ChQuaternion<> qB(QUNIT);
ChQuaternion<> qC = QUNIT;
~~~

The quaternion as a function of a rotation  
\f$ \theta_{u} \f$ about a generic unit vector \f$ \mathbf{u} \f$ can be build with:

~~~{.cpp}
ChQuaternion<> qA = Q_from_AngAxis(theta, u);
~~~

The rotation of a point by a quaternion can be done easily 
by using the .Rotate() member function of quaternions. 

Example: say you want to rotate a point 20 degrees about the Y axis:

~~~{.cpp}
ChVector vA(1,2,3);
ChVector vB();
ChQuaternion qA = Q_from_AngAxis(20 * CH_C_DEG_TO_RAD, VECT_Y);
vB = qA.Rotate(vA);
~~~

The * operator is used to perform quaternion product between 
quaternions. 
From a kinematical point of view, this represent a 
concatenation of rotations. 
Example, a rotation qc followed by a rotation qb can be 
shortened in a single qa, if qa qb qc are quaternions, as follows:

~~~{.cpp}
qa = qb * qc;	   /// concatenate two rotations, first qc, followed by qb
qa.Rotate(mvect1); 
~~~

Alternatively, one can use the  >>  operator that concatenates 
in a specular order, from left to right:

~~~{.cpp}
qa = qc >> qb;	   /// concatenate two rotations, first qc, followed by qb  
qa.Rotate(mvect1); /// (same result as before!)
~~~


# Rotation matrices  {#manual_ChMatrix33}

As an alternative to quaternions, also a 
@ref chrono::ChMatrix33
can represent rotations \f$ \mathbf{A} \f$ in 3D space.

They have some interesting properties:

- rotation matrices are orthogonal, \f$ \mathbf{A} \in \mathsf{SO}(3) \f$, 
  so \f$ \mathbf{A}^{-1} = \mathbf{A}^{t} \f$
- there are Chrono::Engine functions that allow 
  to convert a quaternion into a 3x3 matrix 
  \f$ \mathbf{q} \mapsto \mathbf{A} \f$ and viceversa \f$ \mathbf{A} \mapsto \mathbf{q} \f$

A rotation matrix can be created in many ways:

~~~{.cpp}
ChMatrix33<> mA;     // default zero matrix, this is NOT a rotation matrix
ChMatrix33<> mB(1);  // unit matrix, this is a rotation matrix, meaning no rotation
ChMatrix33<> mC(quat); // build a rotation matrix from a given quaternion
~~~

The * operator can multiply two rotation matrices, 
for instance a rotation mA followed by a rotation mB becomes:

~~~{.cpp}
ChMatrix33<> mC = mB * mA;
~~~

The * operator can multiply a ChVector<>,
this corresponds to rotating the vector:

~~~{.cpp}
ChVector<> vB = m * vA;
~~~

The inverse rotation is the inverse of the matrix, 
that is also the transpose.


# ChCoordsys   {#manual_ChCoordsys}

A @ref chrono::ChCoordsys represents a coordinate system in 
3D space, so it embeds both a vector (coordinate translation \f$ \mathbf{d} \f$ ) 
and a quaternion (coordinate rotation \f$ \mathbf{q} \f$ ):  

\f[
\mathbf{c}=\{\mathbf{d},\mathbf{q}\}
\f]

Since the ChCoordys is a lightweight version of a ChFrame, 
we refer the reader to the following documentation on ChFrame. 
They share lot of features.


# ChFrame   {#manual_ChFrame}

A @ref chrono::ChFrame represents a coordinate system in 3D space like 
the above mentioned ChCoordsys, but includes more advanced features.

![](coord_frame.png)

As shown in the picture above, you can consider this object 
as a piece of information that tells how rotated and 
displaced is a coordinate system **b** respect to another coordinate **a** 
(for example the absolute reference).

<div class="ce-info">
For the entire chapter, we will use the notation 
\f[
\mathbf{d}_{a,b(c)}
\f]
to define a vector \f$ \mathbf{d} \f$ ending in point \f$ a \f$, starting from point \f$ b \f$, 
expressed in the base \f$ c \f$ (that is, _measured_ along the x,y,z axes of the 
coordinate system \f$ c \f$ ).
If \f$ b \f$ is omitted, it is assumed to be the absolute reference.
</div>

Basically, as a ChCoordsys it has a vector for the translation and a quaternion for the rotation:

\f[
\mathbf{c}=\{\mathbf{d},\mathbf{q}\}
\f]

but as an alternative to the quaternion it also stores 
an auxiliary 3x3 rotation matrix that can be used to speedup 
some computations where quaternions would be less efficient. 

One can consider the ChCoordsys as a lightweight version of ChFrame. 
If you have to save memory, or if you do not need advanced features, 
ChCoordsys<> can be more convenient.

There are many ways to build a ChFrame. For instance:

~~~{.cpp}
ChFrame<> Xa;          // build default frame: zero translation, no rotation
ChFrame<> Xb(va, qa);  // build from given translation va and rotation quaternion qa
ChFrame<> Xc(csys);    // build from a given ChCoordys<>
ChFrame<> Xd(va, tetha, u); // build from translation va, rotation theta about axis u
~~~


One of the most important features of a ChFrame object is the 
ability of transforming points and other ChFrame objects.

For example, say you want to transform a point from a local coordinate system **b** 
to the absolute coordinate system **a**, as in the following picture: 

![](coord_trasf1_point.png)

Usually, one would use the following affine transformation, 
that involves a rotation by a matrix A and a translation:

\f[
\mathbf{d}_{P,a(a)}=\mathbf{d}_{b,a(a)} + \mathbf{A}_{ba} \mathbf{d}_{P,b(b)}
\f]

This of course can be done in Chrono::Engine, as 

~~~{.cpp}
ChVector<> d_Paa, d_baa, d_Pbb;
ChMatrix33<> A_ba;
...
d_Paa = d_baa + A_ba * d_Pbb;
~~~

However, thank to the ChFrame<> class, this can be made much simpler:

~~~{.cpp}
ChVector<> d_Paa, d_Pbb;
ChFrame<> X_ba;
...
d_Paa = X_ba * d_Pbb;
~~~

The same concept can be used to chain coordinate transformations, 
say you want to know which is the total rotation and total 
displacement of a frame **c** in figure, respect to frame **a**, 
if you know the transformation from **c** to **b** and from **b** to **a**, as in figure:

![](coord_trasf2_frame.png)

This boils down to:

~~~{.cpp}
ChFrame<> X_ba, X_cb, X_ca;
...
X_ca = X_ba * X_cb;
~~~

Note that this means working with a less complex representation.

We overlap symbols to show that the latter formalism has 
the same meaning of using all the displacements and quaternions:

![](coord_trasf3_frame.png)

The transformation above can be expressed with an alternative
operator >>, whose operands are swapped respect to the * operator:

~~~{.cpp}
ChFrame<> X_ba, X_cb, X_ca;
...
X_ca = X_cb >> X_ba;     // equivalent to   X_ca = X_ba * X_cb;
~~~


<div class="ce-info">
In Chrono::Engine, most transformation between ChCorrdsys<> or ChFrame 
or ChFrameMoving can be expressed in two equivalent ways: <br>
... using the * operator _RIGHT TO LEFT_ transformations, as in: 
	```X_ca = X_ba * X_cb``` <br>
... using the >> operator _LEFT TO RIGHT_ transformations, as in:
	```X_ca = X_cb >> X_ba```  <br>
</div>

<div class="ce-info">
The latter has some advantages: <br>
...  it is more 'intuitive' (see how the subscripts cb-ba follow a 'chain') <br>
...  if the first operand is a vector, as in  ```vnew = v >> X_dc >> X_cb >> X_ba```,  <br>
the default behavior of the compiler is to perform operations from the left, 
giving leading to a sequence of matrix-by-ChFrame operations returning temporary 
vectors, otherwise the * operator would create many temporary ChFrame<> 
and this would be a bit slower.
</div>

One can also use the * or >> operators with other objects, 
for example using >> between a ChFrame Xa and a ChVector vb means: 
obtain a ChFrame that is translated by a value vb:

~~~{.cpp}
ChVector<> vb;
ChFrame<> Xa, Xt; ...
Xt = Xa >> vb;    // also  Xt = vb * Xa;
~~~

The same for in-place operators *= or >>=, that can be used 
to translate a ChFrame or to rotate it, or to transform entirely, 
depending if you use it respectively with a ChVector or a ChQuaternion or another ChFrame. 
For example, to translate Xa by an amount represented by vector vb one writes:

~~~{.cpp}
Xa >>= vb;    
~~~

Note that operators * and >>  create temporary objects, 
that is not the case with *= or >>=, which are a bit more efficient then. 

In sake of maximum efficiency, one can also avoid the * or >> operators 
and use the low-level functions such as TransformLocalToParent, TransformParentToLocal, etc.

Both the * and >> algebras support the inverse element. 
Say, for example, that in the relation 
X_ca = X_cb >> X_ba; 
you know in advance X_ca and X_ba, how to compute X_cb? 
Premultiply both sides of the equation by the inverse of X_cb, 
remember that the product of X_cb by its inverse gives the unit 
element that can be remobed then from the equation, swap left and right, and write:

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
ChFrame, but it stores also information about the speed and accelerations of the frame:

\f[
\mathbf{c}=\{\mathbf{p},\mathbf{q},\dot{\mathbf{p}}, \dot{\mathbf{q}}, \ddot{\mathbf{p}}, \ddot{\mathbf{q}} \}
\f]

Note that using quaternion derivatives to express angular velocity and angular 
acceleration can be cumbersome, so this class also has the possibility of 
setting and getting such data in terms of angular velocity  
\f$ \mathbf{\omega} \f$ 
and of angular acceleration 
\f$ \mathbf{\alpha} \f$ :

\f[
\mathbf{c}=\{\mathbf{p},\mathbf{q},\dot{\mathbf{p}}, \mathbf{\omega}, \ddot{\mathbf{p}}, \mathbf{\alpha}\}
\f]

This can be shown intuitively with the following picture:

![](coord_framemoving.png)

Note that 'angular' velocities and accelerations can be set/get both in the 
basis of the moving frame or in the absolute frame.

For instance, here one creates a ChFrameMoving and assignes non-zero angular 
velocity and linear velocity, and also assignes linear and angular accelerations:

~~~{.cpp}
ChFrameMoving<> X_ba;
X_ba.SetPos(ChVector<>(2,3,5));
X_ba.SetRot(myquaternion);

// set velocity 
X_ba.SetPos_dt(ChVector<>(100,20,53)); 
X_ba.SetWvel_loc(ChVector<>(0,40,0)); // W in local frame, or..
X_ba.SetWvel_par(ChVector<>(0,40,0)); // W in parent frame

// set acceleration
X_ba.SetPos_dtdt(ChVector<>(13,16,22)); 
X_ba.SetWacc_loc(ChVector<>(80,50,0)); // a in local frame, or..
X_ba.SetWacc_par(ChVector<>(80,50,0)); // a in parent frame
~~~

One can use ChFrameMoving objects to transform ChVector (points in space), 
or ChFrame, or ChFrameMoving objects. Also speeds are computed and transformed.

For instance, see how one can compute the absolute speed and angular 
velocity of **c** respect to **a**, if one knows the transformation from **b** to **a**
and from **c** to **b**: 

![](coord_trasf5_framemoving.png)

The case above translates in the following equivalent expressions, 
using the two alternative formalisms (Left-to-right based on >> operator, 
or right-to-left based on * operator):

~~~{.cpp}
X_ca = X_cb >> X_ba;
X_ca = X_ba * X_cb;
~~~

This is _exactly the same algebra_ used in ChFrame and ChCoordsys, 
except that this time also the velocities and accelerations are transformed too.

Note that the transformation automatically takes into account the 
contributions of complex terms such as centripetal accelerations, 
relative accelerations, Coriolis acceleration, etc. 

The following is another example, with a longer concatenation of transformations:

![](coord_trasf6_framemoving.png)

Note that one can also use the inverse of frame transformations, 
using GetInverse() like we already saw for the ChFrame. 

In the following example we want to compute the position, 
speed and acceleration of the moving target 8 respect to the 
gripper 6, expressed in the basis of the frame 6.

![](coord_robotexample.png)

How to compute X_86 knowing all others? 
Start from two equivalent expressions of X_80: 

	X_86>>X_65>>X_54>>X_43>>X_32>>X_21>>X_10 = X_87>>X_70; 

also:

	X_86>>(X_65>>X_54>>X_43>>X_32>>X_21>>X_10) = X_87>>X_70;

Post multiply both sides by inverse of (...), remember that in general

- X >> X.GetInverse() = I
- X.GetInverse() >> X = I

where I is the identity transformation that can be removed, and finally get:

~~~{.cpp}
X_86 = X_87 >> X_70 >> (X_65 >> X_54 >> X_43 >> X_32 >> X_21 >> X_10).GetInverse();
~~~

Another example based on the same figure: compute speed and 
acceleration of the gripper 6 respect to the moving target 8, 
expressed in the basis of the system 8. This translates to:

~~~{.cpp}
X_68 = X_12 >> X_23 >> X_34 >> X_45 >> X_56 >> (X_87 >> X_70).GetInverse();
~~~

See @ref chrono::ChFrameMoving for API details.


# Theory

You can find additional details on the theoretical aspects of coordinate 
transformations in Chrono::Engine by looking at whitepapers.
- [PDF witepaper on rotations](http://projectchrono.org/assets/white_papers/rotations.pdf)
- [PDF witepaper on coordinates](http://projectchrono.org/assets/white_papers/frame_kinematics.pdf)


# Examples

A detailed example is demo_coords.cpp



