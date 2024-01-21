
Coordinate Transformations        {#coordinate_transformations}
==============================

This documentation component introduces important concepts used through the entire Chrono API for managing coordinates, points, transformations and rotations.

\tableofcontents


# Vectors  {#manual_ChVector}


Vectors that represent 3D points in space are defined by means of the 
@ref chrono::ChVector class. 
In mathematical notation:

\f[ 
\mathbf{p}=\{p_x,p_y,p_z\}
\f]

The ChVector<> class is templated. One can have vectors with single 
precision, as ChVector<float>, with double precision,
as ChVector<double>, etc. 
The default data type is `double` precision; i.e., ChVector<> defaults to ChVector<double>.
 

Example, creating a vector:

~~~{.cpp}
ChVector<> mvect1(2,3,4);
~~~
Add, subtract, multiply vectors, using +,-,* operators. 

~~~{.cpp}
ChVector<double> mvect1(2,3,4);	 // Create a vector with given the 2,3,4 ‘double’ components

ChVector<float> mvect2(4,1,2); 	// Create a vector with given the 4,1,2 ‘float’ components

ChVector<> mvect3(); 	// Create a 0,0,0, vector. The <> defaults to ‘double’

ChVector<> mvect4(mvect1 + mvect2);	 // Create a vector by copying another (a result from +)

mvect3 = mvect1 + mvect2; 	// Vector operators: +, - 

mvect3 += mvect1;		// In-place operators

mvect3 = mvect2 * 0.003; 	// Vector product by scalar

mvect3.Normalize();		// One of many member functions, normalizes a vector

mvect3 = mvect1 % mvect2;  	// Operator for cross product: A%B means vector cross-product AxB

double val = mvect1 ^ mvect2;  	// Operator for inner product (scalar product)
~~~

Chrono offers also an useful set of constant (double) vectors - `VNULL`, `VECT_X`, `VECT_Y`, `VECT_Z`, that represent the null and the axes unit vectors respectively.

# Quaternions  {#manual_ChQuaternion}

Quaternions, which provide a number system that extends complex numbers, are used is Chrono to represent rotations in 3D space.
The Chrono implementation of quaternions is encapsulated in the @ref chrono::ChQuaternion class. 
In mathematical notation, a quaternion is represented as a set of four real numbers:

\f[
\mathbf{q}=\{q_0,q_1,q_2,q_3\}
\f]

Quaternion algebra properties:


- A real unit quaternion \f$ \mathbf{q}=\{1,0,0,0\} \f$ represents no rotation
- Given a rotation  \f$ \theta_{u} \f$ about a generic unit vector \f$ \mathbf{u} \f$, 
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

![](http://www.projectchrono.org/assets/manual/coord_quaternions.png)

Only quaternions with unit norm represent valid rotations. 
Quaternions can be built in different ways.

Examples of building an equivalent quaternion that represents no rotation:

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
by using the .Rotate() member function of quaternions. The example below shows how to rotate a point 20 degrees about y-axis:

~~~{.cpp}
ChVector vA(1,2,3);
ChVector vB();
ChQuaternion qA = Q_from_AngAxis(20 * CH_C_DEG_TO_RAD, VECT_Y);
vB = qA.Rotate(vA);
~~~

The * operator is used to perform a quaternion product. From a kinematic perspective, this represents concatenation of rotations.
 
Example: a rotation qc followed by a rotation qb can be 
shortened in a single qa, if qa, qb, and qc are quaternions, as follows:

~~~{.cpp}
qa = qb * qc;	   // Concatenate two rotations, first qc, followed by qb
qa.Rotate(mvect1); 
~~~

Alternatively, the  >>  operator concatenates from left to right:

~~~{.cpp}
qa = qc >> qb;	   // Concatenate two rotations, first qc, followed by qb  
qa.Rotate(mvect1); // (Same result as example before)
~~~


# Rotation matrices  {#manual_ChMatrix33}

A rotation matrix \f$ \mathbf{A} \f$ is used in Chrono to characterize the 3D orientation of a reference frame with respect to a different reference frame. Most of the implementation support related to the concept of rotation matrix is encapsulated in @ref chrono::ChMatrix33. There are Chrono functions that given a quaternion produce a 3x3 orientation matrix 
  \f$ \mathbf{q} \mapsto \mathbf{A} \f$ and viceversa \f$ \mathbf{A} \mapsto \mathbf{q} \f$. Note that a rotation matrix is an orthnormal matrix, \f$ \mathbf{A} \in \mathsf{SO}(3) \f$. Therefore, \f$ \mathbf{A}^{-1} = \mathbf{A}^{t} \f$


Creating a rotation matrix:

~~~{.cpp}
ChMatrix33<> mB(1);  // Unit matrix, this is a rotation matrix, meaning no rotation
ChMatrix33<> mC(quat); // Build a rotation matrix from a given quaternion
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
which is also the transpose.


# ChCoordsys   {#manual_ChCoordsys}

A @ref chrono::ChCoordsys represents a coordinate system in 
3D space. It embeds both a vector (coordinate translation \f$ \mathbf{d} \f$ ) 
and a quaternion (coordinate rotation \f$ \mathbf{q} \f$ ):  

\f[
\mathbf{c}=\{\mathbf{d},\mathbf{q}\}
\f]

The ChCoordys is a lightweight version of a ChFrame, which is discussed next.


# ChFrame   {#manual_ChFrame}

A @ref chrono::ChFrame represents a coordinate system in 3D space like 
ChCoordsys but includes more advanced features.

![](http://www.projectchrono.org/assets/manual/coord_frame.png)

As shown in the picture above, a ChFrame object 
indicates how "rotated" and 
"displaced" a coordinate system **b** is with respect to another coordinate system **a** 
Many time **a** is the absolute reference frame.

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

As an alternative to the quaternion a ChFrame object also stores 
an auxiliary 3x3 rotation matrix that can be used to speed up 
computations where quaternions would be less efficient. 

ChCoordsys can be considered as a lightweight version of ChFrame. 
To save memory or if advanced features are not needed, 
ChCoordsys<> can be a better choice.

There are several ways to build a ChFrame. For instance:

~~~{.cpp}
ChFrame<> Xa;          // build default frame: zero translation, no rotation
ChFrame<> Xb(va, qa);  // build from given translation va and rotation quaternion qa
ChFrame<> Xc(csys);    // build from a given ChCoordys<>
ChFrame<> Xd(va, tetha, u); // build from translation va and rotation theta about axis u
~~~


One of the most important features of a ChFrame object is the 
ability to transform points and other ChFrame objects.

Example: Transforming a point from a local coordinate system **b** 
to the absolute coordinate system **a**:

![](http://www.projectchrono.org/assets/manual/coord_trasf1_point.png)

Usually an affine transformation involving rotation 
by a matrix A and a translation is used:


\f[
\mathbf{d}_{P,a(a)}=\mathbf{d}_{b,a(a)} + \mathbf{A}_{ba} \mathbf{d}_{P,b(b)}
\f]

This can be done in Chrono as follows:

~~~{.cpp}
ChVector<> d_Paa, d_baa, d_Pbb;
ChMatrix33<> A_ba;
...
d_Paa = d_baa + A_ba * d_Pbb;
~~~


However, the ChFrame<> class makes this simpler:

~~~{.cpp}
ChVector<> d_Paa, d_Pbb;
ChFrame<> X_ba;
...
d_Paa = X_ba * d_Pbb;
~~~

The same concept can be used to chain coordinate transformations. For instance, if the transformation from a frame **c** to **b** and from **b** to **a** is known,
then the total frame rotation and displacement can be found.

![](http://www.projectchrono.org/assets/manual/coord_trasf2_frame.png)


~~~{.cpp}
ChFrame<> X_ba, X_cb, X_ca;
...
X_ca = X_ba * X_cb;
~~~

This translates into working with a less complex representation. Symbols are recycled to emphasize that the latter formalism has 
the same meaning insofar using the displacements and quaternions:

![](http://www.projectchrono.org/assets/manual/coord_trasf3_frame.png)

The transformation above can be expressed with an alternative
operator >>, whose operands are swapped with respect to the * operator:

~~~{.cpp}
ChFrame<> X_ba, X_cb, X_ca;
...
X_ca = X_cb >> X_ba;     // equivalent to   X_ca = X_ba * X_cb;
~~~


<div class="ce-info">
In Chrono, most of the transformation between ChCorrdsys<> and ChFrame 
and ChFrameMoving (which is defined below) can be expressed in two equivalent ways: <br>
... using the * operator _RIGHT TO LEFT_ transformations, as in: 
	```X_ca = X_ba * X_cb``` <br>
... using the >> operator _LEFT TO RIGHT_ transformations, as in:
	```X_ca = X_cb >> X_ba```  <br>
</div>

<div class="ce-info">
The latter has some advantages: <br>
...  it is more 'intuitive' (see how the subscripts cb-ba follow a 'chain') <br>
...  if the first operand is a vector, as in  ```vnew = v >> X_dc >> X_cb >> X_ba```,  <br>
The default behavior of the compiler is to perform operations from the left, 
leading to a sequence of matrix-by-ChFrame operations returning temporary 
vectors. Otherwise, the * operator would create several ChFrame<> temporary objects,
which would be slower.
</div>

One can also use the * or >> operators with other objects, 
for example using >> between a ChFrame Xa and a ChVector vb. The outcome of this 
operation is a new ChFrame object obtained by translating the old one by a vector vb:

~~~{.cpp}
ChVector<> vb;
ChFrame<> Xa, Xt; ...
Xt = Xa >> vb;    // also  Xt = vb * Xa;
~~~

The same holds for in-place operators *= or >>=, which can be used 
to translate or to rotate a ChFrame, or to transform entirely, 
depending on how it is used: with a ChVector or a ChQuaternion or another ChFrame. 
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

A ChFrameMoving can be used to transform ChVector (points in space), a ChFrame, or ChFrameMoving objects. Velocities are also computed and transformed.

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


# Theory

Additional details on the theoretical aspects of coordinate 
transformations in Chrono:
- [PDF whitepaper on rotations](http://projectchrono.org/assets/white_papers/rotations.pdf)
- [PDF whitepaper on coordinates](http://projectchrono.org/assets/white_papers/frame_kinematics.pdf)


# Examples

- [demo_coords](\ref tutorial_demo_coords)



