
Mathematical objects in Chrono        {#mathematical_objects}
==============================

This documentation component focuses on Chrono mathematical functions and classes. 
These concepts are quite ubiquitous in the rest of the Chrono API and they are discussed also in the @ref chrono::ChMatrix API documentation.

\tableofcontents



# Linear algebra   {#linear_algebra}

Handling vectors and matrices is a recurring theme throughout the Chrono API.
ChMatrix is the base class for matrices and vectors and it is used extensively.

Matrices are indexed starting from 0, with (row,column) indexing:

\f[
\mathbf{A}=\left[
\begin{array}{cccc}
a_{0,0} & a_{0,1} & a_{0,2} & ... \\
a_{1,0} & a_{1,1} & a_{1,2} & ... \\
a_{2,0} & ... & ... & ... \\
a_{n_{rows}-1,0} & ... & ... & a_{n_{rows}-1,n_{cols}-1} 
\end{array}
\right]
\f]

There are many specializations of ChMatrix and some of their basic features are outlined next.


##ChMatrixDynamic##

Use ChMatrixDynamic<> to create a matrix with generic size, say 12 rows x 4 columns. Owing to the template argument, one can indicate what type of data the matrix is supposed to store. Note that the default type is 'double'. If performance is not a top issue, this is the type of matrix that is most accurate.

~~~{.cpp}
chrono::ChMatrixDynamic<double> mh(12,4);
~~~

See @ref chrono::ChMatrixDynamic for API details.

##ChMatrixNM##

Use ChMatrixNM<> to create matrices that do not need to be resized and whose size is known at compile-time. Columns x rows are passed in template <..> brackets. Although the functionalities are the same of ChMatrixDynamic<>, this type of matrix has better performance. Avoid using it for large M and N sizes since it is allocated on the stack.

~~~{.cpp}
chrono::ChMatrixNM<double,4,4> mm;
~~~

See @ref chrono::ChMatrixNM for API details.


##ChMatrix33##

Use ChMatrix33<> to create 3x3 matrices, which are mostly used for coordinate transformations.
It inherits the same high-performance features of ChMatrixNM<> and offers additional dedicated functions for coordinate and rotation operations.

~~~{.cpp}
	chrono::ChMatrix33<> ma;
~~~

See @ref chrono::ChMatrix33 for API details.


##ChVectorDynamic##

Use ChVectorDynamic<> to create a one-column matrix with a generic number of rows.
<div class="ce-info">
This class is different than the ChVector class. The latter is used to represent 3D vectors in space.
</div>

~~~{.cpp}
chrono::ChVectorDynamic<double> mv(12);
~~~

See @ref chrono::ChVectorDynamic for API details.



##Basic operations with matrices##

~~~{.cpp}
		// Fill a matrix with an element
	mm.FillElem(0.1);

		// Print a matrix to cout (ex. the console, if open)
	mm.StreamOUT(GetLog());


		// Create a 5x7 random matrix
	chrono::ChMatrixDynamic<> me(5,7);
	me.FillRandom(-1, 2);

		// Transpose the matrix in place
	me.MatrTranspose();

		// Reset the matrix to zero (and modify size, if necessary).
	me.Reset(2,2);


		// Create a diagonal matrix
	chrono::ChMatrixDynamic<> md(4,4);
	md.FillDiag(3);

		// Use the () operator to reference matrix entries
	md(0,0) = 10;
	md(1,2) = md(0,0)+md(1,1);
	md.Element(2,2) = 4;	// The md.Element(.,.) function has the same effect as md(.,.)


		// Copy constructor
	chrono::ChMatrixDynamic<> ma1(md);


		// The - unary operator returns a negated matrix
	chrono::ChMatrixDynamic<> ma2(-mm);


		// A late copy - matrix will be resized if necessary
	chrono::ChMatrixDynamic<> ma3(8,4);
	ma3.CopyFromMatrix(ma1);

		// Transposed copy. 
		// This is faster than doing: ma3.CopyFromMatrix(ma2); ma3.MatrTranspose();
	ma3.CopyFromMatrixT(ma1);	
~~~

<br>
Rely on C++ operator overloading to use the  + * -  *= += -=  operators to perform matrix algebra.

<div class="ce-info">
Note that the + - *  operators may introduce overhead because they instantiate temporary
matrix objects as intermediate results. Use *= -= +=  operators whenever possible,  
or use the specific Add() Multiply() functions for highest computational speed.
</div>

~~~{.cpp}
			// Note: size of result is automatically set because of copy constructor
	chrono::ChMatrixDynamic<> result(ma1+ma2); 

			// Using the assignment operator (size of result may be automatically reset)
	result = ma1+ma2;	

	 		// Another way to do operations - more intricate, but allows higher performances.
			// Note that you must prepare 'result' with already appropriate column/row size.
	result.MatrAdd(ma1,ma2);

			// Use of the += operator typically minimizes the need of intermediate temporary matrices.
	result = ma1;
	result += ma2;

			// Different ways to do subtraction...

	result = ma1-ma2;	

	result.MatrSub(ma1,ma2);

	result = ma1;
	result -= ma2;


			// Multiplications between two matrices, different methods...

	result = ma1*ma2;	

	result.MatrMultiply(ma1,ma2);
 

			// Multiplication between matrix and scalars, different methods...

	result = ma1*10;
	
	result = ma1;
	result.MatrScale(10);
	GetLog() << result;

	result = ma1;
	result *=10;
	GetLog() << result;

			// Dot multiplication
	chrono::ChMatrixDynamic<> mmv1(5,1);
	chrono::ChMatrixDynamic<> mmv2(5,1);
	mmv1.FillRandom(1,3);
	mmv2.FillRandom(2,4);
	double mdot = chrono::ChMatrix<>::MatrDot(&mmv1, &mmv2);
    
			// Elementwise matrix comparison
	chrono::ChMatrixDynamic<> mmv3(mmv2);
	if (mmv2==mmv3) 
		GetLog() << "Matrices are exactly equal \n";

			// Tolerance comparison
	mmv3.Element(2,0)+=0.001;
	if (mmv2.Equals(mmv3,0.002)) 
		GetLog() << "Matrices are equal within tol 0.002 \n";
~~~

<br>

The matrices can operate also on 3D vectors \f$ \mathbf{v}=\{v_x,v_y,v_z\} \f$, 
that are defined with the ChVector<> class (the Vector is a shortcut for ChVector<double> ). Example:

~~~{.cpp}
	chrono::Vector mvect(1,2,3);	
	chrono::Quaternion mquat(1,2,3,4);
	chrono::ChMatrix33<> mta1;
	mta1.FillRandom(-1, 2);

			// Vector transformation, typical product [A]*v
	chrono::Vector vres  = mta1.Matr_x_Vect(mvect);

			// More compact syntax: operator * between matrix and vector...
	chrono::Vector vres2 = mta1*mvect;
	if (vres == vres2) 
		GetLog() << "vectors are equal \n";

			// .. same, but with a transposed matrix
	vres = mta1.MatrT_x_Vect(mvect);

			// Custom multiplication functions for 3x4 matrices and quaternions:
	chrono::ChMatrixNM<double,3,4> mgl;
	mgl.FillRandom(-1, 2);
	vres = mgl.Matr34_x_Quat(mquat);

	chrono::Quaternion qres = mgl.Matr34T_x_Vect(mvect);

	chrono::ChMatrixNM<double,4,4> mxq;
	mxq.FillRandom(-1, 2);
	qres = mxq.Matr44_x_Quat(mquat);
~~~




# Function objects  {#ChFunction_objects}



These ChFunction objects are used in many places in Chrono::Engine, 
and are used to represent y=f(x) functions, 
for example when introducing prescribed displacements in a linear actuator.

These functions are scalar, 
\f[ 
 x \in \mathbb{R} \rightarrow y \in \mathbb{R}
\f]
and there are a predefined number of them that are ready to use, 
such as sine, cosine, constant, etc. If the predefined ones are not enough, 
the user can implement his custom function by inheriting from the base ChFunction class.

See @ref chrono::ChFunction for API details and a list of subclasses.


### Example 1

~~~{.cpp}
	ChFunction_Ramp f_ramp;

	f_ramp.Set_ang(0.1);	// set angular coefficient;
	f_ramp.Set_y0(0.4);		// set y value for x=0;

	 // Evaluate y=f(x) function at a given x value, using Get_y() :
	double y	= f_ramp.Get_y(10);
	 // Evaluate derivative df(x)/dx at a given x value, using Get_y_dx() :
	double ydx	= f_ramp.Get_y_dx(10);

	GetLog() << "   ChFunction_Ramp at x=0: y=" << y << "  dy/dx=" << ydx << "\n\n";
~~~


### Example 2

Save values of a sine ChFunction  into a file.

~~~{.cpp}
	ChFunction_Sine f_sine;

	f_sine.Set_amp(2);		// set amplitude;
	f_sine.Set_freq(1.5);	// set frequency;

	ChStreamOutAsciiFile file_f_sine ("f_sine_out.dat");

	 // Evaluate y=f(x) function along 100 x points, and its derivatives, 
	 // and save to file (later it can be loaded, for example, in Matlab)
	for (int i=0; i<100; i++)
	{
		double x = (double)i/50.0;
		double y = f_sine.Get_y(x);
		double ydx = f_sine.Get_y_dx(x);
		double ydxdx = f_sine.Get_y_dxdx(x);
		file_f_sine << x << " " << y << " " << ydx << " " << ydxdx << "\n";
	}	
~~~


### Example 3

Define a custom function.

The following class will be used as an example of how you can create
custom functions based on the ChFunction interface.

There is at least one mandatory member function to implement: 
__Get_y()__.

Note that the base class implements a default computation of 
derivatives Get_ydx() and Get_ydxdx() by using a numerical differentiation, 
however if you know the analytical expression of derivatives, 
you can override the base Get_ydx() and Get_ydxdx() too, for higher precision.

~~~{.cpp}
// First, define a custom class inherited from ChFunction

class ChFunction_MyTest : public ChFunction
{
public:
	ChFunction* new_Duplicate() {return new ChFunction_MyTest;} 
	double Get_y(double x) {return cos(x);} // just for test: simple cosine
};

ChFunction_MyTest f_test;

ChStreamOutAsciiFile file_f_test ("f_test_out.dat");

 // Evaluate y=f(x) function along 100 x points, and its derivatives, 
 // and save to file (later it can be loaded, for example, in Matlab)
for (int i=0; i<100; i++)
{
	double x = (double)i/50.0;
	double y = f_test.Get_y(x);
	double ydx = f_test.Get_y_dx(x);
	double ydxdx = f_test.Get_y_dxdx(x);
	file_f_test << x << " " << y << " " << ydx << " " << ydxdx << "\n";
}
~~~




# Quadrature    {#quadrature}

	
Quadrature is an operation that computes integrals as when computing areas and volumes.

The following code shows how to use the Gauss-Legendre quadrature to compute the integral of a function  
\f$ \mathbb{R} \mapsto \mathbb{R} \f$ 
over a 1D interval, or 
\f$ f: \mathbb{R}^2 \mapsto \mathbb{R}\f$  over a 2D interval or 
\f$ f: \mathbb{R}^3 \mapsto \mathbb{R}\f$  over a 3D interval: 

\f[
	F_{1D}=\int^a_b f(x) dx
\f]	

\f[
	F_{2D}=\int^{a_y}_{b_y}\int^{a_x}_{b_x} f(x,y) dx dy
\f]	

\f[
	F_{3D}=\int^{a_z}_{b_z}\int^{a_y}_{b_y}\int^{a_x}_{b_x} f(x,y,z) dx dy dz
\f]	

If the function is polynomial of degree N and the quadrature is of order N, 
the result is exact, otherwise it is approximate (using large N improves 
quality but remember that this type of integration is often used where N in the 
range 1..10 suffices, otherwise other integration methods might be better). 

For N less than 10, the quadrature uses precomputed coefficients for maximum performance.



~~~{.cpp}
						// Define a y=f(x) function by inheriting ChIntegrable1D:
	class MySine1d : public ChIntegrable1D<double>
	{
	public: 
		void Evaluate (double& result, const double x)  {
			result = sin(x);
		}
	};
						// Create an object from the function class
	MySine1d mfx;
						// Invoke 6th order Gauss-Legendre quadrature on 0..PI interval:
	double qresult;	
	ChQuadrature::Integrate1D<double>(qresult, mfx,  0, CH_C_PI,  6);
	
	GetLog()<< "Quadrature 1d result:" << qresult << " (analytic solution: 2.0) \n";

	
						// Other quadrature tests, this time in 2D

	class MySine2d : public ChIntegrable2D<double>
	{
	public: 
		void Evaluate (double& result, const double x, const double y) { result = sin(x); }
	};

	MySine2d mfx2d;
	ChQuadrature::Integrate2D<double>(qresult, mfx2d, 0, CH_C_PI, -1,1, 6);
	GetLog()<< "Quadrature 2d result:" << qresult << " (analytic solution: 4.0) \n";
~~~

Note that thanks to templating, one can also integrate 
m-dimensional (vectorial, tensorial) functions 
\f$ \mathbf{f}: \mathbb{R}^n \mapsto \mathbb{R}^m \f$ , for example: 

\f[
  \mathbf{F}=\int^{a_y}_{b_y}\int^{a_x}_{b_x} \mathbf{f}(x,y) dx dy \quad \mathbf{F} \in \mathbb{R}^2
\f]

~~~{.cpp}
	class MySine2dM : public ChIntegrable2D< ChMatrixNM<double,2,1> >
	{
	public: 
		void Evaluate (ChMatrixNM<double,2,1>& result, const double x, const double y) 
		{ 
			result(0) = x*y;
			result(1) = 0.5*y*y;
		}
	};

	MySine2dM mfx2dM;
	ChMatrixNM<double,2,1> resultM;
	ChQuadrature::Integrate2D< ChMatrixNM<double,2,1> >(resultM, mfx2dM, 0, 1, 0,3, 6);
	GetLog()<< "Quadrature 2d matrix result:" << resultM << " (analytic solution: 2.25, 4.5) \n";
~~~

