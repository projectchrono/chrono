
Mathematical objects in Chrono        {#mathematical_objects}
==============================

This documentation component focuses on Chrono mathematical functions and classes. 
These concepts are quite ubiquitous in the rest of the Chrono API and they are discussed also in the @ref chrono_linalg API documentation.

\tableofcontents



# Linear algebra   {#linear_algebra}

Handling vectors and matrices is a recurring theme throughout the Chrono API.
Chrono uses [Eigen3](https://eigen.tuxfamily.org/dox/index.html) for representing all matrices (dense and sparse) and vectors.

Dense matrices in Chrono are templated by the scalar type and have row-major storage order. All Chrono matrix and vector types below are simply aliases to appropriate Eigen matrix types; see @ref chrono_linalg and the ChMatrix.h header.

<div class="ce-info">
The [ChVector3](@ref chrono::ChVector3) and [ChVector2](@ref chrono::ChVector2) classes, used to represent 3D vectors in space and 2D vectors in a plane, respectively, are not Eigen types nor derived from Eigen matrices.
</div>

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

There are many matrix and vector specializations and some of their basic features are outlined next.

<br>
**Dynamic size matrices.**
Use [ChMatrixDynamic](@ref chrono::ChMatrixDynamic) to create a matrix with generic size, say 12 rows x 4 columns. ChMatrixDynamic is templated by the scalar type, with `double` as the default.

~~~{.cpp}
chrono::ChMatrixDynamic<double> A(12,4);
~~~

<br>
**Fixed size matrices.**
Use [ChMatrixNM](@ref chrono::ChMatrixNM) to create matrices that do not need to be resized and whose size is known at compile-time. 

~~~{.cpp}
chrono::ChMatrixNM<double,4,4> B;
~~~

<div class="ce-info">
**From the Eigen documentation:**
<br><br>
When should one use fixed sizes and when should one prefer dynamic sizes? The simple answer is: use fixed sizes for very small sizes where you can, and use dynamic sizes for larger sizes or where you have to. For small sizes, especially for sizes smaller than (roughly) 16, using fixed sizes is hugely beneficial to performance, as it allows Eigen to avoid dynamic memory allocation and to unroll loops. 
<br><br>
The limitation of using fixed sizes, of course, is that this is only possible when you know the sizes at compile time. Also, for large enough sizes, say for sizes greater than (roughly) 32, the performance benefit of using fixed sizes becomes negligible. Worse, trying to create a very large matrix using fixed sizes inside a function could result in a stack overflow, since Eigen will try to allocate the array automatically as a local variable, and this is normally done on the stack. Finally, depending on circumstances, Eigen can also be more aggressive trying to vectorize (use SIMD instructions) when dynamic sizes are used.
</div>

<br>
**3x3 fixed size matrices.**
Use @ref chrono::ChMatrix33 "ChMatrix33" to create 3x3 matrices, which are mostly used to represent rotation matrices and 3D inertia tensors.  
ChMatrix33 is templated by the scalar type (with `double` as the default). This matrix type is derived from a 3x3 fixed-size Eigen matrix with row-major storage and offers several dedicated constructors and methods for coordinate and rotation operations.

~~~{.cpp}
	chrono::ChMatrix33d R;
~~~

<br>
**Dynamic size column vectors.**
Use [ChVectorDynamic](@ref chrono::ChVectorDynamic) to create a column vector (one-column matrix) with a generic number of rows.

~~~{.cpp}
chrono::ChVectorDynamic<double> v(12);
~~~

<br>
**Fixed size column vectors.**
Use @ref chrono::ChVectorN "ChVectorN" to create a column vector with fixed length (known at compile time).

~~~{.cpp}
chrono::ChVectorN<double,6> w;
~~~

<br>
**Row vectors.**
Use @ref chrono::ChRowVectorDynamic "ChRowVectorDynamic" and @ref chrono::ChRowVectorN "ChRowVectorN" to create row vectors (one-row matrices) with dynamic size and fixed size, respectively.

<br>
In addition to the above types, specialized 3x4, 4x3, and 4x4 matrices used in multibody formalism are defined in [ChMatrixMBD.h](https://github.com/projectchrono/chrono/blob/main/src/chrono/core/ChMatrixMBD.h).

##Basic operations with matrices##

Consult the Eigen API for all matrix and vector [arithmetic operations](https://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html), [block operations](https://eigen.tuxfamily.org/dox/group__TutorialBlockOperations.html), and [linear system solution](https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html). 

[demo_CH_linalg.cpp](https://github.com/projectchrono/chrono/blob/main/src/demos/core/demo_CH_linalg.cpp) illustrates basic operations with matrices.




# Function objects  {#ChFunction_objects}

These ChFunction objects are used in many places in Chrono, 
and are used to represent y=f(x) functions, 
for example when introducing prescribed displacements in a linear actuator.

These functions are scalar, 
\f[ 
 x \in \mathbb{R} \rightarrow y \in \mathbb{R}
\f]
and there are a predefined number of them that are ready to use, 
such as sine, cosine, constant, etc. If the predefined ones are not enough, 
the user can implement his custom function by inheriting from the base ChFunction class.

See @ref chrono::ChFunction "ChFunction" for API details and a list of subclasses.


### Example 1

~~~{.cpp}
	ChFunctionRamp f_ramp;

	f_ramp.SetAngularCoeff(0.1);	// set angular coefficient;
	f_ramp.SetStartVal(0.4);		// set y value for x=0;

	// Evaluate y=f(x) function at a given x value, using GetVal() :
	double y	= f_ramp.GetVal(10);
	// Evaluate derivative df(x)/dx at a given x value, using GetDer() :
	double ydx	= f_ramp.GetDer(10);

	std::cout << "   ChFunctionRamp at x=0: y=" << y << "  dy/dx=" << ydx << std::endl;
~~~


### Example 2

Save values of a sine ChFunction  into a file.

~~~{.cpp}
	ChFunctionSine f_sine;

	f_sine.SetAmplitude(2);		// set amplitude;
	f_sine.SetFrequency(1.5);	// set frequency;

	std::ofstream file_f_sine ("f_sine_out.dat");

	// Evaluate y=f(x) function along 100 x points, and its derivatives, 
	// and save to file (later it can be loaded, for example, in Matlab)
	for (int i=0; i<100; i++)
	{
		double x = (double)i/50.0;
		double y = f_sine.GetVal(x);
		double ydx = f_sine.GetDer(x);
		double ydxdx = f_sine.GetDer2(x);
		file_f_sine << x << " " << y << " " << ydx << " " << ydxdx << std::endl;
	}	
~~~


### Example 3

Define a custom function.

The following class will be used as an example of how you can create
custom functions based on the ChFunction interface.

There is at least one mandatory member function to implement: 
__GetVal__.

Note that the base class implements a default computation of 
derivatives GetDer() and GetDer2() by using a numerical differentiation, 
however if you know the analytical expression of derivatives, 
you can override the base GetDer() and GetDer2() too, for higher precision.

~~~{.cpp}
// First, define a custom class inherited from ChFunction

class ChFunctionMyTest : public ChFunction
{
public:
	ChFunction* new_Duplicate() {return new ChFunctionMyTest;} 
	double GetVal(double x) {return cos(x);} // just for test: simple cosine
};

ChFunctionMyTest f_test;

std::ofstream file_f_test ("f_test_out.dat");

// Evaluate y=f(x) function along 100 x points, and its derivatives, 
// and save to file (later it can be loaded, for example, in Matlab)
for (int i=0; i<100; i++)
{
	double x = (double)i/50.0;
	double y = f_test.GetVal(x);
	double ydx = f_test.GetDer(x);
	double ydxdx = f_test.GetDer2(x);
	file_f_test << x << " " << y << " " << ydx << " " << ydxdx << std::endl;
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
	ChQuadrature::Integrate1D<double>(qresult, mfx,  0, CH_PI,  6);
	
	std::cout << "Quadrature 1d result:" << qresult << " (analytic solution: 2.0)" << std::endl;

	
	// Other quadrature tests, this time in 2D

	class MySine2d : public ChIntegrable2D<double>
	{
	public: 
		void Evaluate (double& result, const double x, const double y) { result = sin(x); }
	};

	MySine2d mfx2d;
	ChQuadrature::Integrate2D<double>(qresult, mfx2d, 0, CH_PI, -1,1, 6);
	std::cout << "Quadrature 2d result:" << qresult << " (analytic solution: 4.0)" << std::endl;
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
	std::cout << "Quadrature 2d matrix result:" << resultM << " (analytic solution: 2.25, 4.5)" << std::endl;
~~~

<br><br><br>
