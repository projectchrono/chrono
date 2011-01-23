#ifndef CHFUNCT_H
#define CHFUNCT_H

//////////////////////////////////////////////////
//  
//   ChFunction.h
//
//   Function objects, 
//   as scalar functions of scalar variable y=f(t)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>
#include <list>

#include "core/ChApiCE.h"
#include "core/ChMath.h"
#include "physics/ChFilePS.h"
#include "physics/ChProplist.h"


struct JSScript; // forward reference;


namespace chrono 
{


// Each ChFunction must have an unique integer identifier,
// for fast dynamic type checking and downcasting.
// If you implement new functions inherited from ChFunction,
// remember to define an unique FUNCT_xxx identifier for it.

#define FUNCT_CONST		0
#define FUNCT_RAMP		1
#define FUNCT_SINE		2
#define FUNCT_MOCAP		3
#define FUNCT_POLY		4
#define FUNCT_SIGMA     5
#define FUNCT_SEQUENCE	7
#define FUNCT_CONSTACC	8
#define FUNCT_POLY345	9
#define FUNCT_FILLET3	10
#define FUNCT_RECORDER	11
#define FUNCT_OPERATION 12
#define FUNCT_MATLAB    13
#define FUNCT_JSCRIPT	14
#define FUNCT_NOISE		15
#define FUNCT_DERIVE	16
#define FUNCT_INTEGRATE	17
#define FUNCT_MIRROR	18
#define FUNCT_REPEAT	19
#define FUNCT_OSCILLOSCOPE	20


#define POLY_COEFF_ARRAY  6


#define CHF_FILE_HEADER_ID  1234.56



#define OPT_VARIABLES_START virtual char** GetOptVariables() {static char* mOptVars[]={
#define OPT_VARIABLES_END   0};return mOptVars;}


///////////////////////////
// THE BASE FUNCTION CLASS:
// 
//  y= f(x)
// 
///////////////////////////
///  The ChFunction class defines the base class for all Chrono
/// functions of type y=f(x), that is scalar functions of an 
/// input variable x (usually, the time). ChFunctions are often
/// used to set time-dependent properties, for example to set
/// motion laws in linear actuators, engines, etc.
///  This base class just represent a constant function of
/// the type y= C.  Inherited classes must override at least the
/// Get_y() method, in order to represent more complex functions.

class ChApi ChFunction
{
						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI_ROOT(ChFunction);

private:

	double y;			// the return value

public:
	ChFunction () {y = 0;};
	ChFunction (double y_constant) {y = y_constant;};
	virtual ~ChFunction () {};
	virtual void Copy (ChFunction* source);
	virtual ChFunction* new_Duplicate ();

				/// Each class inherited from the ChFunction class must
				/// return an unique integer identifier with the virtual
				/// function Get_type(). This is useful for run-time downcasting
				/// etc.
	virtual int Get_Type () {return (FUNCT_CONST);}				
	
				/// Set the constant C for the function, y=C.
	void Set_yconst (double y_constant) {y = y_constant;};
				/// Get the constant C for the function, y=C.
	double Get_yconst () {return y;};


	
		///// THE MOST IMPORTANT MEMBER FUNCTIONS /////    
		//       At least Get_y() should be overridden
		//       by inherited classes.


				/// Returns the y value of the function, at position x. (For this
				/// base class, it will be a constant value y=C).
	virtual double Get_y      (double x) {return y;};

				/// Returns the dy/dx derivative of the function, at position x. 
				///  Note that inherited classes may also avoid overriding this method,
				/// because this base method already provide a general-purpose numerical differentiation
				/// to get dy/dx only from the Get_y() function. (however, if the analytical derivative
				/// is known, it may better to implement a custom method).
	virtual double Get_y_dx   (double x) {return ((Get_y(x+ BDF_STEP_LOW) - Get_y(x)) / BDF_STEP_LOW); }//((Get_y(x+BDF_STEP_VERYLOW) - Get_y(x)) / BDF_STEP_VERYLOW);};  // return 0;
	
				/// Returns the ddy/dxdx double derivative of the function, at position x. 
				///  Note that inherited classes may also avoid overriding this method,
				/// because this base method already provide a general-purpose numerical differentiation
				/// to get ddy/dxdx only from the Get_y() function. (however, if the analytical derivative
				/// is known, it may be better to implement a custom method).
	virtual double Get_y_dxdx (double x) {return ((Get_y_dx(x+BDF_STEP_LOW) - Get_y_dx(x)) / BDF_STEP_LOW);}; // return 0;

		//
		////////////////////////////////////////
	
				/// Returns the weight of the function (useful for 
				/// applications where you need to mix different weighted ChFunctions)
	virtual double Get_weight (double x) {return 1.0;};
	
				/// These functions can be used to implement automatic zooming 
				/// on the most representative range of function (if GUI is implemented)
	virtual void Extimate_x_range (double& xmin, double& xmax) {xmin = 0.0; xmax = 1.2;};
	virtual void Extimate_y_range (double xmin, double xmax, double& ymin, double& ymax, int derivate);
	
				/// Generic derivative: if derivate=0 is like Get_y(), if derivate=1 
				/// is like Get_y_dx(), etc. 
				/// Note: in current release 'derivate' can be only 0,1,2
	virtual double Get_y_dN (double x, int derivate);
		
		//
		// Some analysis functions. If derivate=0, they are applied on y(x), if derivate =1, on dy/dx, etc.
		// 

				/// Computes the maximum of y(x) in a range xmin-xmax, using a sampling method.
	virtual double Compute_max(double xmin, double xmax, double sampling_step, int derivate);
				/// Computes the minimum of y(x) in a range xmin-xmax, using a sampling method.
	virtual double Compute_min(double xmin, double xmax, double sampling_step, int derivate);
				/// Computes the mean value of y(x) in a range xmin-xmax, using a sampling method.
	virtual double Compute_mean(double xmin, double xmax, double sampling_step, int derivate);
				/// Computes the square mean val. of y(x) in a range xmin-xmax, using sampling.
	virtual double Compute_sqrmean(double xmin, double xmax, double sampling_step, int derivate);
				/// Computes the integral of y(x) in a range xmin-xmax, using a sampling method.
	virtual double Compute_int(double xmin, double xmax, double sampling_step, int derivate);
				/// Computes the positive acceleration coefficient (inherited classes should customize this).
	virtual double Get_Ca_pos () {return 0;};
				/// Computes the positive acceleration coefficient (inherited classes should customize this).
	virtual double Get_Ca_neg () {return 0;};
				/// Computes the speed coefficient (inherited classes must customize this).
	virtual double Get_Cv () {return 0;};


		// Functions which build the tree of optimization variables (or expands it, from given tree)
		// Each function class should implement (at least) the OPT_VARIABLES_START/END macro, that is return a null-terminated
		// array of variables which can be get/set as floating point during optimizations.
		// Variables are strings in C++ /Java/Javascript syntax, that must be interpreted with 
		// some scripting engine (ex. the Javascript one); see OptVariablesToVector and VectorToOptVariables js tools.

	virtual int MakeOptVariableTree(ChList<chjs_propdata>* mtree);
	static int  VariableTreeToFullNameVar(ChList<chjs_propdata>* mtree, ChList<chjs_fullnamevar>* mlist);
	virtual int OptVariableCount();


	OPT_VARIABLES_START		// here expose no vars ('C' var is added hard-wired in MakeOptVariableTree)
	OPT_VARIABLES_END		// Inherited classes, at least implement this. If they enclose children funct. obj, also implement the MakeOptVariableTree().


				/// If the function has some handles (mouse-sensible markers on screen), 
				/// implement these functions:
	virtual int HandleNumber() {return 0;}		// Returns the number of handles of the function
				/// Gets the x and y position of handle, given identifier. 
				/// If set mode, x and y values are stored. Return false if handle not found.
	virtual int HandleAccess(int handle_id, double mx, double my, bool set_mode) {return TRUE;};


			//
			// STREAMING
			//

			// Persistent data serialization: file functions. 
			// Inherited classes may inherit these methods because they could
			// have additional data.

					/// Method to allow serialization of transient data in ascii,
					/// as a readable item, for example   "chrono::GetLog() << myobject;"
	virtual void StreamOUT(ChStreamOutAscii& mstream);

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	virtual void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	virtual void StreamOUT(ChStreamOutBinary& mstream);

		
				/// Plot function in graph space of the ChFile_ps postscript file
				/// where zoom factor, centering, colour, thickness etc. are already defined.
				/// If plotDY=true, plots also the derivative, etc.
	virtual int FilePostscriptPlot(ChFile_ps* m_file, int plotY, int plotDY, int plotDDY);

				/// Save function as X-Y pairs separated by space, with CR at each pair,
				/// into an Ascii file. 
				/// The output file can be later loaded into Excel, GnuPlot or other tools.
				/// The function is 'sampled' for nsteps times, from xmin to xmax.
	virtual int FileAsciiPairsSave(ChStreamOutAscii& m_file, double xmin=0, double xmax=1, int msamples=200);

};




//////////////////////////
// DERIVED CLASSES:
//////////////////////////


////////////////////////////////////////////
/// LINEAR FUNCTION (like a straight ramp)
/// y = y0 + x * speed
/// 

class ChApi ChFunction_Ramp : public ChFunction
{
	CH_RTTI(ChFunction_Ramp, ChFunction);
private:
	double y0;
	double ang;
public:
	ChFunction_Ramp () {y0 = 0; ang=1;};
	ChFunction_Ramp (double m_y0, double m_ang)  {y0 = m_y0; ang = m_ang;};
	~ChFunction_Ramp () {};
	void Copy (ChFunction_Ramp* source);
	ChFunction* new_Duplicate ();

	int Get_Type () {return (FUNCT_RAMP);}			
	
			// Custom data

				/// The value for x=0;
	void	Set_y0   (double m_y0)   {y0 = m_y0;};	
	double	Get_y0 ()  {return y0;};

				/// The angular coefficient.
	void	Set_ang  (double m_ang)  {ang = m_ang;};
	double	Get_ang () {return ang;};


			// Override the Get_y(), Get_y_dx etc. functions with analytical formulas.

	double Get_y      (double x) {return (y0 + (x*ang))  ;};
	double Get_y_dx   (double x) {return (ang)			 ;};
	double Get_y_dxdx (double x) {return  0				 ;};

			// Expose the parameters which can be used for optimizations, as Javascript vars.

	OPT_VARIABLES_START
		"C",
		"ang",
	OPT_VARIABLES_END
	

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};



////////////////////////////////////////////
/// SINE FUNCTION:
/// y = sin (phase + w*x )     w=2*PI*freq

class ChApi ChFunction_Sine : public ChFunction
{
	CH_RTTI(ChFunction_Sine, ChFunction);
private:
	double amp;
	double phase;
	double freq;
	double w;
public:
	ChFunction_Sine () {phase =0; freq=1;  w=2*CH_C_PI*freq; amp = 1;};
	ChFunction_Sine (double m_phase, double m_freq, double m_amp)  {phase = m_phase; freq = m_freq; w=2*CH_C_PI*freq; amp = m_amp;};
	~ChFunction_Sine () {};
	void Copy (ChFunction_Sine* source);
	ChFunction* new_Duplicate ();

	void Set_phase (double m_phase) {phase = m_phase;};
	void Set_freq  (double m_freq)  {freq = m_freq; w=2*CH_C_PI*freq;};
	void Set_w	   (double m_w)		{w = m_w;		freq = w/(2*CH_C_PI);};
	void Set_amp   (double m_amp)   {amp = m_amp;}
	double Get_phase () {return phase;};
	double Get_freq ()  {return freq;};
	double Get_w	()  {return w;};
	double Get_amp ()   {return amp;}

	double Get_y      (double x) {return amp * (sin(phase + w*x)) ;};
	double Get_y_dx   (double x) {return amp * w*(cos(phase + w*x))	;};
	double Get_y_dxdx (double x) {return amp * -w*w*(sin(phase + w*x));};

	int Get_Type () {return (FUNCT_SINE);}

	OPT_VARIABLES_START
		"amp",
		"phase",
		"freq",
	OPT_VARIABLES_END

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};


////////////////////////////////////////////
/// SIGMA FUNCTION:           
/// y = polynomial smooth ramp 


class ChApi ChFunction_Sigma : public ChFunction
{
	CH_RTTI(ChFunction_Sigma, ChFunction);
private:
	double amp;
	double start;
	double end;
public:
	ChFunction_Sigma () {amp =1; start=0;  end=1;}
	ChFunction_Sigma (double m_amp, double m_start, double m_end)  {start = m_start; end = m_end; amp = m_amp;};
	~ChFunction_Sigma () {};
	void Copy (ChFunction_Sigma* source);
	ChFunction* new_Duplicate ();

	void Set_start (double m_start) {start = m_start;}
	void Set_end   (double m_end)   {end = m_end;}
	void Set_amp   (double m_amp)   {amp = m_amp;}
	double Get_start ()   {return start;}
	double Get_end ()   {return end;}
	double Get_amp ()   {return amp;}

	double Get_y      (double x) ;
	double Get_y_dx   (double x) ;
	double Get_y_dxdx (double x) ;

	double Get_Ca_pos () {return 6.0;};
	double Get_Ca_neg () {return 6.0;};
	double Get_Cv () {return 1.5;};

	void Extimate_x_range (double& xmin, double& xmax);
	int Get_Type () {return (FUNCT_SIGMA);}

	OPT_VARIABLES_START
		"start",
		"end",
		"amp",
	OPT_VARIABLES_END

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};


///////////////////////////////////////////
/// MOTION CAPTURE (SAMPLE) FUNCTION
/// y = (linear interpolated array of samples)

class ChApi ChFunction_Mocap : public ChFunction
{
	CH_RTTI(ChFunction_Mocap, ChFunction);
private:
	ChMatrix<>* array_y;
	ChMatrix<>* array_y_dt;
	ChMatrix<>* array_y_dtdt;

	double samp_freq;
	int    samples;
	double timetot;

public:
	ChFunction_Mocap (); //  see .cpp
	ChFunction_Mocap (int m_samples, double freq);  //  see .cpp
	~ChFunction_Mocap ();	//  see .cpp
	void Copy (ChFunction_Mocap* source);
	ChFunction* new_Duplicate ();

	void Set_samp_freq (double m_fr) ; // see .cpp
	void Set_samples (int m_samples) ; // see .cpp

	double Get_samp_freq () {return samp_freq;};
	int	   Get_samples	 () {return samples;};
	double Get_timetot	 () {return ((double)samples/samp_freq);};
	double Get_timeslice () {return (1/samp_freq);};

	ChMatrix<>* Get_array_y() {return array_y;};
	ChMatrix<>* Get_array_y_dt() {return array_y_dt;};
	ChMatrix<>* Get_array_y_dtdt() {return array_y_dtdt;};
	
	void Set_array_y	  (ChMatrix<>* m_array_y); // see cpp
	void Set_array_y_dt	  (ChMatrix<>* m_array_y_dt);  // *** TO DO
	void Set_array_y_dtdt (ChMatrix<>* m_array_y_dtdt);// *** TO DO

	int  Parse_array_AOA ();	// *** TO DO
	int	 Parse_array_Elite ();	// *** TO DO

	void   Compute_array_dt   (ChMatrix<>* array_A, ChMatrix<>* array_A_dt);
	double LinInterp (ChMatrix<>* m_array, double x, double x_max);

	double Get_y      (double x);	// see.cpp
	double Get_y_dx   (double x);	// see.cpp
	double Get_y_dxdx (double x);	// see.cpp

	void Extimate_x_range (double& xmin, double& xmax);
	int Get_Type () {return (FUNCT_MOCAP);}

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};



////////////////////////////////////////////
/// POLYNOMIAL FUNCTION:
/// y = a + bx + cx^2 + dx^3 + ...


class ChApi ChFunction_Poly : public ChFunction
{
	CH_RTTI(ChFunction_Poly, ChFunction);
private:
	double	coeff[POLY_COEFF_ARRAY]; // vector of coefficients
	int		order;					 // 0= const, 1= linear, etc... 
public:
	ChFunction_Poly ();
	~ChFunction_Poly () {};
	void Copy (ChFunction_Poly* source);
	ChFunction* new_Duplicate ();

	void Set_coeff (double m_coeff, int m_ind) {if (m_ind >= POLY_COEFF_ARRAY) {return;}; coeff[m_ind]= m_coeff;};
	void Set_order (int m_order)  {if (m_order >= POLY_COEFF_ARRAY) {m_order= (POLY_COEFF_ARRAY -1);} order= m_order;};
	double Get_coeff (int m_ind) {if (m_ind >= POLY_COEFF_ARRAY) {return 0;};  return coeff[m_ind];};
	int Get_order () {return order;};

	double Get_y      (double x);
	double Get_y_dx   (double x);
	double Get_y_dxdx (double x);

	int Get_Type () {return (FUNCT_POLY);}

	int MakeOptVariableTree(ChList<chjs_propdata>* mtree);

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};




////////////////////////////////////////////
/// CONSTANT ACCELERATION FUNCTION:  
/// h = height, amount of displacement
/// end = duration of motion,
/// av  = fraction of 1st acceleration end  (0..1)  
/// aw  = fraction of 2nd acceleration start (0..1) , with aw>av;

class ChApi ChFunction_ConstAcc : public ChFunction
{
	CH_RTTI(ChFunction_ConstAcc, ChFunction);
private:
	double h;
	double av;
	double aw;
	double end;
public:
	ChFunction_ConstAcc () {h =1; av=0.5; aw=0.5;  end=1;}
	ChFunction_ConstAcc (double m_h, double m_av, double m_aw, double m_end)  {h = m_h; Set_end(m_end); Set_avw(m_av,m_aw);};
	~ChFunction_ConstAcc () {};
	void Copy (ChFunction_ConstAcc* source);
	ChFunction* new_Duplicate ();

	void Set_end  (double m_end)  {if (m_end<0) m_end = 0; end = m_end;}
	void Set_av   (double m_av)   {if (m_av<0) m_av = 0; if (m_av>1) m_av = 1; av = m_av; if (av>aw) av = aw;}
	void Set_aw   (double m_aw)   {if (m_aw<0) m_aw = 0; if (m_aw>1) m_aw = 1; aw = m_aw; if (aw<av) aw = av;}
	void Set_h    (double m_h)    {h = m_h;}
	void Set_avw  (double m_av, double m_aw)	{av=0; aw=1; Set_av(m_av); Set_aw(m_aw);}

	double Get_end () {return end;}
	double Get_av ()  {return av;}
	double Get_aw ()  {return aw;}
	double Get_h ()   {return h;}

	double Get_y      (double x) ;
	double Get_y_dx   (double x) ;
	double Get_y_dxdx (double x) ;

	double Get_Ca_pos ();
	double Get_Ca_neg ();
	double Get_Cv ();

	void Extimate_x_range (double& xmin, double& xmax) {xmin = 0.0; xmax = end;};

	int Get_Type () {return (FUNCT_CONSTACC);}

	OPT_VARIABLES_START
		"h",
		"end",
		"aw",
		"av",
	OPT_VARIABLES_END

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};



////////////////////////////////////////////
/// A RAMP, AS 3-4-5 POLYNOMIAL FUNCTION:  
///   - h   = height, amount of displacement
///   - end = duration of motion,

class ChApi ChFunction_Poly345 : public ChFunction
{
	CH_RTTI(ChFunction_Poly345, ChFunction);
private:
	double h;
	double end;
public:
	ChFunction_Poly345 () {h =1;  end=1;}
	ChFunction_Poly345 (double m_h, double m_end)  {h = m_h; Set_end(m_end);};
	~ChFunction_Poly345 () {};
	void Copy (ChFunction_Poly345* source);
	ChFunction* new_Duplicate ();

	void Set_end  (double m_end)  {if (m_end<0) m_end = 0; end = m_end;}
	void Set_h    (double m_h)    {h = m_h;}

	double Get_end () {return end;}
	double Get_h ()   {return h;}

	double Get_y      (double x) ;
	double Get_y_dx   (double x) ;
	double Get_y_dxdx (double x) ;

	double Get_Ca_pos () {return 5.8;};
	double Get_Ca_neg () {return 5.8;};
	double Get_Cv () {return 1.9;};

	void Extimate_x_range (double& xmin, double& xmax) {xmin = 0.0; xmax = end;};

	int Get_Type () {return (FUNCT_POLY345);}
	
	OPT_VARIABLES_START
		"h",
		"end",
	OPT_VARIABLES_END

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};



////////////////////////////////////////////
/// A CUBIC FILLET (cubic poly with C0 C1 boundary conditions)  
///  - y1 = y at the beginning
///  - dy1 = y' at the beginning
///  - y2 = y at the end
///  - dy2 = y' at the end

class ChApi ChFunction_Fillet3 : public ChFunction
{
	CH_RTTI(ChFunction_Fillet, ChFunction);
private:
	double end;
	double y1;
	double y2;
	double dy1;
	double dy2;

	double c1, c2, c3, c4;	// used internally...

public:
	ChFunction_Fillet3 () {y1 = y2 = dy1 = dy2 = c1 = c2 = c3 = c4 = 0; end = 1.0;}
	~ChFunction_Fillet3 () {};
	void Copy (ChFunction_Fillet3* source);
	ChFunction* new_Duplicate ();

	void Set_end  (double m_end)  {if (m_end<0) m_end = 0; end = m_end; SetupCoefficients();}
	double Get_end () {return end;}

	int SetupCoefficients();

	void Set_y1(double my1) {y1 = my1; SetupCoefficients();}
	void Set_y2(double my2) {y2 = my2; SetupCoefficients();}
	void Set_dy1(double mdy1) {dy1 = mdy1; SetupCoefficients();}
	void Set_dy2(double mdy2) {dy2 = mdy2; SetupCoefficients();}

	double Get_y1() {return y1;}
	double Get_y2() {return y2;}
	double Get_dy1() {return dy1;}
	double Get_dy2() {return dy2;}	

	double Get_y      (double x) ;
	double Get_y_dx   (double x) ;
	double Get_y_dxdx (double x) ;

	void Extimate_x_range (double& xmin, double& xmax) {xmin = 0.0; xmax = end;};
	int Get_Type () {return (FUNCT_FILLET3);}

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};


////////////////////////////////////////////
/// OPERATION BETWEEN FUNCTIONS
/// (math operation between A and  B operands  
///   - fa = first operand function
///   - fb = second operand function

enum {
	ChOP_ADD = 0,
	ChOP_SUB,
	ChOP_MUL,
	ChOP_DIV,
	ChOP_POW,
	ChOP_MAX,
	ChOP_MIN,
	ChOP_MODULO,
	ChOP_FABS,
	ChOP_FUNCT,
};

class ChApi ChFunction_Operation : public ChFunction
{
	CH_RTTI(ChFunction_Operation, ChFunction);
private:
	ChFunction* fa;
	ChFunction* fb;
	int op_type;		// see operation type IDS

public:
	ChFunction_Operation() {op_type = ChOP_ADD; fa = new ChFunction; fb = new ChFunction; }
	~ChFunction_Operation () {if (fa) delete fa; if (fb) delete fb;};
	void Copy (ChFunction_Operation* source);
	ChFunction* new_Duplicate ();

	void Set_optype  (int m_op)  {op_type = m_op;}
	int Get_optype () {return op_type;}

	void Set_fa  (ChFunction* m_fa)  {fa = m_fa;}
	ChFunction* Get_fa () {return fa;}
	
	void Set_fb  (ChFunction* m_fb)  {fb = m_fb;}
	ChFunction* Get_fb () {return fb;}

	double Get_y      (double x) ;
	//	double Get_y_dx   (double x) ;
	//	double Get_y_dxdx (double x) ;

	void Extimate_x_range (double& xmin, double& xmax);

	int Get_Type () {return (FUNCT_OPERATION);}
	
	int MakeOptVariableTree(ChList<chjs_propdata>* mtree);
	
	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};



/////////////////////////////////////////////

class ChApi ChRecPoint 
{
public:
	double x;	
	double y;
	double w;  // weight
};

#define CH_RECORDER_EPSILON 1.e-10


/////////////////////////////////////////////
/// RECORDER FUNCTION
/// y = interpolation of array of (x,y) data, 
///     where (x,y) points can be inserted randomly.

class ChApi ChFunction_Recorder : public ChFunction
{
	CH_RTTI(ChFunction_Recorder, ChFunction);
private:
	ChList<ChRecPoint> points;		// the list of points
	ChNode<ChRecPoint>* lastnode;	// speed optimization: remember the last used pointer

public:
	ChFunction_Recorder () {lastnode = NULL;};
	~ChFunction_Recorder () {points.KillAll();};
	void Copy (ChFunction_Recorder* source);
	ChFunction* new_Duplicate ();

	int AddPoint (double mx, double my, double mw);
	int AddPoint (double mx, double my) {return AddPoint(mx,my,1.0);};
	int AddPointClean (double mx, double my, double dx_clean); // also clean nodes to the right, upt to dx interval
	void Reset() {points.KillAll(); lastnode = NULL;};

	ChList<ChRecPoint>*  GetPointList() {return &points;};

	double Get_y      (double x) ;
	double Get_y_dx   (double x) ;
	double Get_y_dxdx (double x) ;

	void Extimate_x_range (double& xmin, double& xmax);

	int Get_Type () {return (FUNCT_RECORDER);}

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};


/////////////////////////////////////////////
/// OSCILLOSCOPE FUNCTION
/// y = interpolation of array of (x,y) data, 
///     where (x,y) points must be inserted one
///     after the other, strictly with a fixed dx
///     interval. After a maximum amount of recordable
///     points is reached, the firsts are deleted. 
/// Note: differently from ChFunction_Recorder, this
/// 'basic' function does not allow not-uniform dx spacing
/// between points, but may be faster and simplier to
/// use in many cases.

class ChApi ChFunction_Oscilloscope : public ChFunction
{
	CH_RTTI(ChFunction_Oscilloscope, ChFunction);
private:
	std::list<double> values;
	double end_x;
	double dx;
	int max_amount;
	int amount;

public:
	ChFunction_Oscilloscope () {dx = 0.01; amount = 0; max_amount = 100; end_x=0;};
	~ChFunction_Oscilloscope () {};
	void Copy (ChFunction_Oscilloscope* source);
	ChFunction* new_Duplicate ();
		
		/// Add a point at the head (right side of point array).
		/// Note that it is user's responsability to add points
		/// which are spaced uniformily (by dx) on the X axis!
		/// No checks are done on the correctness of the dx spacing,
		/// except that if you enter a point whose mx is less than the
		/// mx of the one you previously entered, the array is cleared.
	int AddLastPoint (double mx, double my);

		/// Reset the array or recorded points.
	void Reset() {values.clear(); amount =0; end_x= 0;};

		/// Access directly the list of points.
	std::list<double>&  GetPointList() {return values;};

		/// Get the dx spacing between recorded points. It is assumed uniform!
	double Get_dx() {return dx;}
		/// Set the dx spacing between recorded points. It is assumed uniform!
	void Set_dx(double mdx) {dx = fabs(mdx);}

		/// Get the maximum amount of points which can be entered (after this,
		/// the first one will be deleted, as in a FIFO)
	int Get_max_amount() {return max_amount;}
		/// Set the maximum amount of points which can be entered (after this,
		/// the first one will be deleted, as in a FIFO)
	void Set_max_amount(int mnum) {if (mnum > 0) max_amount = mnum;}

		/// Get the amount of recorded points
	double Get_amount() {return amount;}

	double Get_y      (double x) ;
	//double Get_y_dx   (double x) ;
	//double Get_y_dxdx (double x) ;

	void Extimate_x_range (double& xmin, double& xmax);

	int Get_Type () {return (FUNCT_OSCILLOSCOPE);}

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};




////////////////////////////////////////////
// THE SEQUENCE FUNCTION:


/// Node for the list of functions 
/// in a ChFunction_Sequence object.

class ChApi ChFseqNode
{
public:
	ChFunction* fx;
	double duration;
	double weight;
	double t_start;
	double t_end;
	double Iy;
	double Iydt;
	double Iydtdt;
	bool y_cont;
	bool ydt_cont;
	bool ydtdt_cont;

	void SetDuration(double mdur);
	void SetTend(double mt_end) {t_end = mt_end; if (t_end < t_start) t_end= t_start; duration = t_end - t_start;};

	ChFseqNode(ChFunction* myfx, double mdur);
	~ChFseqNode();
	void Copy(ChFseqNode* source);

	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};

////////////////////////////////////////////
/// SEQUENCE FUNCTION:
///   y = sequence_of_functions(f1(y), f2(y), f3(y))  
/// All other function types can be inserted into this.
/// This function is very important because very complex motion
/// laws can be created by sequencing many basic ChFunctions.

class ChApi ChFunction_Sequence : public ChFunction
{
	CH_RTTI(ChFunction_Sequence, ChFunction);
private:
	ChList<ChFseqNode> functions;	// the list of sub functions
	double start;					// start time for sequence
public:
	ChFunction_Sequence();
	~ChFunction_Sequence();
	void Copy (ChFunction_Sequence* source);
	ChFunction* new_Duplicate ();
	
	int Get_Type () {return (FUNCT_SEQUENCE);}		


				/// The sequence of functions starts at this x value.
	void	Set_start  (double m_start)  {start = m_start;};
	double	Get_start  ()  {return start;};

				/// Access the list of the sub-functions.
	ChList<ChFseqNode>* Get_list() {return &functions;};

				/// Scans all the seq.of functions and setup the timings and continuity 
				/// offsets, to satisfy all constraints.
				/// This must be called whenever a new function is inserted, or its 
				/// timing and continuity constraints are changed.
	void Setup();

				/// Insert function after the fx with defined "position" index in list.
				///  - If index is higher than available objects, it simply goes to the end. 
				///  - If index = 0 insert at the beginning, 
				///  - If index = -1 insert at the end.
				/// Inserted functions will be deleted automatically when this object will be deleted.
				/// The fx segment has its own 'weight': use 1.0 for default, or different weights 
				/// if you want that Get_weight() will give different results depending on the "x" parameter.
				/// Set c0=true if you want to force C0 continuity with previous function (an offset
				/// will be implicitly added to the function, as y=f(x)+Offset). Same for C1 and C2 continuity,
				/// using c1 and c2 flags.
	int InsertFunct (ChFunction* myfx, double duration, double weight, bool c0, bool c1, bool c2, int position);

				/// Remove and deletes function with defined "position", and returns TRUE. 
				///	 - If position = 0, removes always head (beginning), 
				///  - If position = -1 removes tail (end). 
				///  - If position > max number of current nodes, removes tail anyway, but returns NULL.
	int KillFunct (int position);

				/// Returns the ChFunction with given "position". 
				///  - If position = 0, returns always head (beginning), 
				///  - If position = -1 returns tail (end). 
				///  - If position > max number of current nodes, returns tail fx anyway.
	ChFunction* GetNthFunction (int position);		

				/// As above, but returns the function node (containing function pointer, 
				/// function duration, continuity flags with previous node, etc.) 
	ChFseqNode* GetNthNode(int position);

				/// As above, but returning duration. (return value is reference, so it 
				/// can be also changed later, but remember Setup() for the 
				/// ChFunction_Sequence after you modified this return value by reference ***TO DO***). 
				/// If no function, returns 0.
	double GetNthDuration(int position);

		
	double Get_y      (double x);
	double Get_y_dx   (double x);
	double Get_y_dxdx (double x);

	double Get_weight (double x);

	void Extimate_x_range (double& xmin, double& xmax);


	int MakeOptVariableTree(ChList<chjs_propdata>* mtree);

	int HandleNumber();			
	int HandleAccess(int handle_id, double mx, double my, bool set_mode);

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};





#define CHF_MATLAB_STRING_LEN 200

////////////////////////////////////////////
/// MATLAB FUNCTION:
/// y = matlab evaluation of function y=f(x)  
/// 

class ChApi ChFunction_Matlab : public ChFunction
{
	CH_RTTI(ChFunction_Matlab, ChFunction);
private:
	char mat_command[CHF_MATLAB_STRING_LEN];			// matlab command
public:
	ChFunction_Matlab();
	~ChFunction_Matlab() {};
	void Copy (ChFunction_Matlab* source);
	ChFunction* new_Duplicate ();

	void Set_Command  (char* m_command)  {strcpy (mat_command, m_command);};
	char* Get_Command  ()  {return mat_command;};

	double Get_y      (double x);
	double Get_y_dx   (double x) {return ((Get_y(x+ BDF_STEP_HIGH) - Get_y(x)) / BDF_STEP_HIGH); }//((Get_y(x+BDF_STEP_VERYLOW) - Get_y(x)) / BDF_STEP_VERYLOW);};  // return 0;
	double Get_y_dxdx (double x) {return ((Get_y_dx(x+BDF_STEP_HIGH) - Get_y_dx(x)) / BDF_STEP_HIGH);}; // return 0;

	int Get_Type () {return (FUNCT_MATLAB);}

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};



////////////////////////////////////////////
/// NOISE FUNCTION:
/// y = multi-octave noise with cubic interpolation  
/// 

class ChApi ChFunction_Noise : public ChFunction
{
	CH_RTTI(ChFunction_Noise, ChFunction);
private:
	double amp;
	double freq;
	double amp_ratio;
	int octaves;
public:
	ChFunction_Noise();
	~ChFunction_Noise() {};
	void Copy (ChFunction_Noise* source);
	ChFunction* new_Duplicate ();

	void Set_Amp (double mamp) {amp = mamp;} 
	double Get_Amp ()  {return amp;};
	void Set_Freq (double mf) {freq = mf;} 
	double Get_Freq ()  {return freq;};
	void Set_AmpRatio (double ma) {amp_ratio = ma;} 
	double Get_AmpRatio ()  {return amp_ratio;};
	void Set_Octaves (int mo) {octaves = mo;} 
	int Get_Octaves ()  {return octaves;};

	double Get_y      (double x);

	int Get_Type () {return (FUNCT_NOISE);}

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};


////////////////////////////////////////////
/// DERIVATIVE OF A FUNCTION:
///  y = df/dx  
///
/// Uses a numerical differentiation method to compute the derivative
/// of a generic function.

class ChApi ChFunction_Derive : public ChFunction
{
	CH_RTTI(ChFunction_Derive, ChFunction);
private:
	ChFunction* fa;
	int order;			// 1= derive one time, 2= two times, etc.

public:
	ChFunction_Derive() {order = 1; fa = new ChFunction;}
	~ChFunction_Derive () {if (fa) delete fa;};
	void Copy (ChFunction_Derive* source);
	ChFunction* new_Duplicate ();

	void Set_order  (int m_order)  {order = m_order;}
	int Get_order () {return order;}

	void Set_fa  (ChFunction* m_fa)  {fa = m_fa;}
	ChFunction* Get_fa () {return fa;}
	
	double Get_y      (double x) ;

	void Extimate_x_range (double& xmin, double& xmax);
	
	int Get_Type () {return (FUNCT_DERIVE);}

	int MakeOptVariableTree(ChList<chjs_propdata>* mtree);

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};


////////////////////////////////////////////
/// INTEGRAL OF A FUNCTION:
/// y = int{ f(x) dx  
///
/// Uses a numerical quadrature method to compute the definite integral. 

class ChApi ChFunction_Integrate : public ChFunction
{
	CH_RTTI(ChFunction_Integrate, ChFunction);
private:
	ChFunction* fa;
	int order;			// 1= Integrate one time, 2= two times, etc.
	double C_start;
	double x_start;
	double x_end;
	int num_samples;
	ChMatrix<>* array_x;
public:
	ChFunction_Integrate();
	~ChFunction_Integrate () {if (fa) delete fa; if (array_x) delete array_x;};
	void Copy (ChFunction_Integrate* source);
	ChFunction* new_Duplicate ();

	void ComputeIntegral();
	
	void Set_order  (int m_order)  {order = m_order;}
	int Get_order () {return order;}
	void Set_num_samples  (int m_samples)  {num_samples = m_samples; array_x->Reset(num_samples,1);  ComputeIntegral();}
	int Get_num_samples() {return num_samples;}
	void Set_C_start  (double m_val)  {C_start = m_val; ComputeIntegral();}
	double Get_C_start () {return C_start;}
	void Set_x_start  (double m_val)  {x_start = m_val; ComputeIntegral();}
	double Get_x_start () {return x_start;}
	void Set_x_end  (double m_val)  {x_end = m_val; ComputeIntegral();}
	double Get_x_end () {return x_end;}
	void Set_fa  (ChFunction* m_fa)  {fa = m_fa; ComputeIntegral();}
	ChFunction* Get_fa () {return fa;}

	double Get_y      (double x) ;

	void Extimate_x_range (double& xmin, double& xmax);
	
	int Get_Type () {return (FUNCT_INTEGRATE);}

	int MakeOptVariableTree(ChList<chjs_propdata>* mtree);
	OPT_VARIABLES_START
		"C_start",
	OPT_VARIABLES_END

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};


////////////////////////////////////////////
/// MIRROR FUNCTION:
/// y = __/\__ 
///
/// Mirrors a function about a vertical axis.

class ChApi ChFunction_Mirror : public ChFunction
{
	CH_RTTI(ChFunction_Mirror, ChFunction);
private:
	ChFunction* fa;
	double mirror_axis;			// simmetry axis position on x

public:
	ChFunction_Mirror() {mirror_axis = 0; fa = new ChFunction;}
	~ChFunction_Mirror () {if (fa) delete fa;};
	void Copy (ChFunction_Mirror* source);
	ChFunction* new_Duplicate ();

	void Set_mirror_axis  (double m_axis)  {mirror_axis = m_axis;}
	double Get_mirror_axis () {return mirror_axis;}

	void Set_fa  (ChFunction* m_fa)  {fa = m_fa;}
	ChFunction* Get_fa () {return fa;}
	
	double Get_y      (double x) ;

	void Extimate_x_range (double& xmin, double& xmax);
	int Get_Type () {return (FUNCT_MIRROR);}

	int MakeOptVariableTree(ChList<chjs_propdata>* mtree);
	OPT_VARIABLES_START
		"mirror_axis",
	OPT_VARIABLES_END

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};


////////////////////////////////////////////
/// REPEAT FUNCTION:
/// y = __/__/__/ 
///
/// Repeats a 'window' of a function, periodically.

class ChApi ChFunction_Repeat : public ChFunction
{
	CH_RTTI(ChFunction_Repeat, ChFunction);
private:
	ChFunction* fa;
	double window_start;			// window begin position
	double window_length;			// window length
public:
	ChFunction_Repeat() {window_start=0;window_length=1;  fa = new ChFunction;}
	~ChFunction_Repeat () {if (fa) delete fa;};
	void Copy (ChFunction_Repeat* source);
	ChFunction* new_Duplicate ();

	void Set_window_start  (double m_v)  {window_start = m_v;}
	double Get_window_start () {return window_start;}
	void Set_window_length  (double m_v)  {window_length = m_v;}
	double Get_window_length () {return window_length;}

	void Set_fa  (ChFunction* m_fa)  {fa = m_fa;}
	ChFunction* Get_fa () {return fa;}
	
	double Get_y      (double x) ;

	void Extimate_x_range (double& xmin, double& xmax);
	int Get_Type () {return (FUNCT_REPEAT);}

	int MakeOptVariableTree(ChList<chjs_propdata>* mtree);
	OPT_VARIABLES_START
		"window_start",
		"window_length",
	OPT_VARIABLES_END

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};




} // END_OF_NAMESPACE____


#endif
