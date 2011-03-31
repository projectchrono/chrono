#ifndef CHFUNCT_OSCILLOSCOPE_H
#define CHFUNCT_OSCILLOSCOPE_H

//////////////////////////////////////////////////
//  
//   ChFunction_Oscilloscope.h
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


#include "ChFunction_Base.h"


namespace chrono 
{

#define FUNCT_OSCILLOSCOPE	20


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





} // END_OF_NAMESPACE____


#endif
