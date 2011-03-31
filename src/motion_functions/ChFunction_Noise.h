#ifndef CHFUNCT_NOISE_H
#define CHFUNCT_NOISE_H

//////////////////////////////////////////////////
//  
//   ChFunction_Noise.h
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

#define FUNCT_NOISE		15


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


} // END_OF_NAMESPACE____


#endif
