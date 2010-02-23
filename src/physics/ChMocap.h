#ifndef CHMOCAP_H
#define CHMOCAP_H

//////////////////////////////////////////////////
//  
//   ChMocap.h
//
//   load, save, and filters the files of 
//   motion capture (mocap) of different formats.
//   time history data.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChMath.h"

namespace chrono 
{


//////////////////////////////////////
///
/// Class for parsing motion-capture files (mocap)
/// of different formats.

class ChMocap
{
private:
	Vector** vstream;
	int samples;
	int markers;
	double Hertz;
	
	int undersample;
	double ioscale;
	int format;

public:
	ChMocap();
	~ChMocap();

	//void Copy(ChMocap* source);

	int Alloc_buffers(int msamples, int mmarkers);
	void Reset();

	int Get_samples() {return samples;}
	int Get_markers() {return markers;}

	Vector** Get_streams() {return vstream;}
	Vector* Get_mstream(int mark) {return *(vstream+mark);}

	Vector Get_mpos (int mark, int samp);
	void Set_mpos (int mark, int samp, Vector vect);
	
	double Get_Hertz() {return Hertz;}
	void Set_Hertz(double hz) {Hertz = hz;}
	double Get_ioscale() {return ioscale;}
	void Set_ioscale(double mio) {ioscale = mio;}
	int Get_undersample() {return undersample;}
	void Set_undersample(int mus) {if (mus>=1) undersample = mus;}

	int Get_format() {return format;}
	void Set_format(int mformat) {format = mformat;}

	// Data elaboration

	void Fast_smooth (double strenght);
	void Lin_interpolate (int from, int to, Vector va, Vector vb, int nmark);
	void Gaps_fill (int nmark);
	void Gaps_fill_all();
	void Rescale();

	// File I/O (return 0 if success, otherwise errorcode)

	int Parse (char* mfile);	// use the specified format 
	int Parse_AOA(char* mfile);
	int Parse_Elite(char* mfile);
	int Parse_R3Curve (char* mfile);
	int Parse_Chstream (char* mfile);

	int Write (char* mfile);	// use the specified format 
	int Write_AOA(char* mfile);
	int Write_Elite(char* mfile);
	int Write_R3Curve (char* mfile);
	int Write_Chstream (char* mfile);
};

//
// Define identifiers for for  ChMocap.format 
//

#define MOCF_ELITE		0
#define MOCF_AOA		1
#define MOCF_R3CURVE	2
#define MOCF_CHSTREAM   3
#define MOCF_ANNO		4

// to mark the gaps in streams
#define VGAP_MOCAP  222222


} // END_OF_NAMESPACE____

#endif
