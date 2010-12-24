#ifndef CHLINKFORCE_H
#define CHLINKFORCE_H

//////////////////////////////////////////////////
//  
//   ChLinkForce.h
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include <math.h>
#include <float.h>

#include "core/ChMath.h"
#include "physics/ChFunction.h"


namespace chrono 
{


///
/// Class for forces in link joints of type ChLink().
///

class ChApi ChLinkForce
{
private:
	int active;		// true/false

	double iforce;		// impressed force
	ChFunction* modul_iforce;	// time-modulation of imp. force

	double K;			// stiffness of the dof
	ChFunction* modul_K;  // modulation of K along the dof coord

	double R;			// damping of the dof
	ChFunction* modul_R;  // modulation of R along the dof coord


public:
	ChLinkForce();
	~ChLinkForce();
	void Copy(ChLinkForce* source);
	ChLinkForce* new_Duplicate();

	int Get_active () {return active;}
	void Set_active (int m_a) {active = m_a;}

	double Get_iforce () {return iforce;}
	void Set_iforce (double m_f) {iforce = m_f;}

	double Get_K () {return K;}
	void Set_K (double m_K) {K = m_K;}

	double Get_R () {return R;}
	void Set_R (double m_R) {R = m_R;}

	ChFunction* Get_modul_iforce() {return modul_iforce;};
	ChFunction* Get_modul_K() {return modul_K;};
	ChFunction* Get_modul_R() {return modul_R;};

	void Set_modul_iforce(ChFunction* m_funct);
	void Set_modul_K		(ChFunction* m_funct);
	void Set_modul_R		(ChFunction* m_funct);

	double Get_Kcurrent (double x, double x_dt, double t);
	double Get_Rcurrent (double x, double x_dt, double t);
	double Get_iFcurrent (double x, double x_dt, double t);	

	// this is the most important function: it is called to evaluate
	// the internal force at instant t, position x and speed x_dt
	double Get_Force (double x, double x_dt, double t);	

			//
			// STREAMING
			//

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream);

					/// Method to allow serialization of transient data in ascii,
					/// as a readable item, for example   "chrono::GetLog() << myobject;"
	void StreamOUT(ChStreamOutAscii& mstream);


};


} // END_OF_NAMESPACE____

#endif

