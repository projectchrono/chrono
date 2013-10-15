//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLINKLIMIT_H
#define CHLINKLIMIT_H

//////////////////////////////////////////////////
//  
//   ChLimit.h
//
//   Limit for links (costraints) coordinates.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>
#include <float.h>

#include "core/ChMath.h"
#include "physics/ChFunction.h"
#include "physics/ChLinkMask.h"
#include "lcp/ChLcpConstraintTwoBodies.h"

namespace chrono 
{





/// Class for limits in link joints (for example 
/// limits on elbow or knee rotations, etc.)
/// Old code: Must be improved..

class ChApi ChLinkLimit
{
private:
	int active;		// true/false
	int penalty_only;
	int polar;
	int rotation;
	double max;
	double min;
	double maxCushion;
	double minCushion;
	double Kmax;
	double Kmin;
	double Rmax;
	double Rmin;
	double maxElastic;
	double minElastic;
	ChFunction* modul_Kmax;
	ChFunction* modul_Kmin;
	ChFunction* modul_Rmax;
	ChFunction* modul_Rmin;
	ChFunction* polar_Max;

public:
	ChLcpConstraintTwoBodies constr_upper;
	ChLcpConstraintTwoBodies constr_lower;

	ChLinkLimit();
	~ChLinkLimit();
	void Copy(ChLinkLimit* source);
	ChLinkLimit* new_Duplicate();

	int	Get_active() {return active;}
	int	Get_penalty() {return penalty_only;}
	int Get_polar() {return polar;}
	int Get_rotation() {return rotation;}
	double Get_max() {return max;}
	double Get_min() {return min;}
	double Get_maxCushion() {return maxCushion;}
	double Get_minCushion() {return minCushion;}
	double Get_Kmax() {return Kmax;}
	double Get_Kmin() {return Kmin;}
	double Get_Rmax() {return Rmax;}
	double Get_Rmin() {return Rmin;}
	double Get_maxElastic() {return maxElastic;}
	double Get_minElastic() {return minElastic;}
	ChFunction* GetModul_Kmax() {return modul_Kmax;};
	ChFunction* GetModul_Kmin() {return modul_Kmin;};
	ChFunction* GetModul_Rmax() {return modul_Rmax;};
	ChFunction* GetModul_Rmin() {return modul_Rmin;};
	ChFunction* GetPolar_Max()  {return polar_Max;};
	double Get_polar_max(double pol_ang);

	void Set_active(int m_active) {active = m_active;}
	void Set_penalty(int m_active) {penalty_only = m_active;}
	void Set_polar(int m_pol) {polar = m_pol;}
	void Set_rotation(int m_rot) {rotation = m_rot;}
	void Set_max(double m_max);
	void Set_min(double m_min);
	void Set_maxCushion(double m_maxCushion);
	void Set_minCushion(double m_minCushion);
	void Set_Kmax(double m_K) {Kmax = m_K;}
	void Set_Kmin(double m_K) {Kmin = m_K;}
	void Set_Rmax(double m_R) {Rmax = m_R;}
	void Set_Rmin(double m_R) {Rmin = m_R;}
	void Set_maxElastic(double m_e) {maxElastic = m_e;}
	void Set_minElastic(double m_e) {minElastic = m_e;}
	void SetModul_Kmax	(ChFunction* m_funct);
	void SetModul_Kmin	(ChFunction* m_funct);
	void SetModul_Rmax	(ChFunction* m_funct);
	void SetModul_Rmin	(ChFunction* m_funct);
	void SetPolar_Max	(ChFunction* m_funct);

	double GetViolation (double x);  // return negative violation when x<min, or positive if x>max;
	double GetForce (double x, double x_dt); 

	double GetPolarForce(double x, double x_dt, double pol_ang);


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
