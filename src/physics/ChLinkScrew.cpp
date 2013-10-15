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

///////////////////////////////////////////////////
//
//   ChLinkScrew.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

  
#include "physics/ChLinkScrew.h"
#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{
 

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//
//   CLASS FOR SCREW
//
//

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkScrew> a_registration_ChLinkScrew;


ChLinkScrew::ChLinkScrew ()
{
    type = LNK_SCREW;       // initializes type

    //tau = 0.01;
    Set_thread(0.05);

            // Mask: initialize our LinkMaskLF (lock formulation mask)
            // to X,Y,Z,Rx Ry, (note: the Z lock is'nt a standard LinkLock z-lock and will
            // be handled as a custom screw constraint z = tau *alpha, later in updating functions).
    ((ChLinkMaskLF*)mask)->SetLockMask(true, true, true,
                       false, true, true, false,
                       false, false);
    ChangedLinkMask();
}

ChLinkScrew::~ChLinkScrew ()
{ }

void ChLinkScrew::Copy(ChLinkScrew* source)
{
    // first copy the parent class data...
    //
    ChLinkLock::Copy(source);

    // copy custom data:
    tau = source->tau;
}

ChLink* ChLinkScrew::new_Duplicate ()
{
    ChLinkScrew* m_l;
    m_l = new ChLinkScrew;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}


void ChLinkScrew::UpdateState()
{
    // First, compute everything as it were a normal "revolute" joint, on z axis...
    ChLinkLock::UpdateState();

    // Then, MODIFY the Z part of equations, such that the Z = 0  becomes Z = tau * alpha
    double scr_C, scr_C_dt, scr_C_dtdt;
    double scr_Ct, scr_Qc;
    double coeffa, coeffb, zangle, msign;
    ChMatrixNM<double,1,7> scr_Cq1;
    ChMatrixNM<double,1,7> scr_Cq2;
    double Crz;

    if (fabs (relC.rot.e0) <0.707)
    {
        Crz = relC.rot.e0;  // cos(alpha/2)
        msign = +1;
        zangle = acos (Crz);
        if (relC.rot.e3 < 0)
        {
            zangle = - zangle; // a/2 = -acos(Crz);
            msign = -1;
        }
        double mrelz = relC.pos.z;

        scr_C = mrelz - tau * 2.0 * zangle;
                                                // modulus correction..
        scr_C = scr_C - Get_thread()*floor(scr_C / Get_thread());
        double shiftedC = scr_C - Get_thread()*ceil(scr_C / Get_thread());
        if (fabs(scr_C)>fabs(shiftedC)) scr_C = shiftedC;

        coeffa = + 2.0 * tau * msign * 1/(sqrt(1 - pow (Crz,2.0) ) );
        coeffb = + 2.0 * tau * msign * Crz/(pow  ((1 - pow (Crz,2)), 3.0/2.0 ) );

        scr_C_dt = relC_dt.pos.z + relC_dt.rot.e0 * coeffa;
        scr_C_dtdt = relC_dtdt.pos.z + relC_dt.rot.e0 * coeffb + relC_dtdt.rot.e0 * coeffa;
        scr_Ct = Ct_temp.pos.z + coeffa * Ct_temp.rot.e0;
        scr_Qc = Qc_temp->GetElement(2,0) + coeffa * Qc_temp->GetElement(3,0) - relC_dt.rot.e0 * coeffb;
        scr_Cq1.Reset();
        scr_Cq2.Reset();
        scr_Cq1.PasteClippedMatrix(Cq1_temp, 3,3, 1,4, 0,3);
        scr_Cq2.PasteClippedMatrix(Cq2_temp, 3,3, 1,4, 0,3);
        scr_Cq1.MatrScale(coeffa);
        scr_Cq2.MatrScale(coeffa);
    }
    else
    {
        Crz = relC.rot.e3;  // Zz*sin(alpha/2)
        msign = +1;
        zangle = asin (Crz);
        if (relC.rot.e0 < 0)
        {
            zangle  = CH_C_PI - zangle;
            msign = -1;
        }
        double mrelz = relC.pos.z;//fmod (relC.pos.z , (tau * 2 * CH_C_PI));

        scr_C = mrelz - tau * 2.0 * zangle;
                                                        // modulus correction..
        scr_C = scr_C - Get_thread()*floor(scr_C / Get_thread());
        double shiftedC = scr_C - Get_thread()*ceil(scr_C / Get_thread());
        if (fabs(scr_C)>fabs(shiftedC)) scr_C = shiftedC;

        coeffa = - 2.0 * tau * msign * 1 /(sqrt(1 - pow (Crz,2.0) ) );
        coeffb = - 2.0 * tau * msign * Crz /(pow  ((1 - pow (Crz,2)), 3.0/2.0 ) );

        scr_C_dt = relC_dt.pos.z + relC_dt.rot.e3 * coeffa;
        scr_C_dtdt = relC_dtdt.pos.z + relC_dt.rot.e3 * coeffb + relC_dtdt.rot.e3 * coeffa;
        scr_Ct = Ct_temp.pos.z + coeffa * Ct_temp.rot.e3;
        scr_Qc = Qc_temp->GetElement(2,0) + coeffa * Qc_temp->GetElement(6,0) - relC_dt.rot.e3 * coeffb;
        scr_Cq1.Reset();
        scr_Cq2.Reset();
        scr_Cq1.PasteClippedMatrix(Cq1_temp, 6,3, 1,4, 0,3);
        scr_Cq2.PasteClippedMatrix(Cq2_temp, 6,3, 1,4, 0,3);
        scr_Cq1.MatrScale(coeffa);
        scr_Cq2.MatrScale(coeffa);
    }

    Cq1->PasteClippedMatrix(Cq1_temp, 2,0,  1,7,  2,0);
    Cq2->PasteClippedMatrix(Cq2_temp, 2,0,  1,7,  2,0);
    Cq1->PasteSumMatrix(&scr_Cq1, 2,0);
    Cq2->PasteSumMatrix(&scr_Cq2, 2,0);
    Qc->SetElement(2,0,scr_Qc);
    C->SetElement(2,0, scr_C);
    C_dt->SetElement(2,0, scr_C_dt);
    C_dtdt->SetElement(2,0, scr_C_dtdt);
    Ct->SetElement(2,0, scr_Ct);
}

void ChLinkScrew::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChLinkLock::StreamOUT(mstream);

		// stream out all member data
	mstream << tau;
}

void ChLinkScrew::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChLinkLock::StreamIN(mstream);

		// stream in all member data
	mstream >> tau;
}





///////////////////////////////////////////////////////////////



} // END_OF_NAMESPACE____


