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
//   ChLinkNumdiff.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChLinkNumdiff.h"


#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{






// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkNumdiff> a_registration_ChLinkNumdiff;


            // BUILD
ChLinkNumdiff::ChLinkNumdiff ()
{
}

            // DESTROY
ChLinkNumdiff::~ChLinkNumdiff ()
{
}


void ChLinkNumdiff::Copy(ChLinkNumdiff* source)
{
    // first copy the parent class data...
    ChLinkMasked::Copy(source);

}


ChLink* ChLinkNumdiff::new_Duplicate ()   // inherited classes:  Link* MyInheritedLink::new_Duplicate()
{
    ChLinkNumdiff* m_l;
    m_l = new ChLinkNumdiff;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}



////////////////////////////////////
///
///    UPDATING PROCEDURES





 
/////////       COMPUTE C    -updates the value of C residuals, given current coordinates & time
/////////                     C = C(q,t)

void ChLinkNumdiff::ComputeC ()
{
    // ** Child classes must implement this constraint evaluation, in order
    //    to allow this base class to perform backward differentiation to get
    //    all state matrices.
    //    This function can be called many times by UpdateState() for the numerical
    //    differentiation.
    //
    //    Example: C = my_constraint_functionXXX(distance_between_markers,time), etc.
    //
    //    Default: do nothing (residual always zero)

    C->Reset();     // default: always no violation. C = 0;
}


/////////     COMPUTE Ct
/////////


void ChLinkNumdiff::ComputeCt ()
{
    ChMatrixDynamic<> m_q(GetNumCoords(), 1);       // coordinates
    FetchCoords(&m_q);          // current state coordinates = position of bodies 1&2

    double m_tdt = ChTime + BDF_STEP_VERYLOW;

    ChMatrixDynamic<> C_qt(ndoc,1);            //  C(q,t)      // assuming C is already updated (computed)!!
    C_qt.CopyFromMatrix(*this->C);

    ChMatrixDynamic<> C_qdt(ndoc,1);           //  C(q,t+dt)
    ImposeCoords(&m_q, m_tdt);
    ComputeC();
    C_qdt.CopyFromMatrix(*this->C);

    Ct->MatrSub(C_qdt, C_qt);         // Ct
    Ct->MatrScale(1.0/BDF_STEP_VERYLOW);
}


/////////    COMPUTE Cq
/////////

void ChLinkNumdiff::ComputeCq ()
{
    double orig;

    ChMatrixDynamic<> m_q(GetNumCoords(), 1);       // current coordinates
    FetchCoords(&m_q);              // current state coordinates = position of bodies 1&2

    ChMatrixDynamic<> C_dqt(ndoc,1);
    ChMatrixDynamic<> Cq_column(ndoc, 1);
    ChMatrixDynamic<> C_qt(ndoc,1);            //  C(q,t)      // assuming C is already updated (computed)!!
    C_qt.CopyFromMatrix(*this->C);

    for (int i= 0; i< GetNumCoords(); i++)
    {
        orig = m_q(i,0);
        m_q(i,0) = orig + BDF_STEP_VERYLOW;
        ImposeCoords(&m_q, ChTime);
        ComputeC();
        C_dqt.CopyFromMatrix(*this->C);      // C(dqi,t)

        Cq_column.MatrSub(C_dqt, C_qt);
        Cq_column.MatrScale(1.0/BDF_STEP_VERYLOW);

        if (i<BODY_QDOF)
            this->Cq1->PasteMatrix(&Cq_column,0,i);             // jacobians [Cq], 1&2
        else
            this->Cq2->PasteMatrix(&Cq_column,0,(i-BODY_QDOF));
        m_q(i,0) = orig;
    }
}


/////////     UPDATE STATE
/////////


void ChLinkNumdiff::UpdateState ()
{
    double m_t = ChTime;
    double m_tdt = ChTime + BDF_STEP_VERYLOW;

    ChMatrixDynamic<> m_q(GetNumCoords(), 1);   // The coordinates q
    FetchCoords(&m_q);      // current state coordinates = position of bodies 1&2

    ChMatrixDynamic<> m_qdt(GetNumCoords(), 1); // The coordinates q
    FetchCoords_dt(&m_qdt);     // current state coordinates = position of bodies 1&2

    ChMatrixDynamic<> C_qt(ndoc,1);
    ComputeC();              //  C(q,t)  (assuming system is now at (q,t) )
    C_qt.CopyFromMatrix(*this->C);

    ComputeCt();             //  Ct

    ComputeCq();             //  Cq1 and Cq2


    ChMatrixDynamic<> m_qdt1(BODY_QDOF,1);
    ChMatrixDynamic<> m_qdt2(BODY_QDOF,1);
    m_qdt1.PasteClippedMatrix(&m_qdt, 0,0, BODY_QDOF,1, 0,0);
    m_qdt2.PasteClippedMatrix(&m_qdt, BODY_QDOF,0, BODY_QDOF,1, 0,0);

    ChMatrixDynamic<> m_temp(ndoc,1);

    m_temp.MatrMultiply(*Cq1, m_qdt1);
    C_dt->CopyFromMatrix(m_temp);
    m_temp.MatrMultiply(*Cq2, m_qdt2);
    C_dt->MatrInc(m_temp);
    C_dt->MatrInc(*Ct);      // C_dt  = Ct + [Cq](dq/dt);

        // now compute vector gamma for dynamics:  ???
    //***TO DO***....
}








/////////    -   SET COORDINATES of the two connected bodies
/////////

void ChLinkNumdiff::ImposeCoords(ChMatrix<>* mc, double t)
{
    ChCoordsys<> mcsys;

	if (!(dynamic_cast<ChBody*>(Body1) &&
		  dynamic_cast<ChBody*>(Body2)) )
		throw (ChException("ChLinkNumdiff had pointer to non ChBody item"));

    mcsys = mc->ClipCoordsys(0,0);
    dynamic_cast<ChBody*>(Body1)->Update(mcsys, Body1->GetCoord_dt(), t);

    mcsys = mc->ClipCoordsys(7,0);
    dynamic_cast<ChBody*>(Body2)->Update(mcsys, Body2->GetCoord_dt(), t);

    // - update the time dependant stuff
    UpdateTime(t);
    // - update the relative marker position
    UpdateRelMarkerCoords();
}

void ChLinkNumdiff::FetchCoords(ChMatrix<>* mc)
{
    static Coordsys mcsys;

    mcsys = Body1->GetCoord();
    mc->PasteCoordsys(mcsys,0,0);

    mcsys = Body2->GetCoord();
    mc->PasteCoordsys(mcsys,7,0);
}

void ChLinkNumdiff::FetchCoords_dt(ChMatrix<>* mc)
{
    static Coordsys mcsys;

    mcsys = Body1->GetCoord_dt();
    mc->PasteCoordsys(mcsys,0,0);

    mcsys = Body2->GetCoord_dt();
    mc->PasteCoordsys(mcsys,7,0);
}



/////////
///////// FILE I/O
/////////




void ChLinkNumdiff::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(10);
		// serialize parent class too
	ChLinkMasked::StreamOUT(mstream);

		// stream out all member data

}

void ChLinkNumdiff::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChLinkMasked::StreamIN(mstream);

		// stream in all member data

}














 
} // END_OF_NAMESPACE____


