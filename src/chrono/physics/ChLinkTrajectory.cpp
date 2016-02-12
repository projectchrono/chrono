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
//   ChLinkTrajectory.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChLinkTrajectory.h"
#include "geometry/ChCLineSegment.h"

namespace chrono {

using namespace geometry;

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkTrajectory> a_registration_ChLinkTrajectory;

// BUILDERS
ChLinkTrajectory::ChLinkTrajectory() {
    type = LNK_TRAJECTORY;  // initializes type

    // default s(t) function. User will provide better fx.
    space_fx = std::make_shared<ChFunction_Ramp>(0, 1.);

    // default trajectory is a segment
    trajectory_line = std::make_shared<ChLineSegment>();

    modulo_s = false;

    // Mask: initialize our LinkMaskLF (lock formulation mask)
    // to X  only. It was a LinkMaskLF because this class inherited from LinkLock.
    ((ChLinkMaskLF*)mask)->SetLockMask(true, true, true, false, false, false, false);

    ChangedLinkMask();
}

// DESTROYER
ChLinkTrajectory::~ChLinkTrajectory() {
    // nothing..
}

void ChLinkTrajectory::Copy(ChLinkTrajectory* source) {
    // first copy the parent class data...
    //
    ChLinkLock::Copy(source);

    // copy own data

    space_fx = std::shared_ptr<ChFunction>(source->space_fx->new_Duplicate());  // deep copy

    trajectory_line = std::shared_ptr<ChLine>((ChLine*)source->trajectory_line->Duplicate());  // deep copy
}

ChLink* ChLinkTrajectory::new_Duplicate() {
    ChLinkTrajectory* m_l;
    m_l = new ChLinkTrajectory;
    m_l->Copy(this);
    return (m_l);
}

void ChLinkTrajectory::Set_space_fx(std::shared_ptr<ChFunction> m_funct) {
    space_fx = m_funct;
}

void ChLinkTrajectory::Set_trajectory_line(std::shared_ptr<geometry::ChLine> mline) {
    trajectory_line = mline;
}

/////////    UPDATE TIME
/////////

void ChLinkTrajectory::UpdateTime(double time) {
    ChTime = time;

    double tstep = BDF_STEP_HIGH;
    double tr_time = space_fx->Get_y(time);
    double tr_timeB = space_fx->Get_y(time + tstep);
    double tr_timeA = space_fx->Get_y(time - tstep);

    if (trajectory_line) {
        Vector result, resultB, resultA;
        if (modulo_s) {
            tr_time = fmod(tr_time, 1);
            tr_timeA = fmod(tr_timeA, 1);
            tr_timeB = fmod(tr_timeB, 1);
        }
        trajectory_line->Evaluate(result, tr_time);
        trajectory_line->Evaluate(resultA, tr_timeA);
        trajectory_line->Evaluate(resultB, tr_timeB);

        ChMatrix33<> mw;
        mw.Set_A_quaternion(marker2->GetAbsCoord().rot);

        // if line coordinate is relative to body2:
        marker2->Impose_Rel_Coord(CSYSNORM);
        deltaC.pos = result;
        deltaC_dt.pos = (resultB - resultA) * 1 / (2 * tstep);
        deltaC_dtdt.pos = (resultA + resultB - result * 2) * 4 / pow(2 * tstep, 2);
        /*
        // if line coordinate is relative to absolute space:
        deltaC.pos = mw.MatrT_x_Vect(
                            Vsub (result, marker2->GetAbsCoord().pos));  // ***  CORRECT?
        deltaC_dt.pos =  mw.MatrT_x_Vect(
                            Vmul( Vsub(resultB, resultA), 1/(2*tstep)) );
        deltaC_dtdt.pos =  mw.MatrT_x_Vect (
                            Vmul   ( Vadd (Vadd (resultA, resultB),
                                   Vmul (result,-2)), 4/pow(2*tstep, 2) ) );
        */
        deltaC.rot = QUNIT;
        deltaC_dt.rot = QNULL;
        deltaC_dtdt.rot = QNULL;
    } else {
        GetLog() << "NO TRAJECTORY \n";
    }
}

void ChLinkTrajectory::Initialize(
    std::shared_ptr<ChBody> mbody1,  ///< first  body to join (the one that follows the trajectory)
    std::shared_ptr<ChBody> mbody2,  ///< second body to join (the one that contains the trajectory)
    const ChVector<>& mpos1,     ///< position of the 'following point' on body1, relative to coordinate of body1.
    std::shared_ptr<geometry::ChLine> mline  ///< the line on mbody2 to be followed by point mpos1 of mbody1
    ) {
    ChLinkMarkers::Initialize(mbody1, mbody2, true, ChCoordsys<>(mpos1), ChCoordsys<>());
    this->Set_trajectory_line(mline);
}

void ChLinkTrajectory::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChLinkLock::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(space_fx);
    marchive << CHNVP(trajectory_line);
    marchive << CHNVP(modulo_s);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkTrajectory::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class
    ChLinkLock::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(space_fx);
    marchive >> CHNVP(trajectory_line);
    marchive >> CHNVP(modulo_s);
}



///////////////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____
