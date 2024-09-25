// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban, Rainer Gericke
// =============================================================================

#include <cmath>

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsTorqueConverter.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsTorqueConverter)

ChShaftsTorqueConverter::ChShaftsTorqueConverter()
    : shaft1(NULL),
      shaft2(NULL),
      shaft_stator(NULL),
      torque_in(0),
      torque_out(0),
      state_warning_reverseflow(false),
      state_warning_wrongimpellerdirection(false) {
    K = chrono_types::make_shared<ChFunctionConst>(0.9);
    T = chrono_types::make_shared<ChFunctionConst>(0.9);
}

ChShaftsTorqueConverter::ChShaftsTorqueConverter(const ChShaftsTorqueConverter& other) : ChPhysicsItem(other) {
    shaft1 = NULL;
    shaft2 = NULL;
    shaft_stator = NULL;

    torque_in = other.torque_in;
    torque_out = other.torque_out;

    state_warning_reverseflow = other.state_warning_reverseflow;
    state_warning_wrongimpellerdirection = other.state_warning_wrongimpellerdirection;

    K = std::shared_ptr<ChFunction>(other.K->Clone());  // deep copy
    T = std::shared_ptr<ChFunction>(other.T->Clone());  // deep copy
}

bool ChShaftsTorqueConverter::Initialize(std::shared_ptr<ChShaft> shaft_1,  // input shaft
                                         std::shared_ptr<ChShaft> shaft_2,  // output shaft
                                         std::shared_ptr<ChShaft> shaft_st  // stator shaft (often fixed)
) {
    shaft1 = shaft_1.get();
    shaft2 = shaft_2.get();
    shaft_stator = shaft_st.get();

    assert(shaft1 && shaft2 && shaft_stator);
    assert((shaft1 != shaft2) && (shaft1 != shaft_stator));
    assert((shaft1->GetSystem() == shaft2->GetSystem()) && (shaft1->GetSystem() == shaft_stator->GetSystem()));

    SetSystem(shaft1->GetSystem());

    return true;
}

double ChShaftsTorqueConverter::GetSpeedRatio() const {
    double wrel1 = shaft1->GetPosDt() - shaft_stator->GetPosDt();
    double wrel2 = shaft2->GetPosDt() - shaft_stator->GetPosDt();

    if ((fabs(wrel1) < 10e-9) || (fabs(wrel2) < 10e-9))
        return 0;

    return wrel2 / wrel1;
}

void ChShaftsTorqueConverter::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data

    state_warning_wrongimpellerdirection = false;
    state_warning_reverseflow = false;

    // Compute actual speed ratio
    double mR = GetSpeedRatio();

    // The speed ratio must always be in the [0...1] range,
    // anyway let's correct singular cases:

    // - it should be unusual that speed ratio >1, say if the impeller
    //   outruns turbine (if there's a clutch lock-in, this should happen).
    //   If so, assume a reflection of T curve and a polar reflection of K curve, after 1.
    if (mR > 1) {
        mR = 1 - (mR - 1);
        state_warning_reverseflow = true;
    }

    // - if the output shaft is spinning a bit backward, when
    //   close to stall, maybe win/wout < 0. If so, set as stall anyway:
    if (mR < 0)
        mR = 0;

    // - if input impeller shaft is spinning in negative direction,
    //   this is assumed as an error: set all torques to zero and bail out:
    if (shaft1->GetPosDt() - shaft_stator->GetPosDt() < 0) {
        state_warning_wrongimpellerdirection = true;
        torque_in = 0;
        torque_out = 0;
        return;
    }

    // Compute actual capacity factor
    double mK = K->GetVal(mR);

    // Compute actual torque factor
    double mT = T->GetVal(mR);

    // compute input torque (with minus sign because applied TO input thaft)
    torque_in = -std::pow((shaft1->GetPosDt() / mK), 2);

    if (state_warning_reverseflow)
        torque_in = -torque_in;

    // compute output torque (with opposite sign because
    // applied to output shaft, with same direction of input shaft)
    if (state_warning_reverseflow) {
        // in reverse flow situation the converter is always in clutch mode (TR=1)
        // so the torque cannot be increased
        torque_out = -torque_in;

    } else {
        torque_out = -mT * torque_in;
    }
}

//// STATE BOOKKEEPING FUNCTIONS

void ChShaftsTorqueConverter::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                                ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                                const double c           // a scaling factor
) {
    if (shaft1->IsActive())
        R(shaft1->GetOffset_w()) += torque_in * c;
    if (shaft2->IsActive())
        R(shaft2->GetOffset_w()) += torque_out * c;
    if (shaft_stator->IsActive())
        R(shaft_stator->GetOffset_w()) += GetTorqueReactionOnStator() * c;
}

void ChShaftsTorqueConverter::VariablesFbLoadForces(double factor) {
    // Apply torques to the three connected 1D variables:
    shaft1->Variables().Force()(0) += torque_in * factor;
    shaft2->Variables().Force()(0) += torque_out * factor;
    shaft_stator->Variables().Force()(0) += GetTorqueReactionOnStator() * factor;
}

void ChShaftsTorqueConverter::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftsTorqueConverter>();

    // serialize parent class
    ChPhysicsItem::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(K);
    archive_out << CHNVP(T);
    archive_out << CHNVP(shaft1);        //// TODO  serialize with shared ptr
    archive_out << CHNVP(shaft2);        //// TODO  serialize with shared ptr
    archive_out << CHNVP(shaft_stator);  //// TODO  serialize with shared ptr
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsTorqueConverter::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftsTorqueConverter>();

    // deserialize parent class:
    ChPhysicsItem::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(K);
    archive_in >> CHNVP(T);
    archive_in >> CHNVP(shaft1);        //// TODO  serialize with shared ptr
    archive_in >> CHNVP(shaft2);        //// TODO  serialize with shared ptr
    archive_in >> CHNVP(shaft_stator);  //// TODO  serialize with shared ptr
}

}  // end namespace chrono