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
// Authors: Radu Serban, Alessandro Tasora
// =============================================================================
//
// Physical system in which contact is modeled using a smooth (penalty-based)
// method.
//
// =============================================================================

#include <limits>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChContactContainerSMC.h"

#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/collision/ChCollisionSystemBullet.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSystemSMC)

ChSystemSMC::ChSystemSMC(bool use_material_properties)
    : ChSystem(),
      m_use_mat_props(use_material_properties),
      m_contact_model(Hertz),
      m_adhesion_model(AdhesionForceModel::Constant),
      m_tdispl_model(OneStep),
      m_stiff_contact(false) {
    descriptor = chrono_types::make_shared<ChSystemDescriptor>();

    SetSolverType(ChSolver::Type::PSOR);

    collision_system = chrono_types::make_shared<collision::ChCollisionSystemBullet>();

    // For default SMC there is no need to create contacts 'in advance'
    // when models are closer than the safety envelope, so set default envelope to 0
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0);

    contact_container = chrono_types::make_shared<ChContactContainerSMC>();
    contact_container->SetSystem(this);

    m_minSlipVelocity = 1e-4;
    m_characteristicVelocity = 1;
}

ChSystemSMC::ChSystemSMC(const ChSystemSMC& other) : ChSystem(other) {}

void ChSystemSMC::SetContactContainer(std::shared_ptr<ChContactContainer> container) {
    if (std::dynamic_pointer_cast<ChContactContainerSMC>(container))
        ChSystem::SetContactContainer(container);
}

void ChSystemSMC::SetSlipVelocityThreshold(double vel) {
    m_minSlipVelocity = std::max(vel, std::numeric_limits<double>::epsilon());
}

// STREAMING - FILE HANDLING

// Trick to avoid putting the following mapper macro inside the class definition in .h file:
// enclose macros in local 'my_enum_mappers', just to avoid avoiding cluttering of the parent class.
class my_enum_mappers : public ChSystemSMC {
  public:
    CH_ENUM_MAPPER_BEGIN(ContactForceModel);
    CH_ENUM_VAL(Hooke);
    CH_ENUM_VAL(Hertz);
    CH_ENUM_VAL(PlainCoulomb);
    CH_ENUM_VAL(Flores)
    CH_ENUM_MAPPER_END(ContactForceModel);

    CH_ENUM_MAPPER_BEGIN(AdhesionForceModel);
    CH_ENUM_VAL(AdhesionForceModel::Constant);
    CH_ENUM_VAL(AdhesionForceModel::DMT);
    CH_ENUM_VAL(AdhesionForceModel::Perko);
    CH_ENUM_MAPPER_END(AdhesionForceModel);

    CH_ENUM_MAPPER_BEGIN(TangentialDisplacementModel);
    CH_ENUM_VAL(None);
    CH_ENUM_VAL(OneStep);
    CH_ENUM_VAL(MultiStep);
    CH_ENUM_MAPPER_END(TangentialDisplacementModel);
};

void ChSystemSMC::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChSystemSMC>();

    // serialize parent class
    ChSystem::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(m_use_mat_props);
    marchive << CHNVP(m_minSlipVelocity);
    marchive << CHNVP(m_characteristicVelocity);
    my_enum_mappers::ContactForceModel_mapper mmodel_mapper;
    marchive << CHNVP(mmodel_mapper(m_contact_model), "contact_model");
    my_enum_mappers::AdhesionForceModel_mapper madhesion_mapper;
    marchive << CHNVP(madhesion_mapper(m_adhesion_model), "adhesion_model");
    my_enum_mappers::TangentialDisplacementModel_mapper mtangential_mapper;
    marchive << CHNVP(mtangential_mapper(m_tdispl_model), "tangential_model");
    //***TODO*** complete...
}

/// Method to allow de serialization of transient data from archives.
void ChSystemSMC::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChSystemSMC>();

    // deserialize parent class
    ChSystem::ArchiveIN(marchive);

    // stream in all member data:
    marchive >> CHNVP(m_use_mat_props);
    marchive >> CHNVP(m_minSlipVelocity);
    marchive >> CHNVP(m_characteristicVelocity);
    my_enum_mappers::ContactForceModel_mapper mmodel_mapper;
    marchive >> CHNVP(mmodel_mapper(m_contact_model), "contact_model");
    my_enum_mappers::AdhesionForceModel_mapper madhesion_mapper;
    marchive >> CHNVP(madhesion_mapper(m_adhesion_model), "adhesion_model");
    my_enum_mappers::TangentialDisplacementModel_mapper mtangential_mapper;
    marchive >> CHNVP(mtangential_mapper(m_tdispl_model), "tangential_model");
    //***TODO*** complete...

    // Recompute statistics, offsets, etc.
    this->Setup();
}

}  // end namespace chrono
