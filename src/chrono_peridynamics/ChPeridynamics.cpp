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
// Authors: Alessandro Tasora,
// =============================================================================

#include "chrono/core/ChRandom.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_peridynamics/ChPeridynamics.h"
#include "chrono_peridynamics/ChMatterPeridynamics.h"

namespace chrono {

using namespace fea;
using namespace peridynamics;

static double W_sph(double r, double h) {
    if (r < h) {
        return (315.0 / (64.0 * CH_PI * pow(h, 9))) * pow((h * h - r * r), 3);
    } else
        return 0;
}

static double W_sq_visco(double r, double h) {
    if (r < h) {
        return (45.0 / (CH_PI * pow(h, 6))) * (h - r);
    } else
        return 0;
}

// ------------------------------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChPeridynamics)

ChPeridynamics::ChPeridynamics() : n_added(0), is_updated(true), n_dofs(0), n_constr(0) {}

ChPeridynamics::ChPeridynamics(const ChPeridynamics& other) : ChProximityContainer(other) {
    vnodes = other.vnodes;
    n_added = other.n_added;
    is_updated = other.is_updated;
    materials = other.materials;
    n_dofs = other.n_dofs;
    n_constr = other.n_constr;
}

ChPeridynamics::~ChPeridynamics() {}

void ChPeridynamics::RemoveAllProximities() {
    // nothing to do: proximities are handled by materials in this->materials
    n_added = 0;
}

void ChPeridynamics::BeginAddProximities() {
    // nothing to do: proximities are handled by materials in this->materials

    n_added = 0;
}

void ChPeridynamics::EndAddProximities() {
    is_updated = false;
    // nothing to do: proximities are handled by materials in this->materials
}

void ChPeridynamics::AddProximity(ChCollisionModel* modA, ChCollisionModel* modB) {
    // Fetch the frames of that proximity and other infos

    ChNodePeri* mnA = dynamic_cast<ChNodePeri*>(modA->GetContactable());
    ChNodePeri* mnB = dynamic_cast<ChNodePeri*>(modB->GetContactable());

    if (!(mnA && mnB))
        return;

    // Report the proximity to material(s). They will manage it, for example
    // turning the proximity into a peridynamics bond and storing it in material data structures, etc.
    for (auto& mymat : this->materials) {
        mymat->AddProximity(mnA, mnB);
    }

    // Launch the proximity callback, if implemented by the user

    if (this->add_proximity_callback) {
        this->add_proximity_callback->OnAddProximity(*modA, *modB);
    }

    n_added++;
}

void ChPeridynamics::ReportAllProximities(ReportProximityCallback* mcallback) {
    // TODO: proximities are handled by materials in this->materials
}

void ChPeridynamics::SetupInitial() {
    n_dofs = 0;
    n_constr = 0;

    for (unsigned int i = 0; i < vnodes.size(); i++) {
        if (!vnodes[i]->IsFixed()) {
            vnodes[i]->SetupInitial(GetSystem());

            // count the degrees of freedom
            n_dofs += vnodes[i]->GetNumCoordsPosLevelActive();
        }
    }
    for (auto& mat : this->materials)
        mat->SetupInitial();
}

void ChPeridynamics::SetupInitialBonds(ChSystem* sys, std::shared_ptr<ChPeridynamics> peri) {
    sys->Setup();
    sys->Update(true);
    sys->ComputeCollisions();
    for (auto& node : peri->GetNodes())
        node->is_requiring_bonds = false;
    peri->SetupInitial();
}

void ChPeridynamics::Setup() {
    n_dofs = 0;
    n_constr = 0;

    for (unsigned int i = 0; i < vnodes.size(); i++) {
        // Set node offsets in state vectors (based on the offsets of the containing mesh)
        vnodes[i]->NodeSetOffsetPosLevel(GetOffset_x() + n_dofs);

        // Count the actual degrees of freedom (consider only nodes that are not fixed)
        if (!vnodes[i]->IsFixed()) {
            n_dofs += vnodes[i]->GetNumCoordsPosLevelActive();
        }
    }
    for (auto& mat : this->materials) {
        mat->Setup();
        n_constr += mat->GetNumConstraints();
    }
}

void ChPeridynamics::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChProximityContainer::Update(mytime, update_assets);

    if (!is_updated)  // avoids overhead of Update()
    {
        // tentatively mark as updated
        is_updated = true;
    }
}

void ChPeridynamics::AddMatter(std::shared_ptr<ChMatterPeriBase> mmatter) {
    materials.push_back(mmatter);
    mmatter->SetContainer(this);
}

void ChPeridynamics::AddNode(std::shared_ptr<ChNodePeri> m_node) {
    vnodes.push_back(m_node);

    // If the obj is is already added to a system, mark the system uninitialized and out-of-date
    /*
    if (system) {
        system->is_initialized = false;
        system->is_updated = false;
    }
    */
}

void ChPeridynamics::Fill(
    std::shared_ptr<ChMatterPeriBase>
        mmatter,  ///< matter to be used for this volume. Must be added too to this, via AddMatter().
    std::vector<ChVector3d>& points,  ///< points obtained from a sampler of the volume, ex. use utils::Sampler
    const double spacing,             ///< average spacing used in sampling
    const double mass,                ///< total mass of the volume
    const double volume,              ///< total volume
    const double horizon_sfactor,     ///< the radius of horizon of the particle is 'spacing' multiplied this value
    const double collision_sfactor,
    const ChCoordsys<> mcoords  ///< position and rotation of the volume
) {
    auto nsamples = points.size();
    double per_node_mass = mass / nsamples;
    double per_node_volume = volume / nsamples;
    double horizon = horizon_sfactor * spacing;
    double collrad = collision_sfactor * spacing;

    for (const auto& mpos_loc : points) {
        ChVector3d mpos = mcoords.TransformPointLocalToParent(mpos_loc);
        auto mnode = chrono_types::make_shared<ChNodePeri>();
        mnode->SetX0(mpos);
        mnode->SetPos(mpos);
        mnode->SetMass(per_node_mass);
        mnode->volume = per_node_volume;
        mnode->is_fluid = false;
        mnode->coll_rad = collrad;
        mnode->h_rad = horizon;

        // Add to this
        this->AddNode(mnode);
        mmatter->AddNode(mnode);
    }
}

void ChPeridynamics::FillBox(
    std::shared_ptr<ChMatterPeriBase>
        mmatter,                     ///< matter to be used for this volume. Must be added too to this, via AddMatter().
    const ChVector3d size,           ///< x,y,z sizes of the box to fill (better if integer multiples of spacing)
    const double spacing,            ///< the spacing between two near nodes
    const double initial_density,    ///< density of the material inside the box, for initialization of node's masses
    const ChCoordsys<> boxcoords,    ///< position and rotation of the box
    const double horizon_sfactor,    ///< the radius of horizon of the particle is 'spacing' multiplied this value
    const double collision_sfactor,  ///< the radius of collision shape (sphere) of the particle is 'spacing' multiplied
                                     ///< this value
    const double randomness          ///< randomness of the initial distribution lattice, 0...1
) {
    int samples_x = (int)(size.x() / spacing);
    int samples_y = (int)(size.y() / spacing);
    int samples_z = (int)(size.z() / spacing);
    int totsamples = samples_x * samples_y * samples_z;

    double mrandomness = randomness;

    double horizon = horizon_sfactor * spacing;
    double collrad = collision_sfactor * spacing;
    double mtotvol = size.x() * size.y() * size.z();
    double mtotmass = mtotvol * initial_density;
    double nodemass = mtotmass / (double)totsamples;
    double nodevol = mtotvol / (double)totsamples;

    for (int ix = 0; ix < samples_x; ix++)
        for (int iy = 0; iy < samples_y; iy++)
            for (int iz = 0; iz < samples_z; iz++) {
                ChVector3d pos(ix * spacing + 0.5 * spacing - 0.5 * size.x(),
                               iy * spacing + 0.5 * spacing - 0.5 * size.y(),
                               iz * spacing + 0.5 * spacing - 0.5 * size.z());
                pos += ChVector3d(mrandomness * ChRandom::Get() * spacing, mrandomness * ChRandom::Get() * spacing,
                                  mrandomness * ChRandom::Get() * spacing);
                ChVector3d mpos = boxcoords.TransformPointLocalToParent(pos);
                auto mnode = chrono_types::make_shared<ChNodePeri>();
                mnode->SetX0(mpos);
                mnode->SetPos(mpos);
                mnode->SetMass(nodemass);
                mnode->volume = nodevol;
                mnode->is_fluid = false;
                mnode->coll_rad = collrad;
                mnode->h_rad = horizon;
                mnode->vol_size = spacing;
                this->AddNode(mnode);
                mmatter->AddNode(mnode);
                if ((ix == 0) || (ix == samples_x - 1) || (iy == 0) || (iy == samples_y - 1) || (iz == 0) ||
                    (iz == samples_z - 1)) {
                    mnode->is_boundary = true;
                }
            }
}

void ChPeridynamics::FillBox(
    std::vector<std::pair<std::shared_ptr<ChMatterPeriBase>, double>>
        v_mmatter,  ///< {matters,x_thickness} pairs to be used for layers. Must be added too to this, via AddMatter().
    const ChVector3d size,           ///< x,y,z sizes of the box to fill (better if integer multiples of spacing)
    const double spacing,            ///< the spacing between two near nodes
    const double initial_density,    ///< density of the material inside the box, for initialization of node's masses
    const ChCoordsys<> boxcoords,    ///< position and rotation of the box
    const double horizon_sfactor,    ///< the radius of horizon of the particle is 'spacing' multiplied this value
    const double collision_sfactor,  ///< the radius of collision shape (sphere) of the particle is 'spacing' multiplied
                                     ///< this value
    const double randomness)         ///< randomness of the initial distribution lattice, 0...1
{
    int samples_x = (int)(size.x() / spacing);
    int samples_y = (int)(size.y() / spacing);
    int samples_z = (int)(size.z() / spacing);
    int totsamples = samples_x * samples_y * samples_z;

    double mrandomness = randomness;

    double horizon = horizon_sfactor * spacing;
    double collrad = collision_sfactor * spacing;
    double mtotvol = size.x() * size.y() * size.z();
    double mtotmass = mtotvol * initial_density;
    double nodemass = mtotmass / (double)totsamples;
    double nodevol = mtotvol / (double)totsamples;

    std::shared_ptr<ChMatterPeriBase> A_matter = v_mmatter[0].first;
    std::shared_ptr<ChMatterPeriBase> B_matter = nullptr;
    ////double curr_thickness = v_mmatter[0].second;
    std::vector<double> interfaces(v_mmatter.size());
    for (int imat = 0; imat < v_mmatter.size(); ++imat) {
        if (imat > 0)
            interfaces[imat] = v_mmatter[imat - 1].second;
        interfaces[imat] += v_mmatter[imat].second;
    }

    for (int ix = 0; ix < samples_x; ix++) {
        double x_lay1 = 0;
        double x_lay2 = 0;
        double xmin = ix * spacing;
        double xmax = (ix + 1) * spacing;
        for (int iint = 0; iint < interfaces.size(); ++iint) {
            B_matter = nullptr;
            x_lay2 = interfaces[iint];
            if (xmin >= x_lay1 && xmin < x_lay2) {
                A_matter = v_mmatter[iint].first;
                if (xmax >= interfaces[iint]) {
                    // std::cout << " INTERFACE at  int << " << iint << "  ix=" << ix << "   xmin " << xmin << "  xmax"
                    // << xmax<< "\n";
                    B_matter = v_mmatter[iint + 1].first;  // node cell overlaps interface
                }
                break;
            }
            x_lay1 = interfaces[iint];
        }

        for (int iy = 0; iy < samples_y; iy++)
            for (int iz = 0; iz < samples_z; iz++) {
                ChVector3d pos(ix * spacing + 0.5 * spacing - 0.5 * size.x(),
                               iy * spacing + 0.5 * spacing - 0.5 * size.y(),
                               iz * spacing + 0.5 * spacing - 0.5 * size.z());
                pos += ChVector3d(mrandomness * ChRandom::Get() * spacing, mrandomness * ChRandom::Get() * spacing,
                                  mrandomness * ChRandom::Get() * spacing);
                ChVector3d mpos = boxcoords.TransformPointLocalToParent(pos);
                auto mnode = chrono_types::make_shared<ChNodePeri>();
                mnode->SetX0(mpos);
                mnode->SetPos(mpos);
                mnode->SetMass(nodemass);
                mnode->is_fluid = false;
                mnode->coll_rad = collrad;
                mnode->h_rad = horizon;
                mnode->volume = nodevol;
                mnode->vol_size = spacing;
                if (B_matter) {
                    // node is at interface, shared betwen two matters. Correct volume:
                    mnode->volume = 0.5 * nodevol;
                    mnode->vol_size = spacing * 0.5;  // ?
                }
                this->AddNode(mnode);
                if (A_matter)
                    A_matter->AddNode(mnode);
                if (B_matter)
                    B_matter->AddNode(mnode);
                if ((ix == 0) || (ix == samples_x - 1) || (iy == 0) || (iy == samples_y - 1) || (iz == 0) ||
                    (iz == samples_z - 1)) {
                    mnode->is_boundary = true;
                }
            }
    }
}

void ChPeridynamics::FillSphere(
    std::shared_ptr<ChMatterPeriBase>
        mmatter,                    ///< matter to be used for the sphere. Must be added too to this, via AddMatter().
    const double sphere_radius,     ///< radius of sphere to fill
    const double spacing,           ///< the spacing between two near nodes (grid interval h)
    const double initial_density,   ///< density of the material inside the box, for initialization of node's masses
    const ChCoordsys<> boxcoords,   ///< position and rotation of the box
    const double horizon_sfactor,   ///< the radius of horizon of the particle is 'spacing' multiplied this value
    const double collision_sfactor  ///< the radius of collision shape (sphere) of the particle is 'spacing' multiplied
                                    ///< this value
) {
    double size_x = sphere_radius * 2;
    int samples_x = (int)(size_x / spacing);
    int samples_y = (int)(size_x / spacing);
    int samples_z = (int)(size_x / spacing);

    int totsamples = 0;
    for (int ix = 0; ix < samples_x; ix++)
        for (int iy = 0; iy < samples_y; iy++)
            for (int iz = 0; iz < samples_z; iz++) {
                ChVector3d pos(ix * spacing + 0.5 * spacing - 0.5 * size_x, iy * spacing + 0.5 * spacing - 0.5 * size_x,
                               iz * spacing + 0.5 * spacing - 0.5 * size_x);
                if (pos.Length() < sphere_radius)
                    totsamples++;
            }

    double horizon = horizon_sfactor * spacing;
    double collrad = collision_sfactor * spacing;
    double mtotvol = 4. / 3. * CH_PI * pow(sphere_radius, 3);
    double mtotmass = mtotvol * initial_density;
    double nodemass = mtotmass / (double)totsamples;
    double nodevol = mtotvol / (double)totsamples;

    for (int ix = 0; ix < samples_x; ix++)
        for (int iy = 0; iy < samples_y; iy++)
            for (int iz = 0; iz < samples_z; iz++) {
                ChVector3d pos(ix * spacing + 0.5 * spacing - 0.5 * size_x, iy * spacing + 0.5 * spacing - 0.5 * size_x,
                               iz * spacing + 0.5 * spacing - 0.5 * size_x);
                double pos_rad = pos.Length();
                if (pos_rad < sphere_radius) {
                    ChVector3d mpos = boxcoords.TransformPointLocalToParent(pos);
                    auto mnode = chrono_types::make_shared<ChNodePeri>();
                    mnode->SetX0(mpos);
                    mnode->SetPos(mpos);
                    mnode->SetMass(nodemass);
                    mnode->volume = nodevol;
                    mnode->is_fluid = false;
                    mnode->coll_rad = collrad;
                    mnode->h_rad = horizon;
                    mnode->vol_size = spacing;
                    this->AddNode(mnode);
                    mmatter->AddNode(mnode);
                    if (fabs(pos_rad - sphere_radius) < 0.866025 * spacing) {
                        mnode->is_boundary = true;
                    }
                }
            }
}

void ChPeridynamics::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChPeridynamics>();
    // serialize parent class
    ChProximityContainer::ArchiveOut(marchive);
    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChPeridynamics::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChPeridynamics>();
    // deserialize parent class
    ChProximityContainer::ArchiveIn(marchive);
    // stream in all member data:
}

// -----------------------------------------------------------------------

void ChPeridynamics::IntStateGather(const unsigned int off_x,
                                    ChState& x,
                                    const unsigned int off_v,
                                    ChStateDelta& v,
                                    double& T) {
    unsigned int local_off = 0;
    for (auto& node : vnodes) {
        if (!node->IsFixed()) {
            node->NodeIntStateGather(off_x + local_off, x, off_v + local_off, v, T);
            local_off += 3;
        }
    }
    T = GetChTime();
}

void ChPeridynamics::IntStateScatter(const unsigned int off_x,
                                     const ChState& x,
                                     const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const double T,
                                     bool full_update) {
    unsigned int local_off = 0;
    for (auto& node : vnodes) {
        if (!node->IsFixed()) {
            node->NodeIntStateScatter(off_x + local_off, x, off_v + local_off, v, T);
            local_off += 3;
        }
    }
    Update(T, full_update);
}

void ChPeridynamics::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    unsigned int local_off = 0;
    for (auto& node : vnodes) {
        if (!node->IsFixed()) {
            node->NodeIntStateGatherAcceleration(off_a + local_off, a);
            local_off += 3;
        }
    }
}
void ChPeridynamics::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    unsigned int local_off = 0;
    for (auto& node : vnodes) {
        if (node->IsFixed()) {
            node->NodeIntStateScatterAcceleration(off_a + local_off, a);
            local_off += 3;
        }
    }
}
void ChPeridynamics::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    // RESET FORCES ACCUMULATORS CAUSED BY PERIDYNAMIC MATERIALS!
    // Each node will be reset  as  F=0
    for (auto& mymat : this->materials) {
        mymat->ComputeForcesReset();
    }

    // COMPUTE FORCES CAUSED BY PERIDYNAMIC MATERIALS!
    // Accumulate forces at each node, as  F+=...
    // This can be a time consuming phase, depending on the amount of nodes in each material.
    for (auto& mymat : this->materials) {
        mymat->ComputeForces();
    }

    unsigned int local_off = 0;
    for (auto& node : vnodes) {
        if (!node->IsFixed()) {
            // add gravity too
            ChVector3d Gforce = GetSystem()->GetGravitationalAcceleration() * node->GetMass();
            ChVector3d TotForce = (node->F_peridyn) + node->GetForce() + Gforce;

            R.segment(off + local_off, 3) += c * TotForce.eigen();
            local_off += 3;
        }
    }
}
void ChPeridynamics::IntLoadResidual_Mv(const unsigned int off,
                                        ChVectorDynamic<>& R,
                                        const ChVectorDynamic<>& w,
                                        const double c) {
    unsigned int local_off = 0;
    for (auto& node : vnodes) {
        if (!node->IsFixed()) {
            node->NodeIntLoadResidual_Mv(off + local_off, R, w, c);
            local_off += 3;
        }
    }
}

void ChPeridynamics::IntLoadLumpedMass_Md(const unsigned int off, ChVectorDynamic<>& Md, double& err, const double c) {
    unsigned int local_off = 0;
    for (auto& node : vnodes) {
        if (node->IsFixed()) {
            node->NodeIntLoadLumpedMass_Md(off + local_off, Md, err, c);
            local_off += 3;
        }
    }
}

void ChPeridynamics::IntLoadResidual_CqL(const unsigned int off_L,
                                         ChVectorDynamic<>& R,
                                         const ChVectorDynamic<>& L,
                                         const double c) {
    unsigned int local_off = 0;
    for (auto& mymat : this->materials) {
        mymat->IntLoadResidual_CqL(off_L + local_off, R, L, c);
        local_off += mymat->GetNumConstraints();
    }
}

void ChPeridynamics::IntLoadConstraint_C(const unsigned int off,
                                         ChVectorDynamic<>& Qc,
                                         const double c,
                                         bool do_clamp,
                                         double recovery_clamp) {
    unsigned int local_off = 0;
    for (auto& mymat : this->materials) {
        mymat->IntLoadConstraint_C(off + local_off, Qc, c, do_clamp, recovery_clamp);
        local_off += mymat->GetNumConstraints();
    }
}

void ChPeridynamics::InjectConstraints(ChSystemDescriptor& descriptor) {
    for (auto& mymat : this->materials) {
        mymat->InjectConstraints(descriptor);
    }
}

void ChPeridynamics::LoadConstraintJacobians() {
    for (auto& mymat : this->materials) {
        mymat->LoadConstraintJacobians();
    }
}

void ChPeridynamics::IntToDescriptor(const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const ChVectorDynamic<>& R,
                                     const unsigned int off_L,
                                     const ChVectorDynamic<>& L,
                                     const ChVectorDynamic<>& Qc) {
    unsigned int local_off = 0;
    for (auto& node : vnodes) {
        if (!node->IsFixed()) {
            node->NodeIntToDescriptor(off_v + local_off, v, R);
            local_off += 3;
        }
    }
    unsigned int local_Loff = 0;
    for (auto& mymat : this->materials) {
        mymat->IntToDescriptor(off_v + local_off, v, R, off_L + local_Loff, L, Qc);
        local_Loff += mymat->GetNumConstraints();
    }
}

void ChPeridynamics::IntFromDescriptor(const unsigned int off_v,
                                       ChStateDelta& v,
                                       const unsigned int off_L,
                                       ChVectorDynamic<>& L) {
    unsigned int local_off = 0;
    for (auto& node : vnodes) {
        if (!node->IsFixed()) {
            node->NodeIntFromDescriptor(off_v + local_off, v);
            local_off += 3;
        }
    }
    unsigned int local_Loff = 0;
    for (auto& mymat : this->materials) {
        mymat->IntFromDescriptor(off_v + local_off, v, off_L + local_Loff, L);
        local_Loff += mymat->GetNumConstraints();
    }
}

// -----------------------------------------------------------------------

void ChPeridynamics::VariablesFbReset() {
    // OBSOLETE
    for (auto& node : vnodes) {
        node->VariablesFbReset();
    }
}

void ChPeridynamics::VariablesFbLoadForces(double factor) {
    // RESET FORCES ACCUMULATORS CAUSED BY PERIDYNAMIC MATERIALS!
    // Each node will be reset  as  F=0
    for (auto& mymat : this->materials) {
        mymat->ComputeForcesReset();
    }

    // COMPUTE FORCES CAUSED BY PERIDYNAMIC MATERIALS!
    // Accumulate forces at each node, as  F+=...
    // This can be a time consuming phase, depending on the amount of nodes in each material.
    for (auto& mymat : this->materials) {
        mymat->ComputeForces();
    }

    for (auto& node : vnodes) {
        // add gravity
        ChVector3d Gforce = GetSystem()->GetGravitationalAcceleration() * node->GetMass();
        ChVector3d TotForce = (node->F_peridyn) + node->GetForce() + Gforce;

        node->Variables().Force() += factor * TotForce.eigen();
    }
}

void ChPeridynamics::VariablesQbLoadSpeed() {
    // OBSOLETE
    for (auto& node : vnodes) {
        node->VariablesQbLoadSpeed();
    }
}

void ChPeridynamics::VariablesFbIncrementMq() {
    // OBSOLETE
    for (auto& node : vnodes) {
        node->VariablesFbIncrementMq();
    }
}

void ChPeridynamics::VariablesQbSetSpeed(double step) {
    // OBSOLETE
    for (auto& node : vnodes) {
        node->VariablesQbSetSpeed(step);
    }
}

void ChPeridynamics::VariablesQbIncrementPosition(double step) {
    // OBSOLETE
    for (auto& node : vnodes) {
        node->VariablesQbIncrementPosition(step);
    }
}

void ChPeridynamics::InjectVariables(ChSystemDescriptor& mdescriptor) {
    // variables.SetDisabled(!IsActive());
    for (auto& node : vnodes) {
        node->InjectVariables(mdescriptor);
    }
}

// -----------------------------------------------------------------------

void ChPeridynamics::ForceToRest() {
    for (auto& node : vnodes) {
        node->ForceToRest();
    }
}

void ChPeridynamics::SyncCollisionModels() {
    // COMPUTE COLLISION STATE CHANGES CAUSED BY PERIDYNAMIC MATERIALS!
    // For example generate coll.models in nodes if fractured, remove if elastic-vs-elastic (to reuse persistent
    // bonds)
    for (auto& mymat : this->materials) {
        mymat->ComputeCollisionStateChanges();
    }

    for (auto& node : vnodes) {
        if (node->GetCollisionModel())
            node->GetCollisionModel()->SyncPosition();
    }
}

}  // end namespace chrono
