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

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_peridynamics/ChMatterPeridynamics.h"
#include "chrono_peridynamics/ChProximityContainerPeridynamics.h"

namespace chrono {

using namespace fea;
using namespace peridynamics;
using namespace geometry;



static double W_sph(double r, double h) {
    if (r < h) {
        return (315.0 / (64.0 * CH_C_PI * pow(h, 9))) * pow((h * h - r * r), 3);
    } else
        return 0;
}

static double W_sq_visco(double r, double h) {
    if (r < h) {
        return (45.0 / (CH_C_PI * pow(h, 6))) * (h - r);
    } else
        return 0;
}


// ------------------------------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChProximityContainerPeri)

ChProximityContainerPeri::ChProximityContainerPeri() : n_added(0), is_updated(true), n_dofs(0) {
}

ChProximityContainerPeri::ChProximityContainerPeri(const ChProximityContainerPeri& other)
    : ChProximityContainer(other) {
    vnodes = other.vnodes; 
    n_added = other.n_added;
    is_updated = other.is_updated;
    materials = other.materials;
    n_dofs = other.n_dofs;
}

ChProximityContainerPeri::~ChProximityContainerPeri() {
}

void ChProximityContainerPeri::RemoveAllProximities() {

    // nothing to do: proximities are handled by materials in this->materials
    n_added = 0;
}

void ChProximityContainerPeri::BeginAddProximities() {
    
    // nothing to do: proximities are handled by materials in this->materials

    n_added = 0;
}

void ChProximityContainerPeri::EndAddProximities() {

    is_updated = false;
    // nothing to do: proximities are handled by materials in this->materials
}

void ChProximityContainerPeri::AddProximity(ChCollisionModel* modA, ChCollisionModel* modB) {
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

void ChProximityContainerPeri::ReportAllProximities(ReportProximityCallback* mcallback) {
    
    // TODO: proximities are handled by materials in this->materials
}


void ChProximityContainerPeri::SetupInitial() {
    n_dofs = 0;

    for (unsigned int i = 0; i < vnodes.size(); i++) {
        if (!vnodes[i]->IsFixed()) {
            vnodes[i]->SetupInitial(GetSystem());

            // count the degrees of freedom
            n_dofs += vnodes[i]->GetNdofX_active();
        }
    }
}

void ChProximityContainerPeri::Setup() {
    n_dofs = 0;

    for (unsigned int i = 0; i < vnodes.size(); i++) {
        // Set node offsets in state vectors (based on the offsets of the containing mesh)
        vnodes[i]->NodeSetOffset_x(GetOffset_x() + n_dofs);;

        // Count the actual degrees of freedom (consider only nodes that are not fixed)
        if (!vnodes[i]->IsFixed()) {
            n_dofs += vnodes[i]->GetNdofX_active();
        }
    }
}

void ChProximityContainerPeri::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChProximityContainer::Update(mytime, update_assets);

    if (!is_updated) // avoids overhead of Update()
    {
        // tentatively mark as updated
        is_updated = true;
    }

}


void ChProximityContainerPeri::AddMatter(std::shared_ptr<ChMatterPeriBase> mmatter) {
        materials.push_back(mmatter);
        mmatter->SetContainer(this);
}

void ChProximityContainerPeri::AddNode(std::shared_ptr<ChNodePeri> m_node) {
    vnodes.push_back(m_node);

    // If the obj is is already added to a system, mark the system uninitialized and out-of-date
    /*
    if (system) {
        system->is_initialized = false;
        system->is_updated = false;
    }
    */
}

/*
void ChProximityContainerPeri::Fill(
        std::shared_ptr<fea::ChMatterPeriBase> mmatter, ///< matter to be used for this volume. Must be added too to this, via AddMatter(). 
        utils::PointVectorD& points,                    ///< points obtained from a sampler of the volume, ex. use utils::Sampler
        const double spacing,                           ///< average spacing used in sampling
        const double mass,                              ///< total mass of the volume
        const double horizon_sfactor,                   ///< the radius of horizon of the particle is 'spacing' multiplied this value
        const ChCoordsys<> mcoords                      ///< position and rotation of the volume
        )  
{
    double horizon = horizon_sfactor * spacing;
    unsigned int nsamples = points.size();
    double per_node_mass = mass / nsamples;

    for (const auto& mpos_loc : points) {
        ChVector<> mpos = mcoords.TransformPointLocalToParent(mpos_loc);
        auto mnode = chrono_types::make_shared<ChNodePeri>();
        mnode->SetPosReference(mpos);
        mnode->SetPos(mpos);
        mnode->SetMass(per_node_mass);

        // Add to this
        this->AddNode(mnode);

        // Add to the material, it must know the map of nodes, also it will allocate per-node custom data.
        mmatter->AddNode(mnode,horizon);
    }
}
*/

void ChProximityContainerPeri::FillBox(
        std::shared_ptr<ChMatterPeriBase> mmatter, ///< matter to be used for this volume. Must be added too to this, via AddMatter(). 
        const ChVector<> size,         ///< x,y,z sizes of the box to fill (better if integer multiples of spacing)
        const double spacing,          ///< the spacing between two near nodes
        const double initial_density,  ///< density of the material inside the box, for initialization of node's masses
        const ChCoordsys<> boxcoords,  ///< position and rotation of the box
        const bool do_centeredcube,  ///< if false, array is simply cubic, if true is centered cubes (highest regularity)
        const double horizon_sfactor,  ///< the radius of horizon of the particle is 'spacing' multiplied this value
        const double collision_sfactor,
        const double randomness       ///< randomness of the initial distribution lattice, 0...1
        )  
{
    int samples_x = (int)(size.x() / spacing);
    int samples_y = (int)(size.y() / spacing);
    int samples_z = (int)(size.z() / spacing);
    int totsamples = samples_x*samples_y*samples_z;
    if (do_centeredcube)
        totsamples += (samples_x - 1) * (samples_y - 1) * (samples_z - 1);

    double mrandomness = randomness;
    if (do_centeredcube)
        mrandomness = randomness * 0.5;

    double horizon = horizon_sfactor * spacing;
    double collrad = collision_sfactor * spacing;
    double mtotvol = size.x() * size.y() * size.z();
    double mtotmass = mtotvol * initial_density;
    double nodemass = mtotmass / (double)totsamples;

    for (int ix = 0; ix < samples_x; ix++)
        for (int iy = 0; iy < samples_y; iy++)
            for (int iz = 0; iz < samples_z; iz++) {
                ChVector<> pos(ix * spacing - 0.5 * size.x(), iy * spacing - 0.5 * size.y(), iz * spacing - 0.5 * size.z());
                pos += ChVector<>(mrandomness * ChRandom() * spacing, mrandomness * ChRandom() * spacing,
                                    mrandomness * ChRandom() * spacing);
                ChVector<> mpos = boxcoords.TransformPointLocalToParent(pos);
                auto mnode = chrono_types::make_shared<ChNodePeri>();
                mnode->SetX0(mpos);
                mnode->SetPos(mpos);
                mnode->SetMass(nodemass);
                mnode->is_elastic = true;
                mnode->coll_rad = collrad;
                mnode->h_rad = horizon;
                this->AddNode(mnode);
                mmatter->AddNode(mnode);
                if ((ix == 0) || (ix == samples_x - 1) || (iy == 0) || (iy == samples_y - 1) || (iz == 0) || (iz == samples_z - 1)) {
                    mnode->is_boundary = true;
                }

                if (do_centeredcube && !((ix == samples_x - 1) || (iy == samples_y - 1) || (iz == samples_z - 1))) {
                    ChVector<> pos2 = pos + 0.5 * ChVector<>(spacing, spacing, spacing);
                    pos2 += ChVector<>(mrandomness * ChRandom() * spacing, mrandomness * ChRandom() * spacing,
                                        mrandomness * ChRandom() * spacing);
                    mpos = boxcoords.TransformPointLocalToParent(pos2);
                    mnode = chrono_types::make_shared<ChNodePeri>();
                    mnode->SetX0(mpos);
                    mnode->SetPos(mpos);
                    mnode->SetMass(nodemass);
                    mnode->is_elastic = true;
                    mnode->coll_rad = collrad;
                    mnode->h_rad = horizon;
                    this->AddNode(mnode);
                    mmatter->AddNode(mnode);
                }
            }
}


void ChProximityContainerPeri::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChProximityContainerPeri>();
    // serialize parent class
    ChProximityContainer::ArchiveOut(marchive);
    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChProximityContainerPeri::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChProximityContainerPeri>();
    // deserialize parent class
    ChProximityContainer::ArchiveIn(marchive);
    // stream in all member data:
}



}  // end namespace chrono
