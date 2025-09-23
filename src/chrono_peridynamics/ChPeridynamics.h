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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHPERIDYNAMICS_H
#define CHPERIDYNAMICS_H

#include <list>

#include "chrono/physics/ChProximityContainer.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_peridynamics/ChApiPeridynamics.h"
#include "chrono_peridynamics/ChNodePeri.h"

namespace chrono {
namespace peridynamics {

/// @addtogroup chrono_peridynamics
/// @{

// Forward declaration
class ChMatterPeriBase;

/// Class for handling proximity pairs for a peridynamics
/// deformable continuum (necessary for inter-particle material forces),
/// Such an item must be addd to the physical system if you want to use
/// one or more ChMatterPeri materials.

class ChApiPeridynamics ChPeridynamics : public ChProximityContainer {
  public:
    ChPeridynamics();
    ChPeridynamics(const ChPeridynamics& other);
    virtual ~ChPeridynamics();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChPeridynamics* Clone() const override { return new ChPeridynamics(*this); }

    // PERIDYNAMICS

    /// Adds a peridynamics material. Materials, each acting on cluster of ChNodePeri items, must be added to this
    /// proximity manager before the simulation starts.
    void AddMatter(std::shared_ptr<ChMatterPeriBase> mmatter);

    /// Get the array of materials.
    const std::list<std::shared_ptr<ChMatterPeriBase>>& GetMaterials() const { return materials; }

    /// Add a node. Only nodes that are added here will be simulated. Nodes can be shared among diffferent ChMatterPeri,
    /// but not among different ChPeridynamics.
    void AddNode(std::shared_ptr<ChNodePeri> m_node);

    /// Get the array of nodes.
    const std::vector<std::shared_ptr<ChNodePeri>>& GetNodes() const { return vnodes; }

    /// Get the number of nodes.
    unsigned int GetNnodes() const { return (unsigned int)vnodes.size(); }

    /// Create a region filled with nodes. Adds the node to this, and to the specified matter. This is a helper function
    /// so that you avoid to create all nodes one by one with many calls
    /// to AddNode().

    void Fill(
        std::shared_ptr<ChMatterPeriBase>
            mmatter,  ///< matter to be used for this volume. Must be added too to this, via AddMatter().
        std::vector<ChVector3d>& points,  ///< points obtained from a sampler of the volume, ex. use utils::Sampler
        const double spacing,             ///< average spacing used in sampling
        const double mass,                ///< total mass of the volume
        const double volume,              ///< total volume
        const double horizon_sfactor,     ///< the radius of horizon of the particle is 'spacing' multiplied this value
        const double collision_sfactor,   ///< the radius of collision shape (sphere) of the particle is 'spacing'
                                          ///< multiplied this value
        const ChCoordsys<> mcoords        ///< position and rotation of the volume
    );

    /// Create a box filled with nodes. Face nodes are automatically marked as interface, i.e. are collidable.
    /// Adds the nodes to this, and to the specified matter.
    void FillBox(
        std::shared_ptr<ChMatterPeriBase>
            mmatter,            ///< matter to be used for this volume. Must be added too to this, via AddMatter().
        const ChVector3d size,  ///< x,y,z sizes of the box to fill (better if integer multiples of spacing)
        const double spacing,   ///< the spacing between two near nodes (grid interval h)
        const double initial_density,  ///< density of the material inside the box, for initialization of node's masses
        const ChCoordsys<> boxcoords = CSYSNORM,  ///< position and rotation of the box
        const double horizon_sfactor =
            1.6,  ///< the radius of horizon of the particle is 'spacing' multiplied this value
        const double collision_sfactor =
            0.3,  ///< the radius of collision shape (sphere) of the particle is 'spacing' multiplied this value
        const double randomness = 0.0  ///< randomness of the initial distribution lattice, 0...1
    );

    /// Create a multi-layer box filled with nodes, with N layers of different materials ordered along X.
    /// Nodes at interface are shared, as layers were perfectly glued.
    /// Face nodes are automatically marked as interface, i.e. are collidable.
    void FillBox(
        std::vector<std::pair<std::shared_ptr<ChMatterPeriBase>, double>>
            v_mmatter,          ///< {matters,x_thickness} pairs to be used for layers. Must be added too to this, via
                                ///< AddMatter().
        const ChVector3d size,  ///< x,y,z sizes of the box to fill (better if integer multiples of spacing)
        const double spacing,   ///< the spacing between two near nodes (grid interval h)
        const double initial_density,  ///< density of the material inside the box, for initialization of node's masses
        const ChCoordsys<> boxcoords = CSYSNORM,  ///< position and rotation of the box
        const double horizon_sfactor =
            1.6,  ///< the radius of horizon of the particle is 'spacing' multiplied this value
        const double collision_sfactor =
            0.3,  ///< the radius of collision shape (sphere) of the particle is 'spacing' multiplied this value
        const double randomness = 0.0  ///< randomness of the initial distribution lattice, 0...1
    );

    /// Create a sphere filled with nodes, with lattice xyz sampling.
    /// Nodes at the surface of the sphere are automatically marked as interface, i.e. are collidable.
    void FillSphere(
        std::shared_ptr<ChMatterPeriBase>
            mmatter,                 ///< matter to be used for the sphere. Must be added too to this, via AddMatter().
        const double sphere_radius,  ///< radius of sphere to fill
        const double spacing,        ///< the spacing between two near nodes (grid interval h)
        const double initial_density,  ///< density of the material inside the box, for initialization of node's masses
        const ChCoordsys<> boxcoords = CSYSNORM,  ///< position and rotation of the box
        const double horizon_sfactor =
            1.6,  ///< the radius of horizon of the particle is 'spacing' multiplied this value
        const double collision_sfactor =
            0.3  ///< the radius of collision shape (sphere) of the particle is 'spacing' multiplied this value
    );

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

    // INTERFACE TO PROXIMITY CONTAINER

    /// Tell the number of added contacts
    virtual int GetNproximities() const override { return n_added; }

    /// Remove (delete) all contained contact data.
    virtual void RemoveAllProximities() override;

    /// The collision system will call BeginAddProximities() before adding
    /// all pairs (for example with AddProximity() ).
    virtual void BeginAddProximities() override;

    /// Add a proximity data between two collision models, if possible.
    virtual void AddProximity(ChCollisionModel* modA,  ///< get contact model 1
                              ChCollisionModel* modB   ///< get contact model 2
                              ) override;

    /// The collision system will call BeginAddContact() after adding
    /// all contacts (for example with AddContact() or similar). This optimized version
    /// purges the end of the list of contacts that were not reused (if any).
    virtual void EndAddProximities() override;

    /// Scans all the proximity pairs and, for each pair, executes the OnReportProximity()
    /// function of the provided callback object.
    /// DO NOTHING - please iterate on the bonds of the materials.
    virtual void ReportAllProximities(ReportProximityCallback* mcallback) override;

    // SETUP AND UPDATE

    /// Setup initial lattice of bonds, running a single shot of collision detection
    /// and then calling Compute. It may require some CPU time.
    /// Note: call this as late as possible, after you created all peridynamic material,
    /// ex. just before running the simulation loop.
    /// Call as a static function:   ChPeridynamics::SetupInitialBonds(my_sys, my_peridynamics);
    static void SetupInitialBonds(ChSystem* sys, std::shared_ptr<ChPeridynamics> peri);

    /// This recomputes the number of DOFs, constraints,
    /// as well as state offsets of contained items.
    virtual void Setup() override;

    /// This will call   ComputeForcesReset(), ComputeForces(), ComputeStates()  for each material
    virtual void Update(double mytime, bool update_assets = true) override;

    // STATE

    /// Get the number of scalar coordinates (variables), if any, in this item, excluding those that are in fixed state.
    virtual unsigned int GetNumCoordsPosLevel() override { return n_dofs; }

    /// Get the number of scalar constraints.
    virtual unsigned int GetNumConstraints() override { return n_constr; }

    virtual void IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T) override;

    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T,
                                 bool full_update) override;

    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;

    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;

    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;

    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c) override;

    virtual void IntLoadLumpedMass_Md(const unsigned int off,
                                      ChVectorDynamic<>& Md,
                                      double& err,
                                      const double c) override;

    /// Takes the term Cq'*L, scale and adds to R at given offset:
    ///    R += c*Cq'*L
    virtual void IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                     ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                     const ChVectorDynamic<>& L,  ///< the L vector
                                     const double c               ///< a scaling factor
                                     ) override;

    /// Takes the term C, scale and adds to Qc at given offset:
    ///    Qc += c*C
    virtual void IntLoadConstraint_C(const unsigned int off,  ///< offset in Qc residual
                                     ChVectorDynamic<>& Qc,   ///< result: the Qc residual, Qc += c*C
                                     const double c,          ///< a scaling factor
                                     bool do_clamp,           ///< apply clamping to c*C?
                                     double recovery_clamp    ///< value for min/max clamping of c*C
                                     ) override;

    /// Register with the given system descriptor any ChConstraint objects associated with this item.
    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;

    /// Compute and load current Jacobians in encapsulated ChConstraint objects.
    virtual void LoadConstraintJacobians() override;

    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) override;

    // SOLVER INTERFACE

    // Override/implement system functions of ChPhysicsItem (to assemble/manage data for system solver))

    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void VariablesQbIncrementPosition(double step) override;
    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override;

    // Other functions

    /// Set no speed and no accelerations (but does not change the position).
    virtual void ForceToRest() override;

    /// Synchronize coll.models coordinates and bounding boxes to the positions of the particles.
    virtual void SyncCollisionModels() override;

  private:
    /// Initial setup (before analysis).
    /// This function is called from ChSystem::SetupInitial, marking a point where system
    /// construction is completed.
    /// <pre>
    ///   - Computes the total number of degrees of freedom
    ///   - Precompute auxiliary data, such as (local) stiffness matrices .
    /// </pre>
    virtual void SetupInitial() override;

  protected:
    std::list<std::shared_ptr<ChMatterPeriBase>> materials;
    std::vector<std::shared_ptr<ChNodePeri>> vnodes;

    int n_added;
    bool is_updated;
    unsigned int n_dofs;    ///< total degrees of freedom
    unsigned int n_constr;  ///< total degrees of constraints
};

/// @} chrono_peridynamics

}  // namespace peridynamics
}  // end namespace chrono

#endif
