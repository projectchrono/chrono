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

#ifndef CHMATTERPERIDYNAMICS_H
#define CHMATTERPERIDYNAMICS_H

#include <cmath>

#include "chrono_peridynamics/ChApiPeridynamics.h"
#include "chrono/collision/ChCollisionModel.h"
#include "chrono/physics/ChIndexedNodes.h"
#include "chrono/physics/ChNodeXYZ.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChContinuumMaterial.h"
#include "chrono/solver/ChVariablesNode.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)
class ChSystem;

namespace fea {

class ChProximityContainerPeri;

/// @addtogroup chrono_fea
/// @{



/// Class for a single node in the Peridynamics  cluster
class ChApiPeridynamics ChNodePeri : public ChNodeFEAxyz, public ChContactable_1vars<3> {
public:
    ChNodePeri();
    ChNodePeri(const ChNodePeri& other);
    ~ChNodePeri();

    //ChNodePeri& operator=(const ChNodePeri& other);

    //
    // FUNCTIONS
    //

    /// Get the horizon radius (max. radius while checking surrounding particles).
    double GetHorizonRadius() { return h_rad; }

    /// Set the horizon radius (max. radius while checking surrounding particles).
    void SetHorizonRadius(double mr);


    /// Get collision radius (for colliding with bodies, boundaries, etc.).
    double GetCollisionRadius() { return coll_rad; }
    /// Set collision radius (for colliding with bodies, boundaries, etc.)
    void SetCollisionRadius(double mr);


    /// Access the variables of the node.
    //virtual ChVariablesNode& Variables() override { return variables; }


    //
    // INTERFACE TO ChContactable
    //

    virtual ChContactable::eChContactableType GetContactableType() const override { return CONTACTABLE_3; }

    /// Access variables.
    virtual ChVariables* GetVariables1() override { return &Variables(); }

    /// Tell if the object must be considered in collision detection.
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part).
    virtual int ContactableGet_ndof_x() override { return 3; }

    /// Get the number of DOFs affected by this object (speed part).
    virtual int ContactableGet_ndof_w() override { return 3; }

    /// Get all the DOFs packed in a single vector (position part).
    virtual void ContactableGetStateBlock_x(ChState& x) override { x.segment(0, 3) = this->pos.eigen(); }

    /// Get all the DOFs packed in a single vector (speed part).
    virtual void ContactableGetStateBlock_w(ChStateDelta& w) override { w.segment(0, 3) = this->pos_dt.eigen(); }

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override {
        NodeIntStateIncrement(0, x_new, x, 0, dw);
    }

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector<> GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) override {
        return state_x.segment(0, 3);
    }

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& loc_point,
        const ChState& state_x,
        const ChStateDelta& state_w) override {
        return state_w.segment(0, 3);
    }

    /// Get the absolute speed of point abs_point if attached to the surface.
    /// Easy in this case because there are no roations..
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) override { return this->pos_dt; }

    /// Return the coordinate system for the associated collision model.
    /// ChCollisionModel might call this to get the position of the
    /// contact model (when rigid) and sync it.
    virtual ChCoordsys<> GetCsysForCollisionModel() override { return ChCoordsys<>(this->pos, QNULL); }

    /// Apply the force, expressed in absolute reference, applied in pos, to the
    /// coordinates of the variables. Force for example could come from a penalty model.
    /// The force F and its application point are specified in the absolute reference frame.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F, ///< force 
        const ChVector<>& T, ///< torque
        const ChVector<>& abs_point,
        ChVectorDynamic<>& R) override;

    /// Apply the given force at the given point and load the generalized force array.
    /// The force and its application point are specified in the gloabl frame.
    /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
    /// If needed, the object states must be extracted from the provided state position.
    virtual void ContactComputeQ(const ChVector<>& F, ///< force
        const ChVector<>& T, ///< torque
        const ChVector<>& point,
        const ChState& state_x,
        ChVectorDynamic<>& Q,
        int offset) override {
        Q.segment(offset, 3) = F.eigen();
        Q.segment(offset + 3, 3) = VNULL.eigen();
    }

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
        ChMatrix33<>& contact_plane,
        ChContactable_1vars::type_constraint_tuple& jacobian_tuple_N,
        ChContactable_1vars::type_constraint_tuple& jacobian_tuple_U,
        ChContactable_1vars::type_constraint_tuple& jacobian_tuple_V,
        bool second) override;

    /// Used by some SMC code.
    virtual double GetContactableMass() override { return this->GetMass(); }

    /// This is only for backward compatibility. OBSOLETE
    virtual ChPhysicsItem* GetPhysicsItem() override { return nullptr; };


    //
    // DATA
    //
    bool is_requiring_bonds = true;  // requires collision detection to initialize bounds even if elastic
    bool is_boundary = false;        // always requires collsion detection
    bool is_elastic = false;         // if true, once computed, bounds do not require updating via collision detection proximities

    bool is_colliding = false;       // has a collision model that is already inserted in ChSystem collision engine

    bool IsRequiringCollision() {
        return (is_boundary || !is_elastic || is_requiring_bonds);
    }

    ChVector<> F_peridyn; // placeholder for storing peridynamics forces, automatically computed

    double volume;
    double h_rad;
    double coll_rad;
};


// Base properties per each peridynamics node. Can be inherited if a material requires more data.
class  ChApiPeridynamics ChMatterDataPerNode {
public:
    std::shared_ptr<ChNodePeri> node;
};

// Base properties per each peridynamics bound. Can be inherited if a material requires more data.
class  ChApiPeridynamics ChMatterDataPerBound { 
public: 
    ChNodePeri* nodeA = nullptr;
    ChNodePeri* nodeB = nullptr;

    void Initialize(ChNodePeri* mA, ChNodePeri* mB) {
        nodeA = mA;
        nodeB = mB;
    };
};

// Custom hash for std::pair 
template<typename T>
void hash_combine(std::size_t &seed, T const &key) {
  std::hash<T> hasher;
  seed ^= hasher(key) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
template <typename T1, typename T2>
struct std::hash<::std::pair<T1,T2>>
{
    std::size_t operator()(const std::pair<T1,T2>& s) const noexcept
    {
        std::size_t seed(0);
        hash_combine(seed, s.first);
        hash_combine(seed, s.second);
        return seed;
    }
};

/// Base class for assigning material properties (elasticity, viscosity etc) to a cluster
/// of peridynamics nodes.
class ChApiPeridynamics ChMatterPeriBase { 
public:
    virtual ~ChMatterPeriBase() {
    }

    // -------
    // IMPORTANT FUNCTIONS. 
    // Inherit your material from ChMatterPeri and implement at least
    // the function ComputeForces(). In some materials you may need to implement all three.

    /// CONSTITUTIVE MODEL - INTERFACE TO IMPLEMENT: (optionally)  
    /// Changes the collision model of nodes, from collision to no collision, etc., 
    /// depending on the evolution of the system. It is run right before the collision detection compute proximyty bonds & contacts.
    /// This may add/remove collision models, or may change them. 
    /// The base behavior in ChMatterPeri is: turn on collision model if  mnode->IsRequiringCollision()  
    virtual void ComputeCollisionStateChanges() = 0;

    /// CONSTITUTIVE MODEL - INTERFACE TO IMPLEMENT: (optionally) 
    /// Base behaviour in ChMatterPeri: resets the ChNodePeri::F vector of each ChNodePeri to zero. 
    virtual void ComputeForcesReset() = 0;

    /// CONSTITUTIVE MODEL - INTERFACE TO IMPLEMENT:  ***IMPORTANT***
    /// Add the forces caused by this material to the ChNodePeri::F vector of each ChNodePeri.
    /// Child class can use the containers  this->nodes and this->bounds to compute F, assuming
    /// bounds have been updated with latest collision detection.
    virtual void ComputeForces() = 0;



    // -------
    // Not so important functions - these are already implemented in  ChMatterPeri 
    // with default implementations that should work for all materials.
        
    /// Add a node that will be affected by this material. Note that the node 
    /// must be added also to the ChProximityContainerPeri that manages this material.
    virtual std::shared_ptr<ChNodePeri> AddNode(std::shared_ptr<ChNodePeri> mnode) = 0;

    /// Override/inherit only if really needed. 
    virtual bool RemoveNode(std::shared_ptr<ChNodePeri> mnode) = 0;

    /// Override/inherit only if really needed. There is a default implementation
    /// in ChMatterPeri that takes care of allocating a ChMatterDataPerBound in the map.
    virtual bool AddProximity(
        ChNodePeri* nodeA,   
        ChNodePeri* nodeB) = 0;  

    // Get the owner container that handles peridynamics nodes
    ChProximityContainerPeri* GetContainer() const { return container; }
    // Set the owner container that handles peridynamics nodes
    void SetContainer(ChProximityContainerPeri* mc) { container = mc; }

protected:
    ChProximityContainerPeri* container = 0;

};

/// Sub-base templated class for assigning material properties (elasticity, viscosity etc) to a cluster
/// of peridynamics nodes.
/// It must be inherited by classes that define specific materials, and those may optionally
/// use per-edge and per-node data structures by inheriting ChMatterDataPerNode and ChMatterDataPerBound
/// and passing them as template parameters.

template <class T_per_node = ChMatterDataPerNode, class T_per_bound = ChMatterDataPerBound>
class  ChMatterPeri : public ChMatterPeriBase {

  protected:
    std::unordered_map<ChNodePeri*, T_per_node > nodes;      ///< nodes
    std::unordered_map<std::pair<ChNodePeri*,ChNodePeri*>, T_per_bound> bounds;    ///< bounds

    std::shared_ptr<ChMaterialSurface> matsurface;        ///< data for surface contact and impact

  public:

    /// Build a cluster of nodes for peridynamics.
    /// By default the cluster will contain 0 particles.
    ChMatterPeri() {
        matsurface = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    }

    ChMatterPeri(const ChMatterPeri& other) {
        matsurface = other.matsurface;
        nodes = other.nodes;
        bounds = other.bounds;
    }

    virtual ~ChMatterPeri() {
    }


    /// Get the number of nodes.
    unsigned int GetNnodes() const { return (unsigned int)nodes.size(); }
    
    /// Get the number of bounds.
    unsigned int GetNbounds() const { return (unsigned int)bounds.size(); }

    /// Access the node container
    const std::unordered_map<ChNodePeri*, T_per_node>& GetMapOfNodes() {
        return nodes;
    }
    /// Access the bounds container
    std::unordered_map<std::pair<ChNodePeri*,ChNodePeri*>, T_per_bound>& GetMapOfBounds() {
        return bounds;
    }

    /// Add a  node to the particle cluster
    virtual std::shared_ptr<ChNodePeri> AddNode(std::shared_ptr<ChNodePeri> mnode) override {
        T_per_node node_data;
        node_data.node = mnode;

        //mnode->variables.SetUserData((void*)this);

        this->nodes[mnode.get()] = node_data;  // add to container using ptr of node as unique key

        return (mnode);
    }

    /// Remove a node from the particle cluster
    virtual bool RemoveNode(std::shared_ptr<ChNodePeri> mnode) override {
        this->nodes.erase(mnode.get());
        return true;
    }

    /// Add a proximity: this will be called thousand of times by the broadphase collision detection.
    /// If the two nodes are of proper type, and if bond not yet present, add a T_per_node structure 
    /// to this->bounds and return true, otherwise return false.
    virtual bool AddProximity(
        ChNodePeri* nodeA,   
        ChNodePeri* nodeB    
    ) override {
        ChNodePeri* anodeA = nodeA;
        ChNodePeri* anodeB = nodeB;
        
        // do not add if both nodes are not in this material
        auto foundA = this->nodes.find(anodeA); 
        if (foundA == this->nodes.end())
            return false;
        auto foundB = this->nodes.find(anodeB);
        if (foundB == this->nodes.end())
            return false;
        
        // force A and B to be in increasing order, otherwise two hash keys for same pair
        if (anodeA > anodeB)
            std::swap(anodeA, anodeB);

        auto foundBound = this->bounds.find(std::pair<ChNodePeri*, ChNodePeri*>(anodeA, anodeB));
        if (foundBound != this->bounds.end())
            return false;

        // avoid adding bounds more distant than horizon (use only one of the two horizons of the two particles)
        if ((anodeA->GetPos() - anodeB->GetPos()).Length() > (anodeA->GetHorizonRadius()))
            return false;

        if ((this->GetContainer()->GetChTime()>0.01) && anodeA->is_boundary && anodeB->is_boundary && anodeA->is_elastic && anodeB->is_elastic)
            return false;

        // add bound to container using ptr of two nodes as unique key
        this->bounds[std::pair<ChNodePeri*, ChNodePeri*>(anodeA, anodeB)].Initialize(anodeA, anodeB);
        
        return true;
    }

    
    /// CONSTITUTIVE MODEL - INTERFACE TO IMPLEMENT: (optionally) 
    /// Base behaviour: 
    ///  - resets the ChNodePeri::F vector of each ChNodePeri to zero.
    ///  - adds or remove collision model from the collision engine 
    ///  - resets is_elastic optimization flag (is is_elastic, bounds are persistent and collision engine not needed)
    virtual void ComputeForcesReset() override {
        
        for (auto& nodedata : nodes) {

            // resets force accumulator to zero
            auto& mnode = nodedata.second.node;
            mnode->F_peridyn = VNULL;

            // Initialize flag to mark if the bounds are persistent, as in elastic media. Default: true.
            // Plastic w.large deformation or fluids will flag is_elastic to false, if needed.
            mnode->is_elastic = true;

        }
    }

    /// CONSTITUTIVE MODEL - INTERFACE TO IMPLEMENT: (optionally)  
    /// Changes the collision model of nodes, from collision to no collision, etc., ex when an interface is generated, 
    /// depending on the evolution of the system. It is run right before the collision detection compute proximyty bonds & contacts.
    /// This may add/remove collision models, or may change them. 
    /// The base behavior is: create collision models when mnode->IsRequiringCollision(), remove if not. 
    virtual void ComputeCollisionStateChanges() override {

        for (auto& nodedata : nodes) {

            auto& mnode = nodedata.second.node;

            // add collision model to system if not yet added
            if (!mnode->is_colliding && mnode->IsRequiringCollision()) {
                // create model
                if (!mnode->GetCollisionModel()) {
                    double aabb_rad = mnode->GetHorizonRadius() / 2;  // to avoid too many pairs: bounding boxes hemisizes will sum..  __.__--*--
                    double coll_rad = mnode->GetCollisionRadius();
                    //auto cshape = chrono_types::make_shared<ChCollisionShapePoint>(matsurface, VNULL, coll_rad); // ..Point like ..Sphere, but no point-vs-point contact generation 
                    auto cshape = chrono_types::make_shared<ChCollisionShapeSphere>(matsurface, coll_rad);
                    mnode->AddCollisionShape(cshape);
                    mnode->GetCollisionModel()->SetSafeMargin(coll_rad);
                    mnode->GetCollisionModel()->SetEnvelope(ChMax(0.0, aabb_rad - coll_rad));
                }
                // add to system
                container->GetSystem()->GetCollisionSystem()->Add(mnode->GetCollisionModel());
                mnode->is_colliding = true; // prevents other material adding twice
            }
            // remove collision model from system if not yet removed
            if (mnode->is_colliding && !mnode->IsRequiringCollision()) {
                container->GetSystem()->GetCollisionSystem()->Remove(mnode->GetCollisionModel());
                mnode->is_colliding = false; // prevents other material adding twice
            }

            // Deactivate this flag, that is used only to force the generation of bounds once. 
            // If collison model for proximity collision was needed, than it was alredy setup few lines above 
            mnode->is_requiring_bonds = false;
        }

        for (auto& nodedata : nodes) {
            nodedata.second.node->is_elastic = true;
        }
    };
    

    /// Set the material surface for 'boundary contact'.
    void SetMaterialSurface(const std::shared_ptr<ChMaterialSurface>& mnewsurf) { matsurface = mnewsurf; }

    /// Set the material surface for 'boundary contact'.
    std::shared_ptr<ChMaterialSurface>& GetMaterialSurface() { return matsurface; }


    //
    // STREAMING
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) {
        // version number
        marchive.VersionWrite<ChMatterPeri>();

        // serialize the parent class data too
        //ChMatterPeriBase::ArchiveOut(marchive);

        // serialize all member data:
        //***TODO
    }

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) {
        // version number
        /*int version =*/ marchive.VersionRead<ChMatterPeri>();

        // deserialize the parent class data too
        //ChMatterPeriBase::ArchiveIn(marchive);

        // deserialize all member data:
        //***TODO
    }
};



/// The simplest peridynamic material: a bond-based material based on a 
/// network of springs, each with the same stiffness k regardless of length, etc. 
/// Just for didactical purposes - do not use it for serious applications.
/// Also use a damping coefficient r. 

class ChApiPeridynamics ChMatterPeriSprings : public ChMatterPeri<> {
public:
    double k = 100;
    double r = 10;

    ChMatterPeriSprings() {};

    // Implement the function that adds the peridynamics force to each node, as a 
    // summation of all the effects of neighbouring nodes.
    virtual void ComputeForces() {
        // loop on bounds
        for (auto& bound : this->bounds) {
            ChMatterDataPerBound& mbound = bound.second;
            ChVector<> old_vdist = mbound.nodeB->GetX0() - mbound.nodeA->GetX0();
            ChVector<>     vdist = mbound.nodeB->GetPos() - mbound.nodeA->GetPos();
            ChVector<>     vdir = vdist.GetNormalized();
            double         vel = Vdot(vdir, mbound.nodeB->GetPos_dt() - mbound.nodeA->GetPos_dt());
            ChVector force_val = (vdist.Length() - old_vdist.Length()) * this->k  + vel * this->r;
            mbound.nodeB->F_peridyn += -vdir * force_val;
            mbound.nodeA->F_peridyn += vdir * force_val;
        }
    };
};


class  ChApiPeridynamics ChMatterDataPerBoundBreakable : public ChMatterDataPerBound { 
public: 
    bool broken = false;
};

class ChApiPeridynamics ChMatterPeriSpringsBreakable : public ChMatterPeri<ChMatterDataPerNode, ChMatterDataPerBoundBreakable> {
public:
    double k = 100;
    double r = 10;
    double max_stretch = 0.08;

    ChMatterPeriSpringsBreakable() {};

    // Implement the function that adds the peridynamics force to each node, as a 
    // summation of all the effects of neighbouring nodes.
    virtual void ComputeForces() {
        // loop on bounds
        for (auto& bound : this->bounds) {
            ChMatterDataPerBoundBreakable& mbound = bound.second;
            if (!mbound.broken) {
                ChVector<> old_vdist = mbound.nodeB->GetX0() - mbound.nodeA->GetX0();
                ChVector<>     vdist = mbound.nodeB->GetPos() - mbound.nodeA->GetPos();
                ChVector<>     vdir = vdist.GetNormalized();
                double         vel = Vdot(vdir, mbound.nodeB->GetPos_dt() - mbound.nodeA->GetPos_dt());
                ChVector force_val = (vdist.Length() - old_vdist.Length()) * this->k + vel * this->r;
                mbound.nodeB->F_peridyn += -vdir * force_val;
                mbound.nodeA->F_peridyn += vdir * force_val;

                double stretch = (vdist.Length() - old_vdist.Length()) / old_vdist.Length();
                if (stretch > max_stretch) {
                    mbound.broken = true;
                    // the following will propagate the fracture geometry so that broken parts can collide
                    mbound.nodeA->is_boundary = true; 
                    mbound.nodeB->is_boundary = true;
                }
            }
            else {
                if ((mbound.nodeB->GetPos() - mbound.nodeA->GetPos()).Length() > mbound.nodeA->GetHorizonRadius())
                    bounds.erase(bound.first);
            }

        }
    }
};



class /*ChApiPeridynamics*/ ChVisualPeriSpringsBreakable : public ChGlyphs {
public:
    ChVisualPeriSpringsBreakable(std::shared_ptr<ChMatterPeriSpringsBreakable> amatter) : mmatter(amatter) { is_mutable = true; };
    virtual ~ChVisualPeriSpringsBreakable() {}

protected:
    virtual void Update(ChPhysicsItem* updater, const ChFrame<>& frame) {
        if (!mmatter)
            return;
        this->Reserve(mmatter->GetNnodes());
        unsigned int i = 0;
        for (const auto& anode : mmatter->GetMapOfNodes()) {
            this->SetGlyphPoint(i, anode.second.node->GetPos());
            ++i;
        }
    }

    std::shared_ptr<ChMatterPeriSpringsBreakable> mmatter;
};



/// @} chrono_fea

}  // end namespace fea
}  // end namespace chrono

#endif
