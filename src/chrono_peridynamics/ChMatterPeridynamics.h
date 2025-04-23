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
#include "chrono_peridynamics/ChNodePeri.h"
#include "chrono/collision/ChCollisionModel.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/solver/ChVariablesNode.h"

namespace chrono {
namespace peridynamics {

// Forward declaration
class ChPeridynamics;

/// @addtogroup chrono_peridynamics
/// @{




/// Base properties per each peridynamics node. Can be inherited if a material requires more data.
class  ChApiPeridynamics ChMatterDataPerNode {
public:
    std::shared_ptr<ChNodePeri> node;
};

/// Base properties per each peridynamics bond. Can be inherited if a material requires more data.
class  ChApiPeridynamics ChMatterDataPerBond { 
public: 
    ChNodePeri* nodeA = nullptr;
    ChNodePeri* nodeB = nullptr;

    virtual void Initialize(ChNodePeri* mA, ChNodePeri* mB) {
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
/// Do not inherit directly from this. Better inherit from ChMatterPeri, that provides
/// some useful default functionality like activating/deactivating collision shapes.

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
    /// Base behaviour in ChMatterPeri: resets the ChNodePeri::F_peridyn vector of each ChNodePeri to zero. 
    virtual void ComputeForcesReset() = 0;

    /// CONSTITUTIVE MODEL - INTERFACE TO IMPLEMENT:  ***IMPORTANT***
    /// Add the forces caused by this material to the ChNodePeri::F vector of each ChNodePeri.
    /// Child class can use the containers  this->nodes and this->bonds to compute F, assuming
    /// bonds have been updated with latest collision detection.
    virtual void ComputeForces() = 0;

    /// CONSTITUTIVE MODEL - INTERFACE TO IMPLEMENT: (optionally) 
    /// This function is called where system construction is completed, at the 
    /// beginning of the simulation. Maybe your constitutive law has to initialize
    /// some state data, if so you can implement this function, otherwise leave as empty.
    virtual void SetupInitial() {};

    /// CONSTITUTIVE MODEL - INTERFACE TO IMPLEMENT: (optionally) 
    /// This function is called at each time step. Maybe your constitutive law has to initialize
    /// some state data, if so you can implement this function, otherwise leave as empty.
    virtual void Setup() {};

    // STATE

    /// Get the number of scalar constraints (maybe used for implicit materials)
    virtual unsigned int GetNumConstraints() { return 0; }

    virtual void IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
        ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
        const ChVectorDynamic<>& L,  ///< the L vector
        const double c               ///< a scaling factor
    ) {}

    /// Takes the term C, scale and adds to Qc at given offset:
    ///    Qc += c*C
    virtual void IntLoadConstraint_C(const unsigned int off,  ///< offset in Qc residual
        ChVectorDynamic<>& Qc,   ///< result: the Qc residual, Qc += c*C
        const double c,          ///< a scaling factor
        bool do_clamp,           ///< apply clamping to c*C?
        double recovery_clamp    ///< value for min/max clamping of c*C
    ) {}

    /// Register with the given system descriptor any ChConstraint objects associated with this item.
    virtual void InjectConstraints(ChSystemDescriptor& descriptor) {}

    /// Compute and load current Jacobians in encapsulated ChConstraint objects.
    virtual void LoadConstraintJacobians() {}

    virtual void IntToDescriptor(const unsigned int off_v,
        const ChStateDelta& v,
        const ChVectorDynamic<>& R,
        const unsigned int off_L,
        const ChVectorDynamic<>& L,
        const ChVectorDynamic<>& Qc) {}

    virtual void IntFromDescriptor(const unsigned int off_v,
        ChStateDelta& v,
        const unsigned int off_L,
        ChVectorDynamic<>& L) {}


    // -------
    // Not so important functions - these are already implemented in  ChMatterPeri 
    // with default implementations that should work for all materials.
        
    /// Add a node that will be affected by this material. Note that the node 
    /// must be added also to the ChPeridynamics that manages this material.
    virtual std::shared_ptr<ChNodePeri> AddNode(std::shared_ptr<ChNodePeri> mnode) = 0;

    /// Override/inherit only if really needed. 
    virtual bool RemoveNode(std::shared_ptr<ChNodePeri> mnode) = 0;

    /// Override/inherit only if really needed. There is a default implementation
    /// in ChMatterPeri that takes care of allocating a ChMatterDataPerBond in the map.
    virtual bool AddProximity(
        ChNodePeri* nodeA,   
        ChNodePeri* nodeB) = 0;  

    // Get the owner container that handles peridynamics nodes
    ChPeridynamics* GetContainer() const { return container; }
    // Set the owner container that handles peridynamics nodes
    void SetContainer(ChPeridynamics* mc) { container = mc; }

protected:
    ChPeridynamics* container = 0;

};



/// Sub-base templated class for assigning material properties (elasticity, viscosity etc) to a cluster
/// of peridynamics nodes.
/// It must be inherited by classes that define specific materials, and those may optionally
/// use per-edge and per-node data structures by inheriting ChMatterDataPerNode and ChMatterDataPerBond
/// and passing them as template parameters.

template <class T_per_node = ChMatterDataPerNode, class T_per_bond = ChMatterDataPerBond>
class  ChMatterPeri : public ChMatterPeriBase {

  protected:
    std::unordered_map<ChNodePeri*, T_per_node > nodes;      ///< nodes
    std::unordered_map<std::pair<ChNodePeri*,ChNodePeri*>, T_per_bond> bonds;    ///< bonds

    std::shared_ptr<ChContactMaterial> matsurface;        ///< data for surface contact and impact

  public:

    /// Build a cluster of nodes for peridynamics.
    /// By default the cluster will contain 0 particles.
    ChMatterPeri() {
        matsurface = chrono_types::make_shared<ChContactMaterialNSC>();
    }

    ChMatterPeri(const ChMatterPeri& other) {
        matsurface = other.matsurface;
        nodes = other.nodes;
        bonds = other.bonds;
    }

    virtual ~ChMatterPeri() {
    }


    /// Get the number of nodes.
    unsigned int GetNnodes() const { return (unsigned int)nodes.size(); }
    
    /// Get the number of bonds.
    unsigned int GetNbonds() const { return (unsigned int)bonds.size(); }

    /// Access the node container
    const std::unordered_map<ChNodePeri*, T_per_node>& GetMapOfNodes() {
        return nodes;
    }
    /// Access the bonds container
    std::unordered_map<std::pair<ChNodePeri*,ChNodePeri*>, T_per_bond>& GetMapOfBonds() {
        return bonds;
    }

    /// Add a  node to the particle cluster
    virtual std::shared_ptr<ChNodePeri> AddNode(std::shared_ptr<ChNodePeri> mnode) override {
        T_per_node node_data;
        node_data.node = mnode;

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
    /// to this->bonds and return true, otherwise return false.
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

        auto foundBond = this->bonds.find(std::pair<ChNodePeri*, ChNodePeri*>(anodeA, anodeB));
        if (foundBond != this->bonds.end())
            return false;

        // avoid adding bonds more distant than horizon (use only one of the two horizons of the two particles)
        if ((anodeA->GetPos() - anodeB->GetPos()).Length() > (anodeA->GetHorizonRadius()))
            return false;

        if (!(anodeA->is_requiring_bonds && anodeB->is_requiring_bonds) && anodeA->is_boundary && anodeB->is_boundary && (!anodeA->is_fluid) && (!anodeB->is_fluid) )
            return false;

        // add bond to container using ptr of two nodes as unique key
        this->bonds[std::pair<ChNodePeri*, ChNodePeri*>(anodeA, anodeB)].Initialize(anodeA, anodeB);
        
        return true;
    }

    
    /// CONSTITUTIVE MODEL - INTERFACE TO IMPLEMENT: (optionally) 
    /// Base behaviour: 
    ///  - resets the ChNodePeri::F vector of each ChNodePeri to zero.
    ///  - adds or remove collision model from the collision engine 
    ///  - resets is_fluid optimization flag (is not is_fluid, bonds are persistent as an elastic, and collision engine not needed)
    virtual void ComputeForcesReset() override {
        
        for (auto& nodedata : nodes) {

            // resets force accumulator to zero
            auto& mnode = nodedata.second.node;
            mnode->F_peridyn = VNULL;

            // Initialize flag to mark if the bonds are persistent, as in elastic media. Default: true.
            // Plastic w.large deformation or fluids will flag is_elastic to false, if needed.
            mnode->is_fluid = false;

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
                    double aabb_rad = mnode->GetHorizonRadius() / 2;  // to avoid too many pairs: bonding boxes hemisizes will sum..  __.__--*--
                    double coll_rad = mnode->GetCollisionRadius();
                    std::shared_ptr<ChCollisionShape> cshape;
                    if (mnode->is_boundary)
                        cshape = chrono_types::make_shared<ChCollisionShapeSphere>(matsurface, coll_rad);
                    else
                        cshape = chrono_types::make_shared<ChCollisionShapePoint>(matsurface, VNULL, coll_rad); // ..Point like ..Sphere, but no point-vs-point contact generation 
                    mnode->AddCollisionShape(cshape);
                    mnode->GetCollisionModel()->SetSafeMargin(coll_rad);
                    mnode->GetCollisionModel()->SetEnvelope(std::max(0.0, aabb_rad - coll_rad));
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

        }

        for (auto& nodedata : nodes) {
            nodedata.second.node->is_fluid = false;
        }
    };
    

    /// Set the material surface for 'boundary contact'.
    void SetContactMaterial(const std::shared_ptr<ChContactMaterial>& mnewsurf) { matsurface = mnewsurf; }

    /// Set the material surface for 'boundary contact'.
    std::shared_ptr<ChContactMaterial>& GetContactMaterial() { return matsurface; }


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




/// @} chrono_peridynamics

}  // end namespace peridynamics
}  // end namespace chrono

#endif
