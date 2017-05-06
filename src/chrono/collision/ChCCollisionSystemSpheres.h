//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_COLLISIONSYSTEMSPHERES_H
#define CHC_COLLISIONSYSTEMSPHERES_H

//////////////////////////////////////////////////
//
//   ChCCollisionSystemSpheres.h
//
//   Header for class for collision engine based on
//   the CPU binning algorithm.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             http://www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////

#define THRUST_DEVICE_SYSTEM THRUST_DEVICE_SYSTEM_OMP

#include "core/ChApiCE.h"
#include "collision/ChCCollisionSystem.h"
#include "collision/ChCModelSphereSet.h"
#include <thrust/host_vector.h>
#include <thrust/sort.h>
#include <thrust/scan.h>
#include <thrust/unique.h>
#include <thrust/count.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/iterator/counting_iterator.h>

typedef chrono::ChVector<float> realV;
typedef float real;

#define Thrust_Inclusive_Scan_Sum(x, y)                    \
    thrust::inclusive_scan(x.begin(), x.end(), x.begin()); \
    y = x.back();
#define Thrust_Sort_By_Key(x, y) thrust::sort_by_key(x.begin(), x.end(), y.begin())
#define Thrust_Reduce_By_KeyA(x, y, z)                                                                              \
    x = thrust::reduce_by_key(y.begin(), y.end(), thrust::constant_iterator<uint>(1), y.begin(), z.begin()).first - \
        y.begin()
#define Thrust_Inclusive_Scan(x) thrust::inclusive_scan(x.begin(), x.end(), x.begin())
#define Thrust_Fill(x, y) thrust::fill(x.begin(), x.end(), y)
#define Thrust_Count(x, y) thrust::count(x.begin(), x.end(), y)

using namespace thrust;

namespace chrono {
namespace collision {

class ChCollisionSpheres;
class ChContacts;

///
/// Class for collision engine based on the 'Bullet' library.
/// Contains either the broadphase and the narrow phase Bullet
/// methods.
///

class ChApi ChCollisionSystemSpheres : public ChCollisionSystem {
  public:
    ChCollisionSystemSpheres(unsigned int max_objects = 16000, double scene_size = 500);
    virtual ~ChCollisionSystemSpheres();

    /// Clears all data instanced by this algorithm
    /// if any (like persistent contact manifolds)
    virtual void Clear(void);

    /// Adds a collision model to the collision
    /// engine (custom data may be allocated).
    virtual void Add(ChCollisionModel* model);

    /// Removes a collision model from the collision
    /// engine (custom data may be deallocated).
    virtual void Remove(ChCollisionModel* model);

    /// Removes all collision models from the collision
    /// engine (custom data may be deallocated).
    // virtual void RemoveAll();

    /// Run the algorithm and finds all the contacts.
    /// (Contacts will be managed by the Bullet persistent contact cache).
    virtual void Run();

    // Update...
    virtual void updateDataStructures();

    /// After the Run() has completed, you can call this function to
    /// fill a 'contact container', that is an object inherited from class
    /// ChContactContainer. For instance ChSystem, after each Run()
    /// collision detection, calls this method multiple times for all contact containers in the system,
    /// The basic behavior of the implementation is the following: collision system
    /// will call in sequence the functions BeginAddContact(), AddContact() (x n times),
    /// EndAddContact() of the contact container.
    virtual void ReportContacts(ChContactContainer* mcontactcontainer);

    /// After the Run() has completed, you can call this function to
    /// fill a 'proximity container' (container of narrow phase pairs), that is
    /// an object inherited from class ChProximityContainer. For instance ChSystem, after each Run()
    /// collision detection, calls this method multiple times for all proximity containers in the system,
    /// The basic behavior of the implementation is  the following: collision system
    /// will call in sequence the functions BeginAddProximities(), AddProximity() (x n times),
    /// EndAddProximities() of the proximity container.
    virtual void ReportProximities(ChProximityContainer* mproximitycontainer);

    /// Perform a raycast (ray-hit test with the collision models).
    virtual bool RayHit(const ChVector<>& from, const ChVector<>& to, ChRayhitResult& mresult);

    // Some internally used functions
    void host_Generate_AABB(const realV* pos, const real* radius, realV* aabb_data);
    void host_Count_AABB_BIN_Intersection(const realV* aabb_data, uint* Bins_Intersected);
    void host_Store_AABB_BIN_Intersection(const realV* aabb_data,
                                          const uint* Bins_Intersected,
                                          uint* bin_number,
                                          uint* body_number);
    void host_Count_AABB_AABB_Intersection(const realV* aabb_data,
                                           const uint* bin_number,
                                           const uint* body_number,
                                           const uint* bodyIndex,
                                           const uint* bin_start_index,
                                           const bool* active,
                                           uint* Num_ContactD);
    void host_Store_AABB_AABB_Intersection(const realV* aabb_data,
                                           const uint* bin_number,
                                           const uint* body_number,
                                           const uint* bodyIndex,
                                           const uint* bin_start_index,
                                           const uint* Num_ContactD,
                                           const bool* active,
                                           long long* potential_contacts);
    void host_Store_Contact(const long long* potential_contacts,
                            const realV* pos,
                            const float* radius,
                            const uint* bodyIndex,
                            uint* id_a,
                            uint* id_b,
                            realV* cpt_a,
                            realV* cpt_b,
                            realV* Norm,
                            float* c_dist,
                            float* rest_len,
                            uint* counter);

  public:
    thrust::host_vector<long long> potential_contacts;
    thrust::host_vector<realV> aabb_data;
    thrust::host_vector<uint> Bins_Intersected;
    thrust::host_vector<uint> bin_number;
    thrust::host_vector<uint> body_number;
    thrust::host_vector<uint> bin_start_index;
    thrust::host_vector<uint> Num_ContactD;

    ChCollisionSpheres* particle_list;
    ChContacts* contact_list;

  private:
    realV min_bounding_point;
    realV max_bounding_point;
    realV global_origin;
    realV bin_size_vec;
    realV bins_per_axis;

    uint number_of_particles, number_of_bodies;
    uint last_active_bin, number_of_bin_intersections, number_of_contacts_possible, number_of_contacts;
    uint val;

    thrust::host_vector<ChModelSphereSet*> collModels;
};

struct AABB {
    realV min, max;
};

typedef thrust::pair<realV, realV> bbox;

// reduce a pair of bounding boxes (a,b) to a bounding box containing a and b
struct bbox_reduction : public thrust::binary_function<bbox, bbox, bbox> {
    bbox operator()(bbox a, bbox b) {
        realV ll = realV(fmin(a.first.x, b.first.x), fmin(a.first.y, b.first.y),
                         fmin(a.first.z, b.first.z));  // lower left corner
        realV ur = realV(fmax(a.second.x, b.second.x), fmax(a.second.y, b.second.y),
                         fmax(a.second.z, b.second.z));  // upper right corner
        return bbox(ll, ur);
    }
};

// convert a point to a bbox containing that point, (point) -> (point, point)
struct bbox_transformation : public thrust::unary_function<realV, bbox> {
    bbox operator()(realV point) { return bbox(point, point); }
};

class ChApi ChCollisionSpheres {
  public:
    ChCollisionSpheres();
    virtual ~ChCollisionSpheres();
    void add(int bID, const thrust::host_vector<realV>& sPos, const thrust::host_vector<real>& sRad);
    void set(int bInd, int bIDnum, const thrust::host_vector<realV>& sPos, const thrust::host_vector<real>& sRad);
    void reset(int n_bodies, int n_particles);

  public:
    uint num_particles;
    uint num_bodies;
    uint setPos;

    thrust::host_vector<uint> bID;  // this should have length equal to num_bodies

    thrust::host_vector<realV> pos;    // this should have length equal to num particles
    thrust::host_vector<real> radius;  // this is mapped to pos, giving the radius for each particle
    thrust::host_vector<bool> active;  // this is mapped to pos,
    thrust::host_vector<uint> bodyID;  // this is mapped to pos, the ID number assigned when creating the body
    thrust::host_vector<uint>
        bodyIndex;  // this is mapped to pos, the index into the list of collModels that this sphere is a part of
};

class ChApi ChContacts {
  public:
    ChContacts();
    virtual ~ChContacts();
    void clear();
    void resize(uint);

  public:
    thrust::host_vector<uint> ida;
    thrust::host_vector<uint> idb;
    thrust::host_vector<realV> N;
    thrust::host_vector<real> depth;
    thrust::host_vector<real> rest_len;
    thrust::host_vector<realV> pta;
    thrust::host_vector<realV> ptb;
    uint num_contacts;
};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
