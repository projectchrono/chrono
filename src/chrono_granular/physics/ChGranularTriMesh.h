// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dan Negrut
// =============================================================================
/*! \file */

#pragma once

//#include "chrono_granular/ChGranularDefines.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/utils/ChGranularUtilities.h"
#include "chrono_thirdparty/tinyobjloader/tiny_obj_loader.h"

namespace chrono {
namespace granular {

/**
 *\brief Template class used as a place holder for arrays associated with a mesh. No memory
 * allocation of freeing done by objects of this class. All its members are public.
 *
 *\attention  The order of the nodes in a triangle defines the positive face of the triangle; right-hand rule used.
 *\attention Some other agent needs to allocate/deallocate memory pointed to by variables in this class
 *
 */
template <class T>
class ChTriangleSoup {
  public:
    unsigned int nTrianglesInSoup;    //!< total number of triangles in the soup
    unsigned int nFamiliesInSoup;     //!< indicates how many meshes are squashed together in this soup
    unsigned int* triangleFamily_ID;  //!< each entry says what family that triagnle belongs to; size: nTrianglesInSoup

    T* node1_X;  //!< X position in global reference frame of node 1
    T* node1_Y;  //!< Y position in global reference frame of node 1
    T* node1_Z;  //!< Z position in global reference frame of node 1

    T* node2_X;  //!< X position in global reference frame of node 2
    T* node2_Y;  //!< Y position in global reference frame of node 2
    T* node2_Z;  //!< Z position in global reference frame of node 2

    T* node3_X;  //!< X position in global reference frame of node 3
    T* node3_Y;  //!< Y position in global reference frame of node 3
    T* node3_Z;  //!< Z position in global reference frame of node 3

    float* node1_XDOT;  //!< X velocity in global reference frame of node 1
    float* node1_YDOT;  //!< Y velocity in global reference frame of node 1
    float* node1_ZDOT;  //!< Z velocity in global reference frame of node 1

    float* node2_XDOT;  //!< X velocity in global reference frame of node 2
    float* node2_YDOT;  //!< Y velocity in global reference frame of node 2
    float* node2_ZDOT;  //!< Z velocity in global reference frame of node 2

    float* node3_XDOT;  //!< X velocity in global reference frame of node 3
    float* node3_YDOT;  //!< Y velocity in global reference frame of node 3
    float* node3_ZDOT;  //!< Z velocity in global reference frame of node 3

    float* generalizedForcesPerFamily;  //!< Generalized forces acting on each family. Expressed
                                        //!< in the global reference frame. Size: 6 * TRIANGLE_FAMILIES.
};

/** \brief Class implements functionality required to handle the interaction between a mesh soup and granular material.
 *
 * Mesh soup: a collection of meshes that each has a certain number of triangle elements. For instance, the meshes
 * associated with the four wheels of a rover operating on granular material would be smashed into one soup having four
 * mesh families.
 *
 * Assumptions: Mono-disperse setup, one radius for all spheres. There is no friction. There can be adehsion.
 * The granular material interacts through an implement that is defined via a triangular mesh.
 */
class CH_GRANULAR_API ChSystemGranularMonodisperse_SMC_Frictionless_trimesh
    : public ChSystemGranularMonodisperse_SMC_Frictionless {
  public:
    void run_simulation(float t_end);
    void advance_simulation(float duration);

  private:
    ChTriangleSoup<float> meshSoup_HOST;  //!< clean copy of mesh soup interacting with granular material; HOST-side
    ChTriangleSoup<float>
        meshSoupWorking_HOST;             //!< working copy of mesh soup interacting with granular material; HOST-side
    ChTriangleSoup<int> meshSoup_DEVICE;  //!< mesh soup interacting with granular material; DEVICE-side
    double YoungModulus_SPH2MESH;  //!< the stiffness associated w/ contact between a mesh element and gran material
    float K_n_s2m_SU;  //!< size of the normal stiffness (SU) for sphere-to-mesh contact; expressed in sim. units
    bool problemSetupFinished;
    float timeToWhichDEsHaveBeenPropagated;

    // Function members
    void copy_triangle_data_to_device();

    // Mesh simulations should not have moving BD frames for now
    virtual void copyBD_Frame_to_device() { NOT_IMPLEMENTED_YET; }

    void setupSoup_HOST_DEVICE(const char* meshFileName);
    void setupSoup_HOST(const std::vector<tinyobj::shape_t>& soup, unsigned int nTriangles);
    void cleanupSoup_HOST();
    void setupSoup_DEVICE(unsigned int nTriangles);
    void cleanupSoup_DEVICE();
    void update_DMeshSoup_Location();

    // void initialize();

    virtual double get_max_K();

    void setupSimulation();

  public:
    ChSystemGranularMonodisperse_SMC_Frictionless_trimesh(float radiusSPH, float density, std::string meshFileName);
    virtual ~ChSystemGranularMonodisperse_SMC_Frictionless_trimesh();

    inline void set_YoungModulus_SPH2IMPLEMENT(double someValue) { YoungModulus_SPH2MESH = someValue; }
    inline unsigned int nMeshesInSoup() const { return meshSoup_HOST.nFamiliesInSoup; }
    void collectGeneralizedForcesOnMeshSoup(float crntTime, float* genForcesOnSoup);
    void meshSoup_applyRigidBodyMotion(float crntTime, double* position_orientation_data);
};

}  // namespace granular
}  // namespace chrono
