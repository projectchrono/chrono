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


#pragma once

//#include "chrono_granular/ChGranularDefines.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/utils/ChGranularUtilities.h"
#include "chrono_thirdparty/tinyobjloader/tiny_obj_loader.h"

namespace chrono {
namespace granular {

/**
 * ChTriangleSoup: thin helper class used as a place holder for arrays associated with a mesh. No memory
 * allocation of freeing done by objects of this class. All its members are public.
 */
class ChTriangleSoup {
  public:
    unsigned int nTrianglesInSoup;  /// total number of triangles in the soup
    unsigned int nFamiliesInSoup;
    unsigned int* triangleFamily_ID;  /// each entry says what family that triagnle belongs to; size: nTrianglesInSoup

    /// The order of the nodes in a triangle defines the positive face of the triangle; use right-hand rule
    int* node1_X;  /// X position in global reference frame of node 1
    int* node1_Y;  /// Y position in global reference frame of node 1
    int* node1_Z;  /// Z position in global reference frame of node 1

    int* node2_X;  /// X position in global reference frame of node 2
    int* node2_Y;  /// Y position in global reference frame of node 2
    int* node2_Z;  /// Z position in global reference frame of node 2

    int* node3_X;  /// X position in global reference frame of node 3
    int* node3_Y;  /// Y position in global reference frame of node 3
    int* node3_Z;  /// Z position in global reference frame of node 3

    float* node1_XDOT;  /// X velocity in global reference frame of node 1
    float* node1_YDOT;  /// Y velocity in global reference frame of node 1
    float* node1_ZDOT;  /// Z velocity in global reference frame of node 1

    float* node2_XDOT;  /// X velocity in global reference frame of node 2
    float* node2_YDOT;  /// Y velocity in global reference frame of node 2
    float* node2_ZDOT;  /// Z velocity in global reference frame of node 2

    float* node3_XDOT;  /// X velocity in global reference frame of node 3
    float* node3_YDOT;  /// Y velocity in global reference frame of node 3
    float* node3_ZDOT;  /// Z velocity in global reference frame of node 3

    float* generalizedForcesPerFamily;  //!< Generalized forces acting on each family. Expressed
                                        //!< in the global reference frame. Size: 6 * TRIANGLE_FAMILIES.
};

/**
 * ChSystemGranularMonodisperse_SMC_Frictionless_trimesh: Mono-disperse setup, one radius for all spheres. There is no
 * friction. The granular material interacts through an implement that is defined via a triangular mesh.
 */
class CH_GRANULAR_API ChSystemGranularMonodisperse_SMC_Frictionless_trimesh {
  private:
    /// Data members
    ChSystemGranularMonodisperse_SMC_Frictionless granMat;
    ChTriangleSoup meshSoup_HOST;    /// the mesh soup that interacts with the many-body problem
    ChTriangleSoup meshSoup_DEVICE;  /// the mesh soup that interacts with the many-body problem
    double YoungModulus_SPH2MESH;
    float K_n_s2m_SU;  /// size of the normal stiffness (SU) for sphere-to-mesh contact

    /// Function members
    void copy_const_data_to_device();

    void switch_to_SimUnits();

    void cleanup_simulation() { NOT_IMPLEMENTED_YET }
    void determine_new_stepSize() { NOT_IMPLEMENTED_YET }
    void setupSoup_HOST_DEVICE(const char* meshFileName);
    void SetupSoup_HOST(const std::vector<tinyobj::shape_t>& soup, unsigned int nTriangles);
    void CleanupSoup_HOST();
    void SetupSoup_DEVICE(unsigned int nTriangles);
    void CleanupSoup_DEVICE();

  public:
    ChSystemGranularMonodisperse_SMC_Frictionless_trimesh(float radiusSPH, float density, const char* meshFileName);
    ~ChSystemGranularMonodisperse_SMC_Frictionless_trimesh();

    virtual void setup_simulation() {
        NOT_IMPLEMENTED_YET
    }  //!< set up data structures and carry out pre-processing tasks
    virtual void run_simulation(float t_end);
    virtual void advance_simulation(float duration) { NOT_IMPLEMENTED_YET }

    inline void set_YoungModulus_SPH2IMPLEMENT(double someValue) { YoungModulus_SPH2MESH = someValue; }
    inline ChSystemGranularMonodisperse_SMC_Frictionless& granMatBed() { return granMat; }
    void updateMeshSoup_Location_GeneralizedForces(ChTriangleSoup& outsideSoup) { NOT_IMPLEMENTED_YET }
};



}  // namespace granular
}  // namespace chrono