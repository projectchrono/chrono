#include "chrono_granular/physics/ChGranular.h"

#pragma once

/**
* ChTriangleSoup: thin helper class used as a place holder for arrays associated with a mesh. No memory
* allocation of freeing done by objects of this class. All its members are public.
*/
template <unsigned int TRIANGLE_FAMILIES> class ChTriangleSoup {
public:
    unsigned int nTrianglesInSoup; /// total number of triangles in the soup

    unsigned int* triangleFamily_ID;  /// each entry says what family that triagnle belongs to; size: nTrianglesInSoup

                                      /// The order of the nodes in a triangle defines the positive face of the triangle; use right-hand rule
    int* node1_X; /// X position in global reference frame of node 1
    int* node1_Y; /// Y position in global reference frame of node 1
    int* node1_Z; /// Z position in global reference frame of node 1

    int* node2_X; /// X position in global reference frame of node 2
    int* node2_Y; /// Y position in global reference frame of node 2
    int* node2_Z; /// Z position in global reference frame of node 2

    int* node3_X; /// X position in global reference frame of node 3
    int* node3_Y; /// Y position in global reference frame of node 3
    int* node3_Z; /// Z position in global reference frame of node 3

    float* node1_XDOT; /// X velocity in global reference frame of node 1
    float* node1_YDOT; /// Y velocity in global reference frame of node 1
    float* node1_ZDOT; /// Z velocity in global reference frame of node 1

    float* node2_XDOT; /// X velocity in global reference frame of node 2
    float* node2_YDOT; /// Y velocity in global reference frame of node 2
    float* node2_ZDOT; /// Z velocity in global reference frame of node 2

    float* node3_XDOT; /// X velocity in global reference frame of node 3
    float* node3_YDOT; /// Y velocity in global reference frame of node 3
    float* node3_ZDOT; /// Z velocity in global reference frame of node 3

    float generalizedForcesPerFamily[6 * TRIANGLE_FAMILIES];  //!< Generalized forces acting on each family. Expressed
                                                              //!< in the global reference frame.
};

/**
* ChSystemGranularMonodisperse_SMC_Frictionless_trimesh: Mono-disperse setup, one radius for all spheres. There is no
* friction. The granular material interacts through an implement that is defined via a triangular mesh.
*/
template <unsigned int TRIANGLE_FAMILIES> class CH_GRANULAR_API ChSystemGranularMonodisperse_SMC_Frictionless_trimesh
    : public ChSystemGranularMonodisperse_SMC_Frictionless {
protected:
    virtual void copy_const_data_to_device();
    virtual void resetBroadphaseInformation() { NOT_IMPLEMENTED_YET }

    virtual void switch_to_SimUnits();

    virtual void cleanup_simulation() { NOT_IMPLEMENTED_YET }
    virtual void determine_new_stepSize() { return; }

    double YoungModulus_SPH2MESH;
    float K_n_s2m_SU; /// size of the normal stiffness (SU) for sphere-to-mesh contact

    ChTriangleSoup<TRIANGLE_FAMILIES> meshSoup; /// the mesh soup that interacts with the many-body problem


public:
    ChSystemGranularMonodisperse_SMC_Frictionless_trimesh(float radiusSPH, float density)
        : ChSystemGranularMonodisperse_SMC_Frictionless(radiusSPH, density) {}

    ~ChSystemGranularMonodisperse_SMC_Frictionless_trimesh() {}

    virtual void setup_simulation() { NOT_IMPLEMENTED_YET }  //!< set up data structures and carry out pre-processing tasks
    virtual void run_simulation(float t_end);
    virtual void advance_simulation(float duration) { NOT_IMPLEMENTED_YET }

    inline void set_YoungModulus_SPH2IMPLEMENT(double someValue) { YoungModulus_SPH2MESH = someValue; }
};


//template <unsigned int TRIANGLE_FAMILIES>
//chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh<
//    TRIANGLE_FAMILIES>::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh<TRIANGLE_FAMILIES>(
//    float radiusSPH, float density, const char *meshSoupFileName)
//    : ChSystemGranularMonodisperse_SMC_Frictionless(radiusSPH, density) {}
