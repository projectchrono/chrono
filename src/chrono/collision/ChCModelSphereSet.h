// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CHC_MODELSPHERESET_H
#define CHC_MODELSPHERESET_H

#include <vector>
#include "collision/ChCCollisionModel.h"
#include <thrust/host_vector.h>

namespace chrono {

class ChBody;

namespace collision {

class ChConvexDecomposition;

///  A wrapper to use the Bullet collision detection
///  library

class ChApi ChModelSphereSet : public ChCollisionModel {
  protected:
    uint nSpheres;
    thrust::host_vector<ChVector<float> > sphPosLocal;
    thrust::host_vector<ChVector<float> > sphPosGlobal;
    thrust::host_vector<float> sphRad;
    ChVector<float> myBBminLocal, myBBmaxLocal, myBBminGlobal, myBBmaxGlobal;

    int colFam;
    int noCollWith;

  public:
    ChModelSphereSet();
    virtual ~ChModelSphereSet();

    /// Deletes all inserted geometries.
    /// Also, if you begin the definition of a model, AFTER adding
    /// the geometric description, remember to call the ClearModel().
    /// MUST be inherited by child classes! (ex for resetting also BV hierarchies)
    virtual int ClearModel();

    /// Builds the BV hierarchy.
    /// Call this function AFTER adding the geometric description.
    /// MUST be inherited by child classes! (ex for bulding BV hierarchies)
    virtual int BuildModel();

    //
    // GEOMETRY DESCRIPTION
    //
    //  The following functions must be called inbetween
    //  the ClearModel() BuildModel() pair.

    /// Add a sphere shape to this model, for collision purposes
    virtual bool AddSphere(double radius, ChVector<>* pos = 0);

    virtual bool AddCompoundBody(int numSpheres, std::vector<ChVector<float> >* posLocal, std::vector<float>* rads);

    /// Add an ellipsoid shape to this model, for collision purposes
    virtual bool AddEllipsoid(double rx, double ry, double rz, ChVector<>* pos = 0, ChMatrix33<>* rot = 0);

    /// Add a box shape to this model, for collision purposes
    virtual bool AddBox(double hx, double hy, double hz, ChVector<>* pos = 0, ChMatrix33<>* rot = 0);

    /// Add a cylinder to this model (default axis on Y direction), for collision purposes
    virtual bool AddCylinder(double rx, double rz, double hy, ChVector<>* pos = 0, ChMatrix33<>* rot = 0);

    /// Add a convex hull to this model. A convex hull is simply a point cloud that describe
    /// a convex polytope. Connectivity between the vertexes, as faces/edges in triangle meshes is not necessary.
    /// Points are passed as a list, that is instantly copied into the model.
    virtual bool AddConvexHull(std::vector<ChVector<double> >& pointlist, ChVector<>* pos = 0, ChMatrix33<>* rot = 0);

    /// Add a triangle mesh to this model, passing a triangle mesh (do not delete the triangle mesh
    /// until the collision model, because depending on the implementation of inherited ChCollisionModel
    /// classes, maybe the triangle is referenced via a striding interface or just copied)
    /// Note: if possible, in sake of high performance, avoid triangle meshes and prefer simplified
    /// representations as compounds of convex shapes of boxes/spheres/etc.. type.
    virtual bool AddTriangleMesh(
        const geometry::ChTriangleMesh& trimesh,  ///< the triangle mesh
        bool is_static,  ///< true only if model doesn't move (es.a terrain). May improve performance
        bool is_convex,  ///< true if mesh is used as a convex hull(only for simple mesh), otherwise if false, handle as
        /// concave
        ChVector<>* pos = 0,
        ChMatrix33<>* rot = 0,  ///< displacement respect to COG (optional)
        double sphereswept_thickness = 0.0      ///< optional: outward sphereswept layer (when supported)
        );

    /// CUSTOM for this class only: add a concave triangle mesh that will be managed
    /// by GImpact mesh-mesh algorithm. Note that, despite this can work with
    /// arbitrary meshes, there could be issues of robustness and precision, so
    /// when possible, prefer simplified representations as compounds of convex
    /// shapes of boxes/spheres/etc.. type.
    virtual bool AddTriangleMeshConcave(const geometry::ChTriangleMesh& trimesh,  ///< the concave triangle mesh
                                        ChVector<>* pos = 0,   ///< displacement respect to COG (optional)
                                        ChMatrix33<>* rot = 0  ///< rotation respect to COG (optional)
                                        );
    /// CUSTOM for this class only: add a concave triangle mesh that will be decomposed
    /// into a compound of convex shapes. Decomposition could be more efficient than
    /// AddTriangleMeshConcave(), but preprocessing decomposition might take a while, and
    /// decomposition result is often approximate. Therefore, despite this can work with
    /// arbitrary meshes, there could be issues of robustness and precision, so
    /// when possible, prefer simplified representations as compounds of convex
    /// shapes of boxes/spheres/etc.. type.
    virtual bool AddTriangleMeshConcaveDecomposed(
        ChConvexDecomposition& mydecomposition,  ///< the concave triangle mesh, already decomposed
        ChVector<>* pos = 0,   ///< displacement respect to COG (optional)
        ChMatrix33<>* rot = 0  ///< rotation respect to COG (optional)
        );

    /// Add a barrel-like shape to this model (main axis on Y direction), for collision purposes.
    /// The barrel shape is made by lathing an arc of an ellipse around the vertical Y axis.
    /// The center of the ellipse is on Y=0 level, and it is ofsetted by R_offset from
    /// the Y axis in radial direction. The two radii of the ellipse are R_vert (for the
    /// vertical direction, i.e. the axis parellel to Y) andBullet R_hor (for the axis that
    /// is perpendicular to Y). Also, the solid is clamped with two discs on the top and
    /// the bottom, at levels Y_low and Y_high.
    virtual bool AddBarrel(double Y_low,
                           double Y_high,
                           double R_vert,
                           double R_hor,
                           double R_offset,
                           ChVector<>* pos = 0,
                           ChMatrix33<>* rot = 0);

    /// Add all shapes already contained in another model.
    /// Thank to the adoption of shared pointers, underlying shapes are
    /// shared (not copied) among the models; this will save memory when you must
    /// simulate thousands of objects with the same collision shape.
    /// The 'another' model must be of ChModelBullet subclass.
    virtual bool AddCopyOfAnotherModel(ChCollisionModel* another);

    virtual void SetFamily(int mfamily);
    virtual int GetFamily();
    virtual void SetFamilyMaskNoCollisionWithFamily(int mfamily);
    virtual void SetFamilyMaskDoCollisionWithFamily(int mfamily);
    virtual bool GetFamilyMaskDoesCollisionWithFamily(int mfamily);

    /// Returns the axis aligned bounding box (AABB) of the collision model,
    /// i.e. max-min along the x,y,z world axes. Remember that SyncPosition()
    /// should be invoked before calling this.
    virtual void GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const;

    //
    // STREAMING
    //

    /// Method to allow deserializing a persistent binary archive (ex: a file)
    /// into transient data.
    virtual void StreamIN(ChStreamInBinary& mstream);

    /// Method to allow serializing transient data into a persistent
    /// binary archive (ex: a file).
    virtual void StreamOUT(ChStreamOutBinary& mstream);

    //
    // CUSTOM SphereSet
    //

    /// Gets the global positions of the spheres
    void GetGlobalSpherePos(thrust::host_vector<ChVector<float> >& globalPos);

    /// Gets the global positions of the spheres
    void GetSphereRad(thrust::host_vector<float>& rad);

    uint getNSpheres() { return nSpheres; };
};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
