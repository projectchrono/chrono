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
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: class for a parallel collision model
// =============================================================================

#ifndef CHCCOLLISIONMODELPARALLEL_H
#define CHCCOLLISIONMODELPARALLEL_H

#include "collision/ChCCollisionModel.h"

#include "chrono_parallel/ChApiParallel.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/ChParallelMath.h"

namespace chrono {
// forward references
class ChBody;

namespace collision {
///  A wrapper to uses GPU collision detection

class CH_PARALLEL_API ChCollisionModelParallel : public ChCollisionModel {
 public:

   ChCollisionModelParallel();
   virtual ~ChCollisionModelParallel();

   /// Deletes all inserted geometries.
   /// Also, if you begin the definition of a model, AFTER adding
   /// the geometric description, remember to call the ClearModel().
   /// MUST be inherited by child classes! (ex for resetting also BV hierarchies)
   virtual int ClearModel();

   /// Builds the BV hierarchy.
   /// Call this function AFTER adding the geometric description.
   /// MUST be inherited by child classes! (ex for bulding BV hierarchies)
   virtual int BuildModel();

   /// Sets the position and orientation of the collision
   /// model as the rigid body current position.
   virtual void SyncPosition();

   /// Gets the pointer to the client owner ChPhysicsItem.
   virtual ChPhysicsItem *GetPhysicsItem();
   //
   // GEOMETRY DESCRIPTION
   //
   //  The following functions must be called inbetween
   //  the ClearModel() BuildModel() pair.

   /// Add a sphere shape to this model, for collision purposes
   virtual bool AddSphere(double radius,
                          const ChVector<> &pos = ChVector<>());

   /// Add an ellipsoid shape to this model, for collision purposes
   virtual bool AddEllipsoid(double rx,
                             double ry,
                             double rz,
                             const ChVector<> &pos = ChVector<>(),
                             const ChMatrix33<> &rot = ChMatrix33<>(1));

   /// Add a box shape to this model, for collision purposes
   virtual bool AddBox(double hx,
                       double hy,
                       double hz,
                       const ChVector<> &pos = ChVector<>(),
                       const ChMatrix33<> &rot = ChMatrix33<>(1));

   /// Add a rounded box shape to this model, for collision purposes
   virtual bool AddRoundedBox(double hx,
                              double hy,
                              double hz,
                              double sphere_r,
                              const ChVector<> &pos = ChVector<>(),
                              const ChMatrix33<> &rot = ChMatrix33<>(1));

   /// Add a triangle shape to this model, for collision purposes
   virtual bool AddTriangle(ChVector<> A,
                            ChVector<> B,
                            ChVector<> C,
                            const ChVector<> &pos = ChVector<>(),
                            const ChMatrix33<> &rot = ChMatrix33<>(1));

   /// Add a cylinder to this model (default axis on Y direction), for collision purposes
   virtual bool AddCylinder(double rx,
                            double ry,
                            double rz,
                            const ChVector<> &pos = ChVector<>(),
                            const ChMatrix33<> &rot = ChMatrix33<>(1));

   /// Add a rounded cylinder to this model (default axis on Y direction), for collision purposes
   virtual bool AddRoundedCylinder(double rx,
                                   double rz,
                                   double hy,
                                   double sphere_r,
                                   const ChVector<> &pos = ChVector<>(),
                                   const ChMatrix33<> &rot = ChMatrix33<>(1));

   /// Add a cone to this model (default axis on Y direction), for collision purposes
   virtual bool AddCone(double rx,
                        double rz,
                        double hy,
                        const ChVector<>& pos = ChVector<>(),
                        const ChMatrix33<>& rot = ChMatrix33<>(1));

   /// Add a rounded cone to this model (default axis on Y direction), for collision purposes
   virtual bool AddRoundedCone(double rx,
                               double rz,
                               double hy,
                               double sphere_r,
                               const ChVector<> &pos = ChVector<>(),
                               const ChMatrix33<> &rot = ChMatrix33<>(1));

   /// Add a capsule to this model (default axis in Y direction), for collision purposes
   virtual bool AddCapsule(double radius,
                           double hlen,
                           const ChVector<> &pos = ChVector<>(),
                           const ChMatrix33<> &rot = ChMatrix33<>(1));

   virtual bool AddConvexHull(std::vector<ChVector<double> > &pointlist,
                              const ChVector<>& pos = ChVector<>(),
                              const ChMatrix33<>& rot = ChMatrix33<>(1));

   /// Add a triangle mesh to this model, passing a triangle mesh (do not delete the triangle mesh
   /// until the collision model, because depending on the implementation of inherited ChCollisionModel
   /// classes, maybe the triangle is referenced via a striding interface or just copied)
   /// Note: if possible, in sake of high performance, avoid triangle meshes and prefer simplified
   /// representations as compounds of convex shapes of boxes/spheres/etc.. type.
   virtual bool AddTriangleMesh(const geometry::ChTriangleMesh &trimesh,                ///< the triangle mesh
                                bool is_static,              ///< true only if model doesn't move (es.a terrain). May improve performance
                                bool is_convex,              ///< true if mesh convex hull is used (only for simple mesh). May improve robustness
                                const ChVector<> &pos = ChVector<>(),     ///< displacement respect to COG (optional)
                                const ChMatrix33<> &rot = ChMatrix33<>(1)   ///< the rotation of the mesh - matrix must be orthogonal
                                      );

   /// Add a barrel-like shape to this model (main axis on Y direction), for collision purposes.
   /// The barrel shape is made by lathing an arc of an ellipse around the vertical Y axis.
   /// The center of the ellipse is on Y=0 level, and it is ofsetted by R_offset from
   /// the Y axis in radial direction. The two radii of the ellipse are R_vert (for the
   /// vertical direction, i.e. the axis parellel to Y) and R_hor (for the axis that
   /// is perpendicular to Y). Also, the solid is clamped with two discs on the top and
   /// the bottom, at levels Y_low and Y_high.
   virtual bool AddBarrel(double Y_low,
                          double Y_high,
                          double R_vert,
                          double R_hor,
                          double R_offset,
                          const ChVector<>& pos = ChVector<>(),
                          const ChMatrix33<>& rot = ChMatrix33<>(1));

   /// Add all shapes already contained in another model.
   /// Thank to the adoption of shared pointers, underlying shapes are
   /// shared (not copied) among the models; this will save memory when you must
   /// simulate thousands of objects with the same collision shape.
   /// The 'another' model must be of ChModelBullet subclass.
   virtual bool AddCopyOfAnotherModel(ChCollisionModel *another);

   /// Return the axis aligned bounding box for this collision model.
   virtual void GetAABB(ChVector<> &bbmin,
                        ChVector<> &bbmax) const;

   /// Set the collision family for this object.
   virtual void SetFamily(int mfamily);
   /// Get the collision family for this object.
   virtual int GetFamily();
   /// Set the family mask to disable collision with the specified family.
   virtual void SetFamilyMaskNoCollisionWithFamily(int mfamily);
   /// Set the family mask to enable collision with the specified family.
   virtual void SetFamilyMaskDoCollisionWithFamily(int mfamily);
   /// Check if the model collides with the specified family.
   virtual bool GetFamilyMaskDoesCollisionWithFamily(int mfamily);

   /// Return the collision family group of this model.
   /// The collision family of this model is the position of the single set bit
   /// in the return value.
   short GetFamilyGroup() const { return family_group; }
   /// Set the collision family group of this model.
   /// This is an alternative way of specifying the collision family for this
   /// object.  The value family_group must have a single bit set (i.e. it must
   /// be a power of 2). The corresponding family is then the bit position.
   void SetFamilyGroup(short group);

   /// Return the collision mask for this model.
   /// Each bit of the return value indicates whether this model collides with
   /// the corresponding family (bit set) or not (bit unset).
   short GetFamilyMask() const { return family_mask; }
   /// Set the collision mask for this model.
   /// Any set bit in the specified mask indicates that this model collides with
   /// all objects whose family is equal to the bit position.
   void SetFamilyMask(short mask);

   /// Return a pointer to the body
   ChBody *GetBody() const { return mbody; }

   /// Return the number of objects in this model.
   int GetNObjects() const { return nObjects; }

   float getVolume();

   struct bData {
      real3 A;
      real3 B;
      real3 C;
      real4 R;
      int type;
   };

   std::vector<bData> mData;
   std::vector<real3> local_convex_data;

 protected:

   unsigned int nObjects;
   short family_group;
   short family_mask;
   real3 inertia;
   float total_volume;
};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____
#endif

