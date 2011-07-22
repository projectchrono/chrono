#ifndef CHC_COLLISIONMODEL_H
#define CHC_COLLISIONMODEL_H
 
//////////////////////////////////////////////////
//  
//   ChCCollisionModel.h
//
//   The collision model class. Each body in the
//   simulation may have a collision model, defining
//   the shape for collision detection.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <vector>
#include "core/ChCoordsys.h"
#include "core/ChMatrix.h"
#include "core/ChApiCE.h"

#include "geometry/ChCTriangleMesh.h"



namespace chrono 
{

// forward references
class ChPhysicsItem;


namespace collision 
{
/// Shape types that can be created. Used so that shape type can be determined without going to bullet
/// Both GPU and CPU Collision Models use this enum
enum ShapeType
{
	SPHERE,
	ELLIPSOID, 
	BOX, 
	CYLINDER, 
	CONVEXHULL,
	TRIANGLEMESH,
	BARREL,
	RECT,				//Currently implemented on GPU only
	DISC,				//Currently implemented on GPU only
	ELLIPSE,			//Currently implemented on GPU only
	CAPSULE,			//Currently implemented on GPU only
	CONE,				//Currently implemented on GPU only
	COMPOUND			//Currently implemented on GPU only
};


///
/// Class containing the geometric model ready for collision detection.
/// Each rigid body will have a ChCollisionModel.
/// A ChCollisionModel will contain all the geometric description(s) 
/// of the shape of the rigid body, for collision purposes.
///

class ChApi ChCollisionModel
{


public:

  ChCollisionModel();

  virtual ~ChCollisionModel() {};
  

		/// Deletes all inserted geometries.
		/// Also, if you begin the definition of a model, AFTER adding
		/// the geometric description, remember to call the ClearModel().
		/// MUST be inherited by child classes! (ex for resetting also BV hierarchies)
  virtual int ClearModel() = 0;

		/// Builds the BV hierarchy.
		/// Call this function AFTER adding the geometric description.
		/// MUST be inherited by child classes! (ex for bulding BV hierarchies)
  virtual int BuildModel() = 0;

  
	// 
	// GEOMETRY DESCRIPTION	
	//	
	//  The following functions must be called inbetween 
	//  the ClearModel() BuildModel() pair.
	//  The class must implement automatic deletion of the created 
	//  geometries at class destruction time and at ClearModel()
	//  Return value is true if the child class implements the
	//  corresponding type of geometry.

		/// Add a sphere shape to this model, for collision purposes
  virtual bool AddSphere   (double radius,			///< the radius of the sphere
							ChVector<>* pos=0		///< the position of the sphere in model coordinates
							)=0;		

  		/// Add an ellipsoid shape to this model, for collision purposes
  virtual bool AddEllipsoid   (double rx, 			///< the rad on x axis
							   double ry,			///< the rad on y axis 
							   double rz,			///< the rad on z axis
							   ChVector<>* pos=0,	///< the position of the ellipsoid
							   ChMatrix33<>* rot=0	///< the matrix defining rotation (orthogonal) 
							   )=0;

		/// Add a box shape to this model, for collision purposes
  virtual bool AddBox      (double hx, 				///< the halfsize on x axis
							double hy,				///< the halfsize on y axis 
							double hz,				///< the halfsize on z axis
							ChVector<>* pos=0,		///< the position of the box COG
							ChMatrix33<>* rot=0		///< the rotation of the box - matrix must be orthogonal
							)=0;

		/// Add a cylinder to this model (default axis on Y direction), for collision purposes
  virtual bool AddCylinder (double rx, double rz, double hy, ChVector<>* pos=0, ChMatrix33<>* rot=0)=0;
		
		/// Add a convex hull to this model. A convex hull is simply a point cloud that describe
		/// a convex polytope. Connectivity between the vertexes, as faces/edges in triangle meshes is not necessary.
		/// Points are passed as a list, that is instantly copied into the model.
  virtual bool AddConvexHull (std::vector<ChVector<double> >& pointlist, ChVector<>* pos=0, ChMatrix33<>* rot=0)=0;

		/// Add a triangle mesh to this model, passing a triangle mesh (do not delete the triangle mesh
		/// until the collision model, because depending on the implementation of inherited ChCollisionModel
		/// classes, maybe the triangle is referenced via a striding interface or just copied)
		/// Note: if possible, in sake of high performance, avoid triangle meshes and prefer simplified 
		/// representations as compounds of convex shapes of boxes/spheres/etc.. type. See functions above.
  virtual bool AddTriangleMesh (const geometry::ChTriangleMesh& trimesh,	///< the triangle mesh
								bool is_static,			///< true only if model doesn't move (es.a terrain). May improve performance
								bool is_convex,			///< true if mesh convex hull is used (only for simple mesh). May improve robustness
								ChVector<>* pos=0, ChMatrix33<>* rot=0 ///< displacement respect to COG (optional)
								)=0;  

  		/// Add a barrel-like shape to this model (main axis on Y direction), for collision purposes.
		/// The barrel shape is made by lathing an arc of an ellipse around the vertical Y axis.
		/// The center of the ellipse is on Y=0 level, and it is ofsetted by R_offset from 
		/// the Y axis in radial direction. The two radii of the ellipse are R_vert (for the 
		/// vertical direction, i.e. the axis parellel to Y) and R_hor (for the axis that
		/// is perpendicular to Y). Also, the solid is clamped with two discs on the top and
		/// the bottom, at levels Y_low and Y_high. 
  virtual bool AddBarrel (double Y_low, double Y_high, double R_vert, double R_hor, double R_offset, ChVector<>* pos=0, ChMatrix33<>* rot=0)=0;

		/// Add all shapes already contained in another model.
		/// If possible, child classes implement this so that underlying shapes are 
		/// shared (not copied) among the models.
  virtual bool AddCopyOfAnotherModel (ChCollisionModel* another) = 0;

		/// Add a cluster of convex hulls by a '.chulls' file description. The file is an ascii text that contains
		/// many lines with "[x] [y] [z]" coordinates of the convex hulls. Hulls are separated by lines with "hull".
		/// Inherited classes should not need to implement/overload this, because this base implementation 
		/// basically calls AddConvexHull() n times while parsing the file, that is enough.
  virtual bool AddConvexHullsFromFile(ChStreamInAscii& mstream, ChVector<>* pos=0, ChMatrix33<>* rot=0);


  // OTHER FUNCTIONS
  //


  		/// Gets the pointer to the client owner ChPhysicsItem. 
		/// MUST be implemented by child classes!
  virtual ChPhysicsItem* GetPhysicsItem() = 0;

		/// Sets the position and orientation of the collision
		/// model as the rigid body current position.
		/// MUST be implemented by child classes!
  virtual void SyncPosition()=0;

		/// By default, all collsion objects belong to family n.0, 
		/// but you can set family in range 0..15. This is used when
		/// the objects collided with another: the contact is created
		/// only if the family is within the 'family mask' of the other,
		/// and viceversa.
		/// NOTE: these functions have NO effect if used before you add 
		///  the body to a ChSystem, using AddBody(). Use after AddBody(). 
		/// MUST be implemented by child classes!
  virtual void SetFamily(int mfamily)=0;
  virtual int  GetFamily()=0;

		/// By default, family mask is all turned on, so all families
		/// can collide with this object, but you can turn on-off some bytes
		/// of this mask so that some families do not collide.
		/// When two objects collide, the contact is created
		/// only if the family is within the 'family mask' of the other,
		/// and viceversa.
		/// NOTE: these functions have NO effect if used before you add 
		///  the body to a ChSystem, using AddBody(). Use after AddBody(). 
		/// MUST be implemented by child classes!
  virtual void SetFamilyMaskNoCollisionWithFamily(int mfamily)=0;
  virtual void SetFamilyMaskDoCollisionWithFamily(int mfamily)=0;

		/// Tells if the family mask of this collision object allows
		/// for the collision with another collision object belonging to 
		/// a given family.
		/// NOTE: this function has NO effect if used before you add 
		///  the body to a ChSystem, using AddBody(). Use after AddBody(). 
  virtual bool GetFamilyMaskDoesCollisionWithFamily(int mfamily)=0;

	
	// TOLERANCES, ENVELOPES, THRESHOLDS
	//

		/// Sets the suggested collision 'inward safe margin' for the 
		/// shapes to be added from now on, using the AddBox,
		/// AddCylinder etc (where, if this margin is too high for some
		/// thin or small shapes, it may be clamped).. 
		/// If dist<0 and interpenetation occurs (ex.for numerical errors) within 
		/// this 'safe margin' inward range, collision detection is still fast 
		/// and reliable (beyond this, for deep penetrations, CD still works, 
		/// but might be slower and less reliable)
		/// Side effect: think at the margin as a radius of a 'smoothing' fillet
		/// on all corners of the shapes - that's why you cannot exceed with this...
  virtual void SetSafeMargin(double amargin) 
		{
			model_safe_margin = (float)amargin;
		}
		/// Returns the inward safe margin (see SetSafeMargin() )
  virtual float GetSafeMargin()
		{
			return model_safe_margin;
		}

		/// Sets the suggested collision outward 'envelope' (used from shapes
		/// added, from now on, to this collision model).  This 'envelope' is a
		/// surrounding invisible volume which extends outward from the 
		/// surface, and it is used to detect contacts a bit before shapes
		/// come into contact, i.e. when dist>0. However contact points will stay
		/// on the true surface of the geometry, not on the external surface of the 
		/// envelope.
		/// Side effect: AABB are 'expanded' outward by this amount, so if you
		/// exagerate with this value, CD might be slower and too sensible.
		/// On the other hand, if you set this value to 0, contacts are detected
		/// only for dist<=0, thus causing unstable simulation.
  virtual void SetEnvelope(double amargin) 
		{
			model_envelope = (float)amargin;
		}
		/// Returns the outward safe margin (see SetEnvelope() )
  virtual float GetEnvelope()
		{
			return model_envelope;
		}

		/// Returns the Type of Shape 
  virtual ShapeType GetShapeType()
		{
			return model_type;
		}

  static void SetDefaultSuggestedEnvelope(double menv);
  static void SetDefaultSuggestedMargin(double mmargin);
  static double GetDefaultSuggestedEnvelope();
  static double GetDefaultSuggestedMargin();

		/// Returns the axis aligned bounding box (AABB) of the collision model,
		/// i.e. max-min along the x,y,z world axes. Remember that SyncPosition()
		/// should be invoked before calling this.
		/// MUST be implemented by child classes! 
  virtual void GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const = 0;

  	
			//
			// STREAMING
			//

		/// Method to allow deserializing a persistent binary archive (ex: a file)
		/// into transient data.
  virtual void StreamIN(ChStreamInBinary& mstream);

		/// Method to allow serializing transient data into a persistent
		/// binary archive (ex: a file).
  virtual void StreamOUT(ChStreamOutBinary& mstream);


protected:

	virtual float GetSuggestedFullMargin() 
		{
			return model_envelope + model_safe_margin;
		}			
				// Maximum envelope: surrounding volume from surface
				// to the exterior
	float model_envelope;

				// This is the max.value to be used for fast penetration 
				// contact detection.
	float model_safe_margin;

				// This is the type of shape used for collision model
	ShapeType model_type;

};






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
