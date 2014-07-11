//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHEXTERNALOBJECT_H
#define CHEXTERNALOBJECT_H

//////////////////////////////////////////////////
//
//   ChExternalObject.h
//
//   Interface to external object (3rd party objects
//   in 3d modelers, for example).
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChMatrix.h"  // ***g++
#include "core/ChApiCE.h"


namespace chrono
{


// Forward references
namespace collision
{
class ChCollisionModel;
}
class ChBody;


///***OBSOLETE*** 
/// [This was used for the Realsoft3D version of Chrono, with GUI, 
///  but a more modern approach would be to have this object wrapper
///  as a ChAsset.] Will be removed in future releases.
///
///  This class is used by ChObj as an interface to external
/// object, that is 3rd party objects which may 'encapsulate'
/// chrono objects inherited by the ChObj class.
///  For example, a chrono ChBody object may be a property of a
/// geometric object in a 3D modeler (for example a R3OBJ* in case
/// of a plugin for the Realsoft3D software, or a SceneNode in case
/// of Irrlicht rendering engine, etc.)
///  Thank to this abstract interface, the chrono engine can 'talk'
/// to the external encapsulator.
///  Each custom implementation will provide the proper inherited version
/// of this class, for example there may be a ChExtrenalObjectIrrlicht
/// class, etc.
///

class ChApi ChExternalObject
{
	CH_RTTI_ROOT(ChExternalObject);

public:

			//
			// DATA
			//

	// No data in this 'default' reference, doing nothing...
	// 3rd party developers should derive from this class in order to
	// make references to external objects.

			//
			// CONSTRUCTORS
			//

	ChExternalObject() {};
	virtual ~ChExternalObject() {};

				/// Important!!! Derived classes MUST implement this
				/// function!
	virtual ChExternalObject* new_Duplicate() = 0;

			//
			// FUNCTIONS AND 'CALLBACKS'.
			//

				/// Interface to get the name of r3d-party encapsulator object,
				/// Derived classes may implement this.
	virtual char* GetName() {return NULL;};

				/// Interface to move the r3d-party encapsulator object.
				/// This function will be called each time a chrono object moves
				/// at the end of a frame simulation.
				/// Derived classes may implement this and take the proper actions.
	virtual void onChronoChanged() {};

				/// Interface used by ChProbe() objects to tell that Chrono has completed a
				/// simulation step, at time 'mtime', and recording of data
				/// into external probe object is required for that time instant.
				/// Derived classes may implement this and take the proper actions.
	virtual void onChronoProbeRecord(double mtime) {};

				/// Interface used by ChProbe() objects to tell that Chrono wants the
				/// recorded data in external probe object to be reset/deleted.
				/// Derived classes may implement this and take the proper actions.
	virtual void onChronoProbeReset() {};

				/// Interface to tell that Chrono wants the 3rd-party encapsulator
				/// object to add collision geometries (for example, in a plugin implementation,
				/// here a conversion to native 3d objects to chrono collision objects may take
				/// place.) The required level of detail of conversion is 'lod'.
	virtual void onAddCollisionGeometries(collision::ChCollisionModel* chmodel, ChBody* mbody, int lod, ChVector<double>* mt, ChMatrix33<double>* mr) {};

				/// Interface to access the N-th sub-object 
				/// (n-th child in tree hieararchy, if any).
				/// This _creates_ a ChExternalObject object enclosing the n-th child.
				/// If not success, or nc too high, return 0.
				/// Derived classes may implement this and take the proper actions.
	virtual ChExternalObject* GetNthChild(int nc) {return 0;};
	
				/// Given a name, _creates_ a ChExternalObject object enclosing
				/// an item whose name matches.
				/// If not success, returned 0.
				/// Derived classes may implement this and take the proper actions.
	virtual ChExternalObject* GetChildByName(char* mname) {return 0;};

				/// Returns position as ChCoordsys, if possible
	virtual ChCoordsys<> GetPos() {return CSYSNULL;}
				/// Set position, if possible
	virtual void SetPos(ChCoordsys<> mpos) {}

				/// Computes position at parametric u,v values, if it's a surface
	virtual ChVector<> Eval(double u, double v, double w) {return VNULL;}
				/// Computes normal at parametric u,v values, if it's a surface
	virtual ChVector<> Normal(double u, double v, double w) {return VNULL;}

};










} // END_OF_NAMESPACE____




#endif  // END of ChCoordsys.h
