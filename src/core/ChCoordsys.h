//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCOORDSYS_H
#define CHCOORDSYS_H

//////////////////////////////////////////////////
//  
//   ChCoordsys.h
//
//   Basic math functions for 3d coordinates (position
//   and rotation). 
//   For more advanced features, look into headers
//   ChFrame.h or ChFrameMoving.h.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "core/ChVector.h"
#include "core/ChQuaternion.h"


namespace chrono
{



///
/// COORDSYS:	
///
///  This class contains both traslational variable 
/// (the origin of the axis) and the rotational variable
/// (that is the unitary quaternion which represent the 
/// special-orthogonal transformation matrix).
///   Basic features for point-coordinate transformations
/// are provided. However, for more advanced features, the 
/// heavier classes ChFrame() or ChFrameMoving() may suit better.
///  The coordsys object comes either with the template "ChCoordsys<type>" mode,
/// either in the 'shortcut' flavour, that is "Coordsys", which assumes
/// the type of the four scalars is double precision, so it is faster to type.
///

template <class Real = double>
class ChCoordsys 
{
public:

			//
			// DATA
			//

	ChVector<Real>		pos;
	ChQuaternion<Real>  rot;

			//
			// CONSTRUCTORS
			//

	ChCoordsys(): pos(VNULL), rot(QUNIT) {};

	explicit ChCoordsys(const ChVector<Real> mv, const ChQuaternion<Real> mq = QUNIT): pos(mv), rot(mq) {};


					/// Copy constructor 
	ChCoordsys(const ChCoordsys<Real>& other) : pos(other.pos), rot(other.rot)  {};

			//
			// OPERATORS OVERLOADING
			//
					/// Assignment operator: copy from another coordsys
	ChCoordsys<Real>& operator=(const ChCoordsys<Real>& other)	
					{if (&other == this) return *this; pos = other.pos;  rot = other.rot; return *this; }

				
	bool operator<=(const ChCoordsys<Real>&other) const { return rot<=other.rot && pos<=other.pos;};
	bool operator>=(const ChCoordsys<Real>&other) const { return rot>=other.rot && pos>=other.pos;};

	bool operator==(const ChCoordsys<Real>& other) const { return rot==other.rot && pos==other.pos;}
	bool operator!=(const ChCoordsys<Real>& other) const { return rot!=other.rot || pos!=other.pos;}
	


			//
			// FUNCTIONS
			//

					/// Force to z=0, and z rotation only. No normalization to quaternion, however.
	void Force2D () 	{
							pos.z = 0;
							rot.e1 = 0;
							rot.e2 = 0;
						}

					/// Returns true if coordsys is identical to other coordsys
	bool	Equals ( const ChCoordsys<Real>& other) const { return rot.Equals(other.rot) && pos.Equals(other.pos);}

					/// Returns true if coordsys is equal to other coordsys, within a tolerance 'tol'
	bool	Equals ( const ChCoordsys<Real>& other, Real tol) const
						{
							return rot.Equals(other.rot, tol) && pos.Equals(other.pos, tol);
						}

					/// Sets to no translation and no rotation
	void    SetIdentity()
						{
							pos=VNULL; rot=QUNIT;
						}

				// POINT TRANSFORMATIONS, USING POSITION AND ROTATION QUATERNION

					/// This function transforms a point from the parent coordinate
					/// system to a local coordinate system, whose relative position 
					/// is given by this coodsys, i.e. 'origin' translation and 'alignment' quaternion.
					/// \return The point in local coordinate, as local=q'*[0,(parent-origin)]*q

	ChVector<Real> TrasformParentToLocal (
								const ChVector<Real>& parent		///< point to transform, given in parent coordinates
								) const
						{
							return rot.RotateBack(parent - pos);
						}

					/// This function transforms a point from the local coordinate
					/// system to the parent coordinate system. Relative position of local respect
					/// to parent is given by this coordys, i.e. 'origin' translation and 'alignment' quaternion.
					/// \return The point in parent coordinate, as parent=origin +q*[0,(local)]*q'

	ChVector<Real> TrasformLocalToParent (
								const ChVector<Real>& local			///< point to transform, given in local coordinates
								) const
						{
							return pos + rot.Rotate(local);
						}



			//
			// STREAMING
			//
					/// Method to allow serializing transient data into in ascii
					/// as a readable item, for example   "chrono::GetLog() << myobject;"
	void StreamOUT(ChStreamOutAscii& mstream)
						{
							mstream << "\n" << pos;
							mstream << "\n" << rot;
						}

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream)
						{
							mstream << pos;
							mstream << rot;
						}

					/// Operator to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	void StreamIN(ChStreamInBinary& mstream)
						{
							mstream >> pos;
							mstream >> rot;
						}


};



/// Shortcut for faster use of typical double-precision coordsys.
///  Instead of writing    "ChCoordsys<double> foo;"   you can write 
///  the shorter version   "Coordsys foo;"
///
typedef ChCoordsys<double> Coordsys;

/// Shortcut for faster use of typical single-precision coordsys.
///
typedef ChCoordsys<float>  CoordsysF;



//
// STATIC COORDSYS OPERATIONS
//

			/// Force 3d coordsys to lie on a XY plane (note: no normaliz. on quat)
ChApi 
Coordsys  Force2Dcsys (Coordsys* cs); 





//
// CONSTANTS
//


static const Coordsys CSYSNULL (VNULL,QNULL);
static const Coordsys CSYSNORM (VNULL,QUNIT); 



} // END_OF_NAMESPACE____




#endif  // END of ChCoordsys.h 
