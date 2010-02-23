#ifndef CHFRAMEMOVING_H
#define CHFRAMEMOVING_H

//////////////////////////////////////////////////
//
//   ChFrameMoving.h
//
//   Math functions for FRAMES WHICH HAVE SPEED AND
//   ACCELERATION, that is a coordinate
//   system with translation and rotation etc-
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright 1996/2005 Alessandro Tasora
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChFrame.h"



namespace chrono
{



/// ChFrame: a class for coordinate systems in 3D space.
///
///  A 'frame' coordinate system has a translation and
/// a rotation respect to a 'parent' coordinate system,
/// usually the absolute (world) coordinates.
///
///  Differently from a simple ChCoordsys() object, however,
/// the ChFrame implements some optimizations because
/// each ChFrame stores also a 3x3 rotation matrix, which
/// can speed up coordinate transformations when a large
/// amount of vectors must be transfomed by the same
/// coordinate frame.
///

template <class Real = double>
class ChFrameMoving : public ChFrame<Real>
{
public:
			//
			// DATA
			//
						/// Rotation and position speed, as vector+quaternion
	ChCoordsys<Real> coord_dt;
						/// Rotation and position acceleration, as vector+quaternion
	ChCoordsys<Real> coord_dtdt;


			//
			// CONSTRUCTORS
			//

						/// Construct from pos and rot (as a quaternion)
	explicit ChFrameMoving(const ChVector<Real>& mv = VNULL, const ChQuaternion<Real>& mq = QUNIT)
		: ChFrame<Real>(mv,mq) { coord_dt.rot = coord_dtdt.rot = QNULL; };

						/// Construct from pos and rotation (as a 3x3 matrix)
	explicit ChFrameMoving(const ChVector<Real>& mv, ChMatrix33<Real>& ma)
		: ChFrame<Real>(mv,ma) { coord_dt.rot = coord_dtdt.rot = QNULL; };

						/// Construct from a coordsys
	explicit ChFrameMoving(const ChCoordsys<Real>& mc)
		: ChFrame<Real>(mc) { coord_dt.rot = coord_dtdt.rot = QNULL; };

						/// Copy constructor, build from another frame
	ChFrameMoving(const ChFrameMoving<Real>& other) :
					ChFrame<Real>(other),
					coord_dt(other.coord_dt),
					coord_dtdt(other.coord_dtdt)
					{}



			//
			// OPERATORS OVERLOADING
			//


					/// Assignment operator: copy from another frame
	ChFrameMoving<Real>& operator=(const ChFrameMoving<Real>& other)
		{
				if (&other == this) return *this;
				ChFrame<Real>::operator=(other);
				coord_dt = other.coord_dt;
				coord_dtdt = other.coord_dtdt;
				return *this;
		}

					/// Returns true for identical frames.
	bool operator==(const ChFrameMoving<Real>& other) const { return Equals(other);}

					/// Returns true for different frames.
	bool operator!=(const ChFrameMoving<Real>& other) const { return !Equals(other);}


					/// The '>>' operator transforms a coordinate system, so
					/// transformations can be represented with this syntax:
					///  new_frame = old_frame >> tr_frame;
					/// For a sequence of transformations, i.e. a chain of coordinate
					/// systems, you can also write this (like you would do with
					/// a sequence of Denavitt-Hartemberg matrix multiplications,
					/// but in the _opposite_ order...)
					///  new_frame = old_frame >> frame3to2 >> frame2to1 >> frame1to0;
					/// This operation is not commutative.
					/// Also speeds and accelerations are transformed.
	ChFrameMoving<Real> operator >> (const ChFrameMoving<Real>& Fb) const
		{
				static ChFrameMoving<Real> res;
				Fb.TrasformLocalToParent(*this, res);
				return res;
		}

					/// The '*' operator transforms a coordinate system, so
					/// transformations can be represented with this syntax:
					///  new_frame = tr_frame * old_frame;
					/// For a sequence of transformations, i.e. a chain of coordinate
					/// systems, you can also write this (just like you would do with
					/// a sequence of Denavitt-Hartemberg matrix multiplications!)
					///  new_frame = frame1to0 * frame2to1 * frame3to2 * old_frame;
					/// This operation is not commutative.
					/// Also speeds and accelerations are transformed.
	ChFrameMoving<Real> operator * (const ChFrameMoving<Real>& Fb) const
		{
				static ChFrameMoving<Real> res;
				TrasformLocalToParent(Fb,res);
				return res;
		}


					/// Performs pre-multiplication of this frame by another
					/// frame, for example: A%=T means  A'=T*A
	ChFrameMoving<Real>& operator %= (const ChFrameMoving<Real>& T)
		{
				ConcatenatePreTransformation(T);
				return *this;
		}

					/// Performs post-multiplication of this frame by another
					/// frame, for example: A*=T means  A'=A*T
	ChFrameMoving<Real>& operator *= (const ChFrameMoving<Real>& T)
		{
				ConcatenatePostTransformation(T);
				return *this;
		}


			//
			// FUNCTIONS
			//

				// GET-FUNCTIONS

					/// Return both current rotation and translation speeds as
					/// a coordsystem object, with vector and quaternion
	ChCoordsys<Real> GetCoord_dt()
						{ return coord_dt; }
					/// Return both current rotation and translation accelerations as
					/// a coordsystem object, with vector and quaternion
	ChCoordsys<Real> GetCoord_dtdt()
						{ return coord_dtdt; }

					/// Return the current speed as a 3d vector
	ChVector<Real> GetPos_dt()
						{ return coord_dt.pos; }
					/// Return the current acceleration as a 3d vector
	ChVector<Real> GetPos_dtdt()
						{ return coord_dtdt.pos; }

					/// Return the current rotation speed as a quaternion
	ChQuaternion<Real> GetRot_dt()
						{ return coord_dt.rot; }
					/// Return the current rotation acceleration as a quaternion
	ChQuaternion<Real> GetRot_dtdt()
						{ return coord_dtdt.rot; }


					/// Computes the actual angular speed (expressed in local coords)
	ChVector<Real>	GetWvel_loc() const
						{
							ChMatrixNM<Real,3,4> tempGl;
							ChFrame<Real>::SetMatrix_Gl(tempGl, this->coord.rot);
							return tempGl.Matr34_x_Quat(coord_dt.rot); // wl=[Gl]*q_dt
						}
					/// Computes the actual angular speed (expressed in parent coords)
	ChVector<Real>	GetWvel_par() const
						{
							ChMatrixNM<Real,3,4> tempGw;
							ChFrame<Real>::SetMatrix_Gw(tempGw, this->coord.rot);
							return tempGw.Matr34_x_Quat(coord_dt.rot); // ww=[Gw]*q_dt
						}

					/// Computes the actual angular acceleration (expressed in local coords)
	ChVector<Real>	GetWacc_loc() const
						{
							ChMatrixNM<Real,3,4> tempGl;
							ChFrame<Real>::SetMatrix_Gl(tempGl, this->coord.rot);
							return tempGl.Matr34_x_Quat(coord_dtdt.rot); // al=[Gl]*q_dtdt
						}
					/// Computes the actual angular acceleration (expressed in parent coords)
	ChVector<Real>	GetWacc_par() const
						{
							ChMatrixNM<Real,3,4> tempGw;
							ChFrame<Real>::SetMatrix_Gw(tempGw, this->coord.rot);
							return tempGw.Matr34_x_Quat(coord_dtdt.rot); // aw=[Gw]*q_dtdt
						}


				// SET-FUNCTIONS

					/// Set both linear speed and rotation speed as a
					/// single ChCoordsys derivative.
	virtual void SetCoord_dt(const ChCoordsys<Real>& mcoord_dt)
						{
							coord_dt = mcoord_dt;
						}

					/// Set the linear speed
	virtual void SetPos_dt(const ChVector<Real>& mvel)
						{
							coord_dt.pos = mvel;
						}

					/// Set the rotation speed as a quaternion.
					/// Note: the quaternion must already satisfy  dot(q,q_dt)=0
	virtual void SetRot_dt(const ChQuaternion<Real>& mrot_dt)
						{
							coord_dt.rot = mrot_dt;
						}

					/// Set the rotation speed from given angular speed
					/// (expressed in local csys)
	virtual void SetWvel_loc(const ChVector<Real>& wl)
						{
							coord_dt.rot.Cross(this->coord.rot, ChQuaternion<Real>(0,wl));
							coord_dt.rot*=0.5;	// q_dt = 1/2 * q * (0,wl)
						}

					/// Set the rotation speed from given angular speed
					/// (expressed in parent csys)
	virtual void SetWvel_par(const ChVector<Real>& wp)
						{
							coord_dt.rot.Cross(ChQuaternion<Real>(0,wp), this->coord.rot);
							coord_dt.rot*=0.5;	// q_dt = 1/2 * (0,wp) * q
						}


					/// Set both linear acceleration and rotation acceleration as a
					/// single ChCoordsys derivative.
	virtual void SetCoord_dtdt(const ChCoordsys<Real>& mcoord_dtdt)
						{
							coord_dtdt = mcoord_dtdt;
						}


					/// Set the linear acceleration
	virtual void SetPos_dtdt(const ChVector<Real>& macc)
						{
							coord_dtdt.pos = macc;
						}

					/// Set the rotation acceleration as a quaternion derivative.
					/// Note: the quaternion must already satisfy  dot(q,q_dt)=0
	virtual void SetRot_dtdt(const ChQuaternion<Real>& mrot_dtdt)
						{
							coord_dtdt.rot = mrot_dtdt;
						}

					/// Set the rotation acceleration from given angular acceleration
					/// (expressed in local csys)
	virtual void SetWacc_loc(const ChVector<Real>& al)
						{
								// q_dtdt = q_dt * q' * q_dt + 1/2 * q * (0,al)
							coord_dtdt.rot = (coord_dt.rot % this->coord.rot.GetConjugate() % coord_dt.rot)
											+ (this->coord.rot % ChQuaternion<Real>(0,al) *0.5);
						}

					/// Set the rotation speed from given angular speed
					/// (expressed in parent csys)
	virtual void SetWacc_par(ChVector<Real>& ap)
						{
								// q_dtdt = q_dt * q' * q_dt + 1/2 * (0,ap) * q
							coord_dtdt.rot = (coord_dt.rot % this->coord.rot.GetConjugate() % coord_dt.rot)
											+ (ChQuaternion<Real>(0,ap) % this->coord.rot * 0.5);
						}

					/// Computes the time derivative of rotation matrix, mAdt.
	void Compute_Adt(ChMatrix33<Real>& mA_dt) const
						{
							//  [A_dt]=2[dFp/dt][Fm]'=2[Fp(q_dt)][Fm(q)]'
							ChMatrixNM<Real,3,4> Fpdt;
							ChMatrixNM<Real,3,4> Fm;
							ChFrame<Real>::SetMatrix_Fp (Fpdt, coord_dt.rot);
							ChFrame<Real>::SetMatrix_Fm (Fm,   this->coord.rot);
							mA_dt.MatrMultiplyT(Fpdt, Fm);
							mA_dt.MatrScale(2.);
						}

					/// Computes the 2nd time derivative of rotation matrix, mAdtdt.
	void Compute_Adtdt(ChMatrix33<Real>& mA_dtdt)
						{
							//  [A_dtdt]=2[Fp(q_dtdt)][Fm(q)]'+2[Fp(q_dt)][Fm(q_dt)]'
							ChMatrixNM<Real,3,4> ma;
							ChMatrixNM<Real,3,4> mb;
							ChMatrix33<Real> mr;

							ChFrame<Real>::SetMatrix_Fp (ma, coord_dtdt.rot);
							ChFrame<Real>::SetMatrix_Fm (mb, this->coord.rot);
							mr.MatrMultiplyT(ma,mb);
							ChFrame<Real>::SetMatrix_Fp (ma, coord_dt.rot);
							ChFrame<Real>::SetMatrix_Fm (mb, coord_dt.rot);
							mA_dtdt.MatrMultiplyT(ma, mb);
							mA_dtdt.MatrInc(mr);
							mA_dtdt.MatrScale(2.);
						}

					/// Computes and returns an Adt matrix (-note: prefer using
					/// Compute_Adt() directly for better performance)
	ChMatrix33<Real> GetA_dt()
						{
							ChMatrix33<Real> res; Compute_Adt(res); return res;
						}

					/// Computes and returns an Adt matrix (-note: prefer using
					/// Compute_Adtdt() directly for better performance)
	ChMatrix33<Real> GetA_dtdt()
						{
							ChMatrix33<Real> res; Compute_Adtdt(res); return res;
						}

				// FUNCTIONS TO TRANSFORMATE THE FRAME ITSELF

					/// Apply a transformation (rotation and translation) represented by
					/// another ChFrameMoving T. This is equivalent to pre-multiply this frame
					/// by the other frame T:   this'= T * this;
	void ConcatenatePreTransformation(const ChFrameMoving<Real>& T)
						{
							ChFrameMoving<Real> res;
							T.TrasformLocalToParent(*this, res);
							*this = res;
						}

					/// Apply a transformation (rotation and translation) represented by
					/// another ChFrameMoving T in local coordinate. This is equivalent to
					/// post-multiply this frame by the other frame T:   this'= this * T;
	void ConcatenatePostTransformation(const ChFrameMoving<Real>& T)
						{
							ChFrameMoving<Real> res;
							this->TrasformLocalToParent(T, res);
							*this = res;
						}




				// FUNCTIONS FOR COORDINATE TRANSFORMATIONS

					/// Given the position of a point in local frame coords, and
					/// assuming it is sticky to frame, return the speed in parent coords.
	ChVector<Real> PointSpeedLocalToParent(const ChVector<Real>& localpos) const
						{
							return coord_dt.pos +
									((coord_dt.rot % ChQuaternion<Real>(0,localpos) % this->coord.rot.GetConjugate()).GetVector()*2) ;
						}

					/// Given the position of a point in local frame coords, and
					/// assuming it has a frame-relative speed localspeed,
					/// return the speed in parent coords.
	ChVector<Real> PointSpeedLocalToParent(const ChVector<Real>& localpos, const ChVector<Real>& localspeed) const
						{
							return coord_dt.pos +
									this->Amatrix.Matr_x_Vect(localspeed) +
									((coord_dt.rot % ChQuaternion<Real>(0,localpos) % this->coord.rot.GetConjugate()).GetVector()*2) ;
						}

					/// Given the position of a point in local frame coords, and
					/// assuming it is sticky to frame, return the acceleration in parent coords.
	ChVector<Real> PointAccelerationLocalToParent(const ChVector<Real>& localpos) const
						{
							return coord_dtdt.pos +
									((coord_dtdt.rot % ChQuaternion<Real>(0,localpos) % this->coord.rot.GetConjugate()).GetVector()*2) +
									((coord_dt.rot % ChQuaternion<Real>(0,localpos) % coord_dt.rot.GetConjugate()).GetVector()*2);
						}

					/// Given the position of a point in local frame coords, and
					/// assuming it has a frame-relative speed localspeed and frame-relative
					/// acceleration localacc, return the acceleration in parent coords.
	ChVector<Real> PointAccelerationLocalToParent(const ChVector<Real>& localpos, const ChVector<Real>& localspeed, const ChVector<Real>& localacc) const
						{
							return coord_dtdt.pos +
									this->Amatrix.Matr_x_Vect(localacc) +
									((coord_dtdt.rot % ChQuaternion<Real>(0,localpos) % this->coord.rot.GetConjugate()).GetVector()*2) +
									((coord_dt.rot % ChQuaternion<Real>(0,localpos) % coord_dt.rot.GetConjugate()).GetVector()*2) +
									((coord_dt.rot % ChQuaternion<Real>(0,localspeed) % this->coord.rot.GetConjugate()).GetVector()*4);
						}

					/// Given the position of a point in parent frame coords, and
					/// assuming it has an absolute speed parentspeed,
					/// return the speed in local coords.
	ChVector<Real> PointSpeedParentToLocal(const ChVector<Real>& parentpos, const ChVector<Real>& parentspeed) const
						{
							ChVector<Real> localpos = ChFrame<Real>::TrasformParentToLocal(parentpos);
							return this->Amatrix.MatrT_x_Vect( parentspeed - coord_dt.pos -
								((coord_dt.rot % ChQuaternion<Real>(0,localpos) % this->coord.rot.GetConjugate()).GetVector()*2) );
						}

					/// Given the position of a point in parent frame coords, and
					/// assuming it has an absolute speed parentspeed and absolute
					/// acceleration parentacc, return the acceleration in local coords.
	ChVector<Real> PointAccelerationParentToLocal(const ChVector<Real>& parentpos, const ChVector<Real>& parentspeed, const ChVector<Real>& parentacc) const
						{
							ChVector<Real> localpos = ChFrame<Real>::TrasformParentToLocal(parentpos);
							ChVector<Real> localspeed = PointSpeedParentToLocal(parentpos, parentspeed);
							return this->Amatrix.MatrT_x_Vect( parentacc - coord_dtdt.pos -
									(coord_dtdt.rot % ChQuaternion<Real>(0,localpos) % this->coord.rot.GetConjugate()).GetVector()*2 -
									(coord_dt.rot % ChQuaternion<Real>(0,localpos) % coord_dt.rot.GetConjugate()).GetVector()*2 -
									(coord_dt.rot % ChQuaternion<Real>(0,localspeed) % this->coord.rot.GetConjugate()).GetVector()*4  );
						}



					/// This function transforms a frame from 'this' local coordinate
					/// system to parent frame coordinate system, and also transforms the speed
					/// and acceleration of the frame.
	void TrasformLocalToParent (
								const ChFrameMoving<Real>& local,	///< frame to transform, given in local frame coordinates
								ChFrameMoving<Real>& parent			///< transformed frame, in parent coordinates, will be stored here
								)  const
						{
								// pos & rot
								ChFrame<Real>::TrasformLocalToParent(local,parent);

								// pos_dt
								parent.coord_dt.pos = PointSpeedLocalToParent(local.coord.pos, local.coord_dt.pos);

								// pos_dtdt
								parent.coord_dtdt.pos = PointAccelerationLocalToParent(local.coord.pos, local.coord_dt.pos, local.coord_dtdt.pos);

								// rot_dt
								parent.coord_dt.rot =
										coord_dt.rot % local.coord.rot +
										this->coord.rot % local.coord_dt.rot;

								// rot_dtdt
								parent.coord_dtdt.rot =
										coord_dtdt.rot % local.coord.rot +
										(coord_dt.rot % local.coord_dt.rot)*2 +
										this->coord.rot % local.coord_dtdt.rot;

						}


					/// This function transforms a frame from the parent coordinate
					/// system to 'this' local frame coordinate system.
	void TrasformParentToLocal (
								const ChFrameMoving<Real>& parent,	///< frame to transform, given in parent coordinates
								ChFrameMoving<Real>& local			///< transformed frame, in local coordinates, will be stored here
								)  const
						{
								// pos & rot
								ChFrame<Real>::TrasformParentToLocal(parent,local);

								// pos_dt
								local.coord_dt.pos = PointSpeedParentToLocal(parent.coord.pos, parent.coord_dt.pos);

								// pos_dtdt
								local.coord_dtdt.pos = PointAccelerationParentToLocal(parent.coord.pos, parent.coord_dt.pos, parent.coord_dtdt.pos);

								// rot_dt
								local.coord_dt.rot = this->coord.rot.GetConjugate() %
									(parent.coord_dt.rot - coord_dt.rot % local.coord.rot);

								// rot_dtdt
								local.coord_dtdt.rot =  this->coord.rot.GetConjugate() %
										(parent.coord_dtdt.rot - coord_dtdt.rot % local.coord.rot - (coord_dt.rot % local.coord_dt.rot)*2 );
						}


				// OTHER FUNCTIONS

					/// Returns true if coordsys is identical to other coordsys
	bool	Equals ( const ChFrameMoving<Real>& other) const { return this->coord.Equals(other.coord) && coord_dt.Equals(other.coord_dt) && coord_dtdt.Equals(other.coord_dtdt);}

					/// Returns true if coordsys is equal to other coordsys, within a tolerance 'tol'
	bool	Equals ( const ChFrameMoving<Real>& other, Real tol) const { return this->coord.Equals(other.coord,tol) && coord_dt.Equals(other.coord_dt,tol) && coord_dtdt.Equals(other.coord_dtdt,tol);}


					/// The trasformation (also for speeds, accelerations) is
					/// inverted in place.
					/// That is if w=A*v, then A.Invert();v=A*w;
	virtual void Invert()
						{
							static ChFrameMoving<Real> tmp;
							static ChFrameMoving<Real> unit;
							tmp = *this;
							tmp.TrasformParentToLocal(unit, *this);
						}

	ChFrameMoving<Real> GetInverse()
						{
							ChFrameMoving<Real> tmp(*this);
							tmp.Invert(); return tmp;
						}
			//
			// STREAMING
			//


					/// Method to allow serializing transient data into in ascii
					/// as a readable item, for example   "chrono::GetLog() << myobject;"
	virtual void StreamOUT(ChStreamOutAscii& mstream)
						{
							ChFrame<Real>::StreamOUT(mstream);
							ChVector<Real> mwl = GetWvel_loc();
							ChVector<Real> mal = GetWacc_loc();
							mstream << "\n   speed:     "<<  coord_dt.pos.x << "  " <<  coord_dt.pos.y << "  " <<  coord_dt.pos.z;
							mstream << "\n   ang.speed: "<<  mwl.x << "  " <<  mwl.y << "  " <<  mwl.z;
							mstream << "\n   accel:     "<<  coord_dtdt.pos.x << "  " <<  coord_dtdt.pos.y << "  " <<  coord_dtdt.pos.z;
							mstream << "\n   ang.accel: "<<  mal.x << "  " <<  mal.y << "  " <<  mal.z;
							mstream << "\n ";
						}

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	virtual void StreamOUT(ChStreamOutBinary& mstream)
						{
							// Serialize parent class
							ChFrame<Real>::StreamOUT(mstream);

							// Serialize other data
							mstream << coord_dt;
							mstream << coord_dtdt;
						}

					/// Operator to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	virtual void StreamIN(ChStreamInBinary& mstream)
						{
							// Deserialize parent class
							ChFrame<Real>::StreamIN(mstream);

							// Deserialize other data
							mstream >> coord_dt;
							mstream >> coord_dtdt;
						}


};








} // END_OF_NAMESPACE____




#endif  // END of ChFrame.h
