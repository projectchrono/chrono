#ifndef CHLCPVARIABLESBODYSHAREDMASS_H
#define CHLCPVARIABLESBODYSHAREDMASS_H

//////////////////////////////////////////////////
//
//   ChLcpVariablesBodySharedMass.h
//
//    Specialized class for representing a mass matrix
//   and associate variables (6 element vector, ex.speed)
//   for a 3D rigid body, in a LCP problem of the type:
//
//    | M -Cq'|*|q|- | f|= |0| ,  c>0, l>0, l*c=0;
//    | Cq  0 | |l|  |-b|  |c|
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChLcpVariablesBody.h"


namespace chrono
{


///  Used by ChLcpVariablesBodySharedMass objects to 
/// reference a single mass propery.

class ChApi ChSharedMassBody
{
public:

	ChMatrix33<double> inertia;	// 3x3 inertia matrix
	double mass;		// mass value

	ChMatrix33<double> inv_inertia;
	double inv_mass;

public:
	
	ChSharedMassBody()
				{
					inertia.Set33Identity();
					inv_inertia.Set33Identity();
					mass = 1.;
					inv_mass = 1.;
				}

				/// Set the inertia matrix
	void    SetBodyInertia(const ChMatrix33<>* minertia)
						{
							inertia.CopyFromMatrix(*minertia);
							inertia.FastInvert(&inv_inertia);
						}

				/// Set the mass associated with translation of body
	void    SetBodyMass(const double mmass)
						{
							mass = mmass;
							inv_mass = 1.0 / mass;
						}

				/// Access the 3x3 inertia matrix
	ChMatrix33<>& GetBodyInertia() {return inertia;}

				/// Access the 3x3 inertia matrix inverted
	ChMatrix33<>& GetBodyInvInertia() {return inv_inertia;}

				/// Get the mass associated with translation of body
	double	GetBodyMass()	 {return mass;}

};


///    Specialized class for representing a 6-DOF item for a
///   LCP system, that is a 3D rigid body, with mass matrix and
///   associate variables (a 6 element vector, ex.speed)
///    Differently from the 'naive' implementation ChLcpVariablesGeneric,
///   here a full 6x6 mass matrix is not built, since only the 3x3
///   inertia matrix and the mass value are enough.
///    This is very similar to ChLcpVariablesBodyOwnMass, but the 
///   mass and inertia values are shared, that can be useful for
///   problems with thousands of equally-shaped objects.

class ChApi ChLcpVariablesBodySharedMass :  public ChLcpVariablesBody
{
	CH_RTTI(ChLcpVariablesBodySharedMass, ChLcpVariablesBody)

private:
			//
			// DATA
			//
				/// the data (qb, variables and fb, forces, already defined in base class)

			ChSharedMassBody* sharedmass;

public:

			//
			// CONSTRUCTORS
			//

	ChLcpVariablesBodySharedMass() 
				{
					sharedmass = 0;
				};

	virtual ~ChLcpVariablesBodySharedMass()
				{
				};


				/// Assignment operator: copy from other object
	ChLcpVariablesBodySharedMass& operator=(const ChLcpVariablesBodySharedMass& other);


			//
			// FUNCTIONS
			//


				/// Get the pointer to shared mass
	ChSharedMassBody* GetSharedMass() {return sharedmass;};

				/// Set pointer to shared mass
	void SetSharedMass(ChSharedMassBody* ms) {sharedmass = ms;};



				// IMPLEMENT PARENT CLASS METHODS


				/// Get the mass associated with translation of body
	virtual double	GetBodyMass()	 {return sharedmass->GetBodyMass();}

				/// Access the 3x3 inertia matrix
	virtual ChMatrix33<>& GetBodyInertia() {return sharedmass->GetBodyInertia();}

					/// Access the 3x3 inertia matrix inverted
	virtual ChMatrix33<>& GetBodyInvInertia() {return sharedmass->GetBodyInvInertia();};


				/// Computes the product of the inverse mass matrix by a
				/// vector, and set in result: result = [invMb]*vect
	virtual void Compute_invMb_v(ChMatrix<float>& result, const ChMatrix<float>& vect)
					{
						assert (vect.GetRows() == Get_ndof());
						assert (result.GetRows() == Get_ndof());
						// optimized unrolled operations
						result(0)= (float)(sharedmass->inv_mass) * vect(0);
						result(1)= (float)(sharedmass->inv_mass) * vect(1);
						result(2)= (float)(sharedmass->inv_mass) * vect(2);
						result(3)= (float)(sharedmass->inv_inertia(0,0)*vect(3) + sharedmass->inv_inertia(0,1)*vect(4) + sharedmass->inv_inertia(0,2)*vect(5));
						result(4)= (float)(sharedmass->inv_inertia(1,0)*vect(3) + sharedmass->inv_inertia(1,1)*vect(4) + sharedmass->inv_inertia(1,2)*vect(5));
						result(5)= (float)(sharedmass->inv_inertia(2,0)*vect(3) + sharedmass->inv_inertia(2,1)*vect(4) + sharedmass->inv_inertia(2,2)*vect(5));
					};
	virtual void Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect)
					{
						assert (vect.GetRows() == Get_ndof());
						assert (result.GetRows() == Get_ndof());
						// optimized unrolled operations
						result(0)= sharedmass->inv_mass * vect(0);
						result(1)= sharedmass->inv_mass * vect(1);
						result(2)= sharedmass->inv_mass * vect(2);
						result(3)= sharedmass->inv_inertia(0,0)*vect(3) + sharedmass->inv_inertia(0,1)*vect(4) + sharedmass->inv_inertia(0,2)*vect(5);
						result(4)= sharedmass->inv_inertia(1,0)*vect(3) + sharedmass->inv_inertia(1,1)*vect(4) + sharedmass->inv_inertia(1,2)*vect(5);
						result(5)= sharedmass->inv_inertia(2,0)*vect(3) + sharedmass->inv_inertia(2,1)*vect(4) + sharedmass->inv_inertia(2,2)*vect(5);
					};

				/// Computes the product of the inverse mass matrix by a
				/// vector, and increment result: result += [invMb]*vect
	virtual void Compute_inc_invMb_v(ChMatrix<float>& result, const ChMatrix<float>& vect)
					{
						assert (vect.GetRows() == Get_ndof());
						assert (result.GetRows() == Get_ndof());
						// optimized unrolled operations
						result(0)+= (float)(sharedmass->inv_mass) * vect(0);
						result(1)+= (float)(sharedmass->inv_mass) * vect(1);
						result(2)+= (float)(sharedmass->inv_mass) * vect(2);
						result(3)+= (float)(sharedmass->inv_inertia(0,0)*vect(3) + sharedmass->inv_inertia(0,1)*vect(4) + sharedmass->inv_inertia(0,2)*vect(5));
						result(4)+= (float)(sharedmass->inv_inertia(1,0)*vect(3) + sharedmass->inv_inertia(1,1)*vect(4) + sharedmass->inv_inertia(1,2)*vect(5));
						result(5)+= (float)(sharedmass->inv_inertia(2,0)*vect(3) + sharedmass->inv_inertia(2,1)*vect(4) + sharedmass->inv_inertia(2,2)*vect(5));
					};
	virtual void Compute_inc_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect)
					{
						assert (vect.GetRows() == Get_ndof());
						assert (result.GetRows() == Get_ndof());
						// optimized unrolled operations
						result(0)+= sharedmass->inv_mass * vect(0);
						result(1)+= sharedmass->inv_mass * vect(1);
						result(2)+= sharedmass->inv_mass * vect(2);
						result(3)+= sharedmass->inv_inertia(0,0)*vect(3) + sharedmass->inv_inertia(0,1)*vect(4) + sharedmass->inv_inertia(0,2)*vect(5);
						result(4)+= sharedmass->inv_inertia(1,0)*vect(3) + sharedmass->inv_inertia(1,1)*vect(4) + sharedmass->inv_inertia(1,2)*vect(5);
						result(5)+= sharedmass->inv_inertia(2,0)*vect(3) + sharedmass->inv_inertia(2,1)*vect(4) + sharedmass->inv_inertia(2,2)*vect(5);
					};


				/// Computes the product of the mass matrix by a
				/// vector, and set in result: result = [Mb]*vect
	virtual void Compute_Mb_v(ChMatrix<float>& result, const ChMatrix<float>& vect)
					{
						assert (result.GetRows() == Get_ndof());
						assert (vect.GetRows()   == Get_ndof());
						// optimized unrolled operations
						result(0)= (float)(sharedmass->mass) * vect(0);
						result(1)= (float)(sharedmass->mass) * vect(1);
						result(2)= (float)(sharedmass->mass) * vect(2);
						result(3)= (float)(sharedmass->inertia(0,0)*vect(3) + sharedmass->inertia(0,1)*vect(4) + sharedmass->inertia(0,2)*vect(5));
						result(4)= (float)(sharedmass->inertia(1,0)*vect(3) + sharedmass->inertia(1,1)*vect(4) + sharedmass->inertia(1,2)*vect(5));
						result(5)= (float)(sharedmass->inertia(2,0)*vect(3) + sharedmass->inertia(2,1)*vect(4) + sharedmass->inertia(2,2)*vect(5));
					};
	virtual void Compute_Mb_v(ChMatrix<double>& result, const ChMatrix<double>& vect)
					{
						assert (result.GetRows() == Get_ndof());
						assert (vect.GetRows()   == Get_ndof());
						// optimized unrolled operations
						result(0)= sharedmass->mass * vect(0);
						result(1)= sharedmass->mass * vect(1);
						result(2)= sharedmass->mass * vect(2);
						result(3)= (sharedmass->inertia(0,0)*vect(3) + sharedmass->inertia(0,1)*vect(4) + sharedmass->inertia(0,2)*vect(5));
						result(4)= (sharedmass->inertia(1,0)*vect(3) + sharedmass->inertia(1,1)*vect(4) + sharedmass->inertia(1,2)*vect(5));
						result(5)= (sharedmass->inertia(2,0)*vect(3) + sharedmass->inertia(2,1)*vect(4) + sharedmass->inertia(2,2)*vect(5));
					};

				/// Computes the product of the corresponding block in the 
				/// system matrix (ie. the mass matrix) by 'vect', and add to 'result'. 
				/// NOTE: the 'vect' and 'result' vectors must already have
				/// the size of the total variables&constraints in the system; the procedure
				/// will use the ChVariable offsets (that must be already updated) to know the 
				/// indexes in result and vect.
	virtual void MultiplyAndAdd(ChMatrix<double>& result, const ChMatrix<double>& vect) const
					{
						assert(result.GetColumns()==1 && vect.GetColumns()==1);
						// optimized unrolled operations
						double q0 = vect(this->offset+0);
						double q1 = vect(this->offset+1);
						double q2 = vect(this->offset+2);
						double q3 = vect(this->offset+3);
						double q4 = vect(this->offset+4);
						double q5 = vect(this->offset+5);
						result(this->offset+0)+= sharedmass->mass * q0;
						result(this->offset+1)+= sharedmass->mass * q1;
						result(this->offset+2)+= sharedmass->mass * q2;
						result(this->offset+3)+= (sharedmass->inertia(0,0)*q3 + sharedmass->inertia(0,1)*q4 + sharedmass->inertia(0,2)*q5);
						result(this->offset+4)+= (sharedmass->inertia(1,0)*q3 + sharedmass->inertia(1,1)*q4 + sharedmass->inertia(1,2)*q5);
						result(this->offset+5)+= (sharedmass->inertia(2,0)*q3 + sharedmass->inertia(2,1)*q4 + sharedmass->inertia(2,2)*q5);
					}

				/// Add the diagonal of the mass matrix (as a column vector) to 'result'.
				/// NOTE: the 'result' vector must already have the size of system unknowns, ie
				/// the size of the total variables&constraints in the system; the procedure
				/// will use the ChVariable offset (that must be already updated) as index.
	virtual void DiagonalAdd(ChMatrix<double>& result) const 
					{
						assert(result.GetColumns()==1);
						result(this->offset+0)+= sharedmass->mass;
						result(this->offset+1)+= sharedmass->mass;
						result(this->offset+2)+= sharedmass->mass;
						result(this->offset+3)+= sharedmass->inertia(0,0);
						result(this->offset+4)+= sharedmass->inertia(1,1);
						result(this->offset+5)+= sharedmass->inertia(2,2);
					}

				/// Build the mass matrix (for these variables) storing
				/// it in 'storage' sparse matrix, at given column/row offset.
				/// Note, most iterative solvers don't need to know mass matrix explicitly.
				/// Optimised: doesn't fill unneeded elements except mass and 3x3 inertia.
	virtual void Build_M(ChSparseMatrix& storage, int insrow, int inscol)
					{
						storage.SetElement(insrow+0, inscol+0, sharedmass->mass);
						storage.SetElement(insrow+1, inscol+1, sharedmass->mass);
						storage.SetElement(insrow+2, inscol+2, sharedmass->mass);
						storage.PasteMatrix(&(sharedmass->inertia), insrow+3, inscol+3);
					};


};




} // END_OF_NAMESPACE____




#endif  // END of ChLcpVariablesBody.h
