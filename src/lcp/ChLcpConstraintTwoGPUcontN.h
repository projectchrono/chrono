#ifndef CHLCPCONSTRAINTTWOGPUCONTN_H
#define CHLCPCONSTRAINTTWOGPUCONTN_H

//////////////////////////////////////////////////
//
//   ChLcpConstraintTwoGPUcontN.h
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChLcpConstraintTwoGPUcontT.h"

#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{


///  This class is inherited by the ChLcpConstraintTwo(),
/// It is used to represent the normal reaction between two objects
/// ONLY when also two ChLcpConstraintTwoGPUcontT objects are
/// used to represent friction. 
/// It can be used only with the ChLcpIterativeCudaSolver or similar,
/// which use the GPU hardware.

class ChLcpConstraintTwoGPUcontN : public ChLcpConstraintTwo
{
	CH_RTTI(ChLcpConstraintTwoGPUcontN, ChLcpConstraintTwo)

			//
			// DATA
			//

protected:
				/// the friction coefficient 'f', for  sqrt(Tx^2+Ty^2)<f*Nz
	double friction;

	ChVector<> p1;  /// Contact point on 1st surface, in world reference
	ChVector<> p2;  /// Contact point on 2nd surface, in world reference
	ChVector<float> normal; /// Normal on 1st surface, in world reference

	float* contact_cache; /// cache of multipliers for warm starting (persistent manifold is handled by coll.engine)

					/// the pointer to U tangential component
	ChLcpConstraintTwoGPUcontT* constraint_U;
					/// the pointer to V tangential component
	ChLcpConstraintTwoGPUcontT* constraint_V;

public:

			//
			// CONSTRUCTORS
			//
						/// Default constructor
	ChLcpConstraintTwoGPUcontN() 
					{
						mode = CONSTRAINT_FRIC;
						friction = 0.0;
						constraint_U = constraint_V = 0;
						contact_cache = 0;
					};

						/// Construct and immediately set references to variables,
						/// also setting the U and V tangential friction constraints
	ChLcpConstraintTwoGPUcontN(ChLcpVariables* mvariables_a, 
							   ChLcpVariables* mvariables_b,
							   ChLcpConstraintTwoGPUcontT* aU = 0,
							   ChLcpConstraintTwoGPUcontT* aV = 0
								)
			//: ChLcpConstraintTwo(mvariables_a, mvariables_b)
					{
						SetVariables(mvariables_a, mvariables_b);
						mode = CONSTRAINT_FRIC;
						friction=0.0;
						constraint_U = aU;
						constraint_V = aV;
						contact_cache = 0;
					};

						/// Copy constructor
	ChLcpConstraintTwoGPUcontN(const ChLcpConstraintTwoGPUcontN& other) 
			: ChLcpConstraintTwo(other)
					{
						friction=other.friction;
						constraint_U = other.constraint_U;
						constraint_V = other.constraint_V;
						contact_cache = other.contact_cache;
					}

	virtual ~ChLcpConstraintTwoGPUcontN()
					{
					};

	virtual ChLcpConstraintTwoGPUcontN* new_Duplicate () {return new ChLcpConstraintTwoGPUcontN(*this);};

					/// Assignment operator: copy from other object
	ChLcpConstraintTwoGPUcontN& operator=(const ChLcpConstraintTwoGPUcontN& other)
					{
						if (&other == this)
							return *this;
						// copy parent class data
						ChLcpConstraintTwo::operator=(other);
						
						friction = other.friction;
						constraint_U = other.constraint_U;
						constraint_V = other.constraint_V;
						contact_cache = other.contact_cache;
						return *this;
					}



			//
			// FUNCTIONS
			//

				/// Set references to the constrained objects
	virtual void SetVariables(ChLcpVariables* mvariables_a, ChLcpVariables* mvariables_b)
					{
						valid = true;
						variables_a = mvariables_a;
						variables_b = mvariables_b;
					}


	ChVector<> GetP1() {return p1;}
	void SetP1( ChVector<>& mp) {p1 = mp;}
	
	ChVector<> GetP2() {return p2;}
	void SetP2( ChVector<>& mp) {p2 = mp;}

	ChVector<float> GetNormal() {return normal;}
	void SetNormal( ChVector<float>& mn) {normal = mn;}

	float* GetContactCache() {return contact_cache;}
	void SetContactCache(float* mcache) {contact_cache = mcache;}


				/// Get the friction coefficient
	double GetFrictionCoefficient() {return friction; }
				/// Set the friction coefficient
	void SetFrictionCoefficient(double mcoeff) {friction = mcoeff;}


				/// Get pointer to U tangential component
	ChLcpConstraintTwoGPUcontT* GetTangentialConstraintU() {return constraint_U;}			
				/// Get pointer to V tangential component
	ChLcpConstraintTwoGPUcontT* GetTangentialConstraintV() {return constraint_V;}

				/// Set pointer to U tangential component
	void SetTangentialConstraintU(ChLcpConstraintTwoGPUcontT* mconstr) {constraint_U = mconstr;}
				/// Set pointer to V tangential component
	void SetTangentialConstraintV(ChLcpConstraintTwoGPUcontT* mconstr) {constraint_V = mconstr;}


				/// The projection should be done directly on the GPU custom kernel code,
				/// so this won't be ever called.
	virtual void Project() {assert (false);};

				/// Computations with the following data are done directly on the GPU 
				/// custom kernel code, so this won't be ever called. 
	virtual ChMatrix<float>* Get_Cq_a() {assert (false); return 0;}
	virtual ChMatrix<float>* Get_Cq_b() {assert (false); return 0;}
	virtual ChMatrix<float>* Get_Eq_a() {assert (false); return 0;}
	virtual ChMatrix<float>* Get_Eq_b() {assert (false); return 0;}
	virtual double Compute_Cq_q() {assert (false); return 0;}
	virtual void Increment_q(const double deltal) {assert (false);};
	virtual void Build_Cq (ChSparseMatrix& storage, int insrow) {assert (false);};
	virtual void Build_CqT(ChSparseMatrix& storage, int inscol) {assert (false);};

			//
			// STREAMING
			//

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	virtual void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	virtual void StreamOUT(ChStreamOutBinary& mstream);
};




} // END_OF_NAMESPACE____



#include "core/ChMemorynomgr.h" // back to default new/delete/malloc/calloc etc. Avoid conflicts with system libs.


#endif  
