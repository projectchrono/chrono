#ifndef CHFEM_H
#define CHFEM_H

//////////////////////////////////////////////////
//
//   ChFem.h
//
//   Finite elements
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>

#include "core/ChMath.h"
#include "physics/ChIndexedNodes.h"
#include "lcp/ChLcpKstiffnessGeneric.h"
#include "lcp/ChLcpVariablesNode.h"


namespace chrono
{
namespace fem
{



/// Base class for a generic finite element node
/// that can be stored in ChMesh containers.
/// Children classes must implement specialized versions.

class ChApi ChNodeFEMbase  :  public chrono::ChNodeXYZ
{
public:
				/// Access the 'LCP variables' of the node. To be implemented in children classes.
	//virtual ChLcpVariables& GetVariables() =0;

				/// Set the rest position as the actual position.
	virtual void Relax () =0;


};



/// Class for a generic finite element node 
/// in 3D space, with x,y,z displacement. This is the typical
/// node that can be used for tetahedrons, etc.

class ChApi ChNodeFEMxyz : public ChNodeFEMbase

{
private:
	ChLcpVariablesNode	variables; /// 3D node variables, with x,y,z

	ChVector<> X0;		///< reference position
	ChVector<> Force;	///< applied force
	
	double mass;

public:

	ChNodeFEMxyz(ChVector<> initial_pos = VNULL)
					{
						X0 = initial_pos;
						pos = initial_pos;
						Force = VNULL;
						mass = 1.0;
					}

	~ChNodeFEMxyz() {};


	virtual ChLcpVariables& Variables()
					{
						return this->variables; 
					} 

	virtual void Relax () 
					{
						X0 = this->pos; this->pos_dt=VNULL; this->pos_dtdt=VNULL; 
					}

			//
			// Functions for interfacing to the LCP solver
			//

	virtual void VariablesFbLoadForces(double factor=1.) 
					{ 
						this->variables.Get_fb().PasteSumVector( this->Force * factor ,0,0);
					};

	virtual void VariablesQbLoadSpeed() 
					{ 
						this->variables.Get_qb().PasteVector(this->pos_dt,0,0); 
					};

	virtual void VariablesQbSetSpeed(double step=0.) 
					{
						ChVector<> old_dt = this->pos_dt;
						this->SetPos_dt( this->variables.Get_qb().ClipVector(0,0) );
						if (step)
						{
							this->SetPos_dtdt( (this->pos_dt - old_dt)  / step);
						}
					};

	virtual void VariablesQbIncrementPosition(double step) 
					{
						ChVector<> newspeed = variables.Get_qb().ClipVector(0,0);

						// ADVANCE POSITION: pos' = pos + dt * vel
						this->SetPos( this->GetPos() + newspeed * step);
					};

			//
			// Custom properties functions
			//
				/// Set mass of the node.
	virtual double GetMass() const {return this->variables.GetNodeMass();}
				/// Set mass of the node.
	virtual void SetMass(double mm) {this->variables.SetNodeMass(mm);}


				/// Set the initial (reference) position
	virtual void SetX0(ChVector<> mx) { X0 = mx;}
				/// Get the initial (reference) position
	virtual ChVector<> GetX0 () {return X0;}

				/// Set the 3d applied force, in absolute reference
	virtual void SetForce(ChVector<> mf) { Force = mf;}
				/// Get the 3d applied force, in absolute reference
	virtual ChVector<> GetForce () {return Force;}

};










/// Base class for all finite elements, that can be
/// used in the ChMesh physics item.
///  ***NEW, EXPERIMENTAL ***

class ChApi ChElementBase
{
protected:

public:

	ChElementBase() {};
	virtual ~ChElementBase() {};

	virtual int GetNcoords() =0;
	virtual int GetNnodes() =0;
	virtual ChNodeFEMbase* GetNodeN(int n) =0;
	

			//
			// FEM functions
			//

				/// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes global damping matrix R, scaled by Rfactor. Corotational
				/// elements can take the local Kl & Rl matrices and rotate them.
				/// ChLDREN CLASSES MUST IMPLEMENT THIS!!!
	virtual void ComputeKRmatricesGlobal (ChMatrix<>& H, double Kfactor, double Rfactor=0)  = 0;

				/// Sets Hl as the local stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes local damping matrix R, scaled by Rfactor.
				/// This is usually called only once in the simulation. 
				/// ChLDREN CLASSES MUST IMPLEMENT THIS!!!
	virtual void ComputeKRmatricesLocal (ChMatrix<>& Hl, double Kfactor, double Rfactor=0) = 0;

				/// Computes the internal forces (ex. the actual position of
				/// nodes is not in relaxed reference position) and set values
				/// in the Fi vector, whith n.rows = n.of dof of element.
				/// ChLDREN CLASSES MUST IMPLEMENT THIS!!!
	virtual void ComputeInternalForces	(ChMatrixDynamic<>& Fi) = 0;

				/// Setup (optional). Precompute matrices that do not change during the 
				/// simulation, such as the local stiffness of each element, if needed, etc.
	virtual void Setup() {};


			//
			// Functions for interfacing to the LCP solver
			// 

				/// Tell to a system descriptor that there are item(s) of type
				/// ChLcpKstiffness in this object (for further passing it to a LCP solver)
				/// Basically does nothing, but inherited classes must specialize this.
	virtual void InjectKmatrices(ChLcpSystemDescriptor& mdescriptor) =0;

				/// Adds the current stiffness K and damping R matrices in encapsulated
				/// ChLcpKstiffness item(s), if any. The K and R matrices are load with scaling 
				/// values Kfactor and Rfactor. 
	virtual void KmatricesLoad(double Kfactor, double Rfactor) =0;

				/// Adds the internal forces, expressed as nodal forces, into the
				/// encapsulated ChLcpVariables, in the 'fb' part: qf+=forces*factor
	virtual void VariablesFbLoadInternalForces(double factor=1.) {};


};


/// Class for all elements whose stiffness matrix can be seen
/// as a NxN block-matrix to be splitted between N nodes.
/// Helps reducing the complexity of inherited FEM elements because
/// it implements some bookkeeping for the interface with LCP solver.

class ChApi ChElementGeneric : public ChElementBase
{
protected:
	ChLcpKstiffnessGeneric Kmatr;

public:

	ChElementGeneric() {};
	virtual ~ChElementGeneric() {};

				/// Access the proxy to stiffness, for sparse LCP solver
	ChLcpKstiffnessGeneric& Kstiffness() {return Kmatr;}


			//
			// Functions for interfacing to the LCP solver
			//

				/// Tell to a system descriptor that there are item(s) of type
				/// ChLcpKstiffness in this object (for further passing it to a LCP solver)
				/// Basically does nothing, but inherited classes must specialize this.
	virtual void InjectKmatrices(ChLcpSystemDescriptor& mdescriptor)
				{
					mdescriptor.InsertKstiffness(&Kmatr);
				}

				/// Adds the current stiffness K and damping R matrices in encapsulated
				/// ChLcpKstiffness item(s), if any. The K and R matrices are load with scaling 
				/// values Kfactor and Rfactor. 
	virtual void KmatricesLoad(double Kfactor, double Rfactor)
				{
					this->ComputeKRmatricesGlobal(*this->Kmatr.Get_K(), Kfactor, Rfactor);
				}

				/// Adds the internal forces, expressed as nodal forces, into the
				/// encapsulated ChLcpVariables, in the 'fb' part: qf+=forces*factor
	virtual void VariablesFbLoadInternalForces(double factor=1.) 
				{
					// (This is a default (unoptimal) book keeping so that in children classes you can avoid 
					// implementing this VariablesFbLoadInternalForces function, unless you need faster code)
					ChMatrixDynamic<> mFi(this->GetNcoords(), 1);
					this->ComputeInternalForces(mFi);
					int stride = 0;
					for (int in=0; in < this->GetNnodes(); in++)
					{
						int nodedofs = GetNodeN(in)->Get_ndof();
						GetNodeN(in)->Variables().Get_fb().PasteSumClippedMatrix(&mFi, stride,0, nodedofs,1, 0,0);
						stride += nodedofs;
					}
				};

};


/// Simple finite element with two nodes and a spring between
/// the two nodes.
///  ***NEW, EXPERIMENTAL ***

class ChApi ChElementSpring : public ChElementGeneric
{
protected:
	std::vector<ChNodeFEMxyz*> nodes;
	double spring_k;
	double damper_r;
public:

	ChElementSpring() { spring_k = 1.0; damper_r = 0.01; nodes.resize(2);};
	virtual ~ChElementSpring() {};

	virtual int GetNcoords() {return 6;}
	virtual int GetNnodes()  {return 2;}
	virtual ChNodeFEMbase* GetNodeN(int n) {return nodes[n];}
	

	virtual void SetNodes(ChNodeFEMxyz* nodeA, ChNodeFEMxyz* nodeB) 
				{
					nodes[0]=nodeA; 
					nodes[1]=nodeB;
					std::vector<ChLcpVariables*> mvars;
					mvars.push_back(&nodes[0]->Variables());
					mvars.push_back(&nodes[1]->Variables());
					Kmatr.SetVariables(mvars);
				}

			//
			// FEM functions
			//

				/// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes global damping matrix R, scaled by Rfactor.
				/// (For the spring matrix there is no need to corotate local matrices: we already know a closed form expression.)
	virtual void ComputeKRmatricesGlobal	(ChMatrix<>& H, double Kfactor, double Rfactor=0) 
				{
					assert((H.GetRows() == 6) && (H.GetColumns()==6));

					// compute stiffness matrix (this is already the explicit
					// formulation of the corotational stiffness matrix in 3D)
					
					ChVector<> dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
					ChMatrixDynamic<> dircolumn; 
					dircolumn.PasteVector(dir, 0,0);

					ChMatrix33<> submatr;
					submatr.MatrTMultiply(dircolumn, dircolumn);

						// note that stiffness and damping matrices are the same, so join stuff here
					double commonfactor = this->spring_k * Kfactor + 
										  this->damper_r * Rfactor ;
					submatr.MatrScale(commonfactor);
					H.PasteMatrix(&submatr,0,0);
					H.PasteMatrix(&submatr,3,3);
					submatr.MatrNeg();
					H.PasteMatrix(&submatr,0,3);
					H.PasteMatrix(&submatr,3,0);
				}

				/// Sets Hl as the local stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes local damping matrix R, scaled by Rfactor.
				/// This is usually called only once in the simulation. 
	virtual void ComputeKRmatricesLocal (ChMatrix<>& Hl, double Kfactor, double Rfactor=0)
				{
					assert((Hl.GetRows() == 6) && (Hl.GetColumns() == 6));

					// to keep things short, here local K is as global K (anyway, only global K is used in simulations)
					ComputeKRmatricesLocal (Hl, Kfactor, Rfactor);
				}

				/// Computes the internal forces (ex. the actual position of
				/// nodes is not in relaxed reference position) and set values
				/// in the Fi vector.
	virtual void ComputeInternalForces	(ChMatrixDynamic<>& Fi)
				{
					assert((Fi.GetRows() == 6) && (Fi.GetColumns()==1));

					ChVector<> dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
					double L_ref = (nodes[1]->GetX0()  - nodes[0]->GetX0() ).Length();
					double L     = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();
					double internal_Kforce_local = this->spring_k * (L - L_ref); 
					double internal_Rforce_local = this->spring_k * (L - L_ref);
					double internal_force_local = internal_Kforce_local + internal_Rforce_local;
					ChMatrixDynamic<> displacements(6,1);
					ChVector<> int_forceA =  dir * internal_force_local;
					ChVector<> int_forceB = -dir * internal_force_local;
					Fi.PasteVector(int_forceA, 0,0);
					Fi.PasteVector(int_forceB, 3,0);
				}

				/// Setup. Precompute matrices that do not change during the 
				/// simulation, such as the local tangent stiffness Kl of each element, if needed, etc.
				/// (**Not needed for the spring element because global K is computed on-the-fly in ComputeAddKRmatricesGlobal() )
	virtual void Setup() {}


			//
			// Custom properties functions
			//

				/// Set the stiffness of the spring that connects the two nodes (N/m)
	void   SetSpringK(double ms) { spring_k = ms;}
	double GetSpringK() {return spring_k;}

				/// Set the damping of the damper that connects the two nodes (Ns/M)
	void   SetDamperR(double md) { damper_r = md;}
	double GetDamperR() {return damper_r;}


			//
			// Functions for interfacing to the LCP solver 
			//            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

};





////////////////////////////////////////////
////////////////////////////////////////////   OBSOLETE STUFF!!!!   OBSOLETE STUFF!!!!   OBSOLETE STUFF!!!!   OBSOLETE STUFF!!!!   OBSOLETE STUFF!!!! 


/// Class for a generic finite element node 
/// in 3D space  ***OBSOLETE***

class ChApi ChFnode : public ChObj
{
public:
	Vector X;		///< actual position
	Vector X_dt;	///< actual speed
	Vector X0;		///< original position
	double mass;	///< mass

	int Get_dof () {return (3);} // xyz = 3 dof
	void Relax () { X0 = X; X_dt.x=0; X_dt.y=0; X_dt.z=0; }
};



//////////////////////////////////////
//////////////////////////////////////

#define CHCLASS_ChFELEM 20

//////////////////////////////////////
//
// CLASS FOR FINITE ELEMENT

#define FELEM_BRICK_LINEAR		0
#define FELEM_BRICK_QUADRIC		1
#define FELEM_SHELL_LINEAR		2
#define FELEM_SHELL_QUADRIC		3
#define FELEM_ROD				4
#define FELEM_BAR				5
#define FELEM_TETRA_LINEAR		6
#define FELEM_TETRA_QUADRIC		7


/// Base class for a generic finete element, with stiffness
/// matrix, local coordinate system, mass, etc. ***OBSOLETE***

class ChApi ChFelem : public ChObj
{
protected:

	ChMatrixDynamic<>* Kl;		// the stiffness matrix in local coords
	ChMatrix33<>* A;		// the actual coordinate system
	ChMatrix33<>* A0;		// the original coordinate system
	double volume;	// the volume
	double mass;	// the mass
	ChFnode** np_list;// the array of pointers to nodes

public:

	ChFelem();
	~ChFelem();


	void SetNode (int index, ChFnode* node);
	ChFnode* GetNode (int index);

	virtual int Get_type () {return 0;}
	virtual int Get_dof () {return 0;}
	virtual int Get_nodes() {return 0;}

	double Get_volume () {return volume;}
	double Get_mass () {return mass;}

	Vector Get_NodeLocX0 (int index);			///< position of node in local coordsys at beginning (last relax)
	Vector Get_NodeLocX (int index);			///< position of node in local coordsys, in this instant.
	Vector Get_NodeLocDisplacement (int index); ///< local displacement {Xl}-{X0l} from beginning, in local csys.


	ChMatrix33<>* Get_A()  {return A;}
	ChMatrix33<>* Get_A0() {return A0;}
	ChMatrixDynamic<>* Get_Kl() {return Kl;}

	virtual ChMatrix<>* Get_E()  =0;
	virtual ChMatrix<>* Get_C()  =0;
	virtual ChMatrix<>* Get_N()  =0;
	virtual ChMatrix<>* Get_Be() =0;
	virtual ChMatrix<>* Get_Dl() =0;
	virtual ChMatrix<>* Get_Dl_dt() =0;
	virtual ChMatrix<>* Get_Fr() =0;



			/// Fills the static vector of local displacements
	void Compute_Dl	();

			/// Fills the static vector of local node speeds
	void Compute_Dl_dt	();


			/// Compute the constitutive matrix
			/// - the matrix will be considered always in _local_ coordinate
	virtual void Compute_Ematr (double E, double v, double G, double l, double u) =0;


			/// Compute the shape function N=N(i,j,k) for given coordinate values

	virtual void Compute_N	(double i, double j, double k) =0;


			/// Compute the shape derivative, i.e. strains "e" are:  {e}=[Be]{d}
			/// where {d} is the vector of displacement, knot by knot

	virtual void Compute_Be	(double i, double j, double k) =0;

			/// Compute  BEBj, the argument of the integral which provides the Kl matrix.
			/// This function is called by the Gauss integrator.

	virtual ChMatrix<>* Compute_BEBj (double i, double j, double k) =0;

			/// Compute stiffness matrix [K]l for given position of nodes.
			/// - Be sure to have set the E matrix
			/// - The matrix is in _local_ coordinates
			/// - Check the element topology: it must have all nodes properly set

	virtual void Compute_Kl() =0;


			/// Compute the strain and stress at given position (at given local parametric coordinates)
			/// - The strain/stresses are in _local_ coordinates, you may want to transform them
			///   into world coords later (see the related functions)
			/// - If pointer to loc_stress is NULL, it won't be computed.
			/// - Use it _after_ you have calculated the displacements of the knots, having
			///   performed some kind of analysis on the system (static or dynamic), with proper Kl.
			/// - the stress matrix is a column-vector (6 rows) with Ox, Oy, Oz, Txy, Tyz, Txz.
			/// - the strain matrix is a column-vector (6 rows) with Ex, Ey, Ez, Yxy, Yyz, Yxz.

	virtual void Compute_LocStrainStress (Vector* par, ChMatrix<>* loc_strain, ChMatrix<>* loc_stress); // -> see cpp


			/// Transformations from local to absolute coords, of 3d stress and 3d strain
	void StressToAbsCoords (ChMatrix<>* loc_stress, ChMatrix<>* abs_stress);
	void StrainToAbsCoords (ChMatrix<>* loc_strain, ChMatrix<>* abs_strain);


			/// Compute the internal reactions "Fr" which should be applied
			/// to the nodes to keep the deformed element in the current shape.
			/// You can imagine the element as a "complex spring" which is applying
	        /// forces  f=-Fr at the nodes.
			/// - The displacement vector must be already computed
			/// - the stiffness matrix Kl must be already computed.
			/// - the forces are in absolute coordinates.

	virtual void Compute_Fr ();


	//virtual void Compute_Kw(Matrix* m_Kw) {};	// project local stiffness into world-coordinates,

	 		/// updates [A] depending on knot positions
	virtual void Update_A() =0;

			/// setup [A0]=[A] at beginning, it "relaxes" the coordsys.
	void Setup_A0();

			/// Relax both nodes and coordsys; {X0}={X}   [A0]=[A]
	void Relax();

};


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






