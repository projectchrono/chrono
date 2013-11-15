#ifndef CHELEMENTTETRA4_H
#define CHELEMENTTETRA4_H

//////////////////////////////////////////////////
//  
//   ChElementTetra_4.h
//
//   Class for tetahedrons 4 nodes
//
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright: 
//
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChElement3D.h"
#include "ChNodeFEMxyz.h"

namespace chrono
{
namespace fem
{

class ChApiFem ChElementTetra_4 : public ChTetrahedron
{
protected:
		std::vector<ChNodeFEMxyz*> nodes;
		ChSharedPtr<ChContinuumElastic> Material;
		ChMatrixDynamic<> MatrB;		// matrix of shape function's partial derivatives
		ChMatrixDynamic<> StiffnessMatrix;
	
public:

	ChElementTetra_4();
	virtual ~ChElementTetra_4();

	virtual int GetNcoords() {return 12;}
	virtual int GetNnodes()  {return 4;}
	virtual ChNodeFEMbase* GetNodeN(int n) {return nodes[n];}

	virtual void SetNodes(ChNodeFEMxyz* nodeA, ChNodeFEMxyz* nodeB, ChNodeFEMxyz* nodeC, ChNodeFEMxyz* nodeD) 
				{
					nodes[0]=nodeA; 
					nodes[1]=nodeB;
					nodes[2]=nodeC;
					nodes[3]=nodeD;
					std::vector<ChLcpVariables*> mvars;
					mvars.push_back(&nodes[0]->Variables());
					mvars.push_back(&nodes[1]->Variables());
					mvars.push_back(&nodes[2]->Variables());
					mvars.push_back(&nodes[3]->Variables());
					Kmatr.SetVariables(mvars);
				}
			
			//
			// FEM functions
			//

	double ComputeVolume()
				{
					ChVector<> B1,C1,D1;
					B1.Sub(nodes[1]->pos,nodes[0]->pos);
					C1.Sub(nodes[2]->pos,nodes[0]->pos);
					D1.Sub(nodes[3]->pos,nodes[0]->pos);
					ChMatrixDynamic<> M(3,3);
					M.PasteVector(B1,0,0);
					M.PasteVector(C1,0,1);
					M.PasteVector(D1,0,2);
					M.MatrTranspose();
					Volume = abs(M.Det()/6);
					return Volume;
				}

	
				/// Computes the global STIFFNESS MATRIX of the element:    
				/// K = Volume * [B]' * [D] * [B]
	virtual void ComputeStiffnessMatrix()
				{
					ChMatrixDynamic<> MatCoeff(3,3);
					MatCoeff.PasteVector(nodes[1]->pos,0,0);
					MatCoeff.PasteVector(nodes[2]->pos,0,1);
					MatCoeff.PasteVector(nodes[3]->pos,0,2); 
					MatCoeff.MatrTranspose();
					ChMatrixDynamic<> MatCoeffB = MatCoeff;
				MatCoeffB.PasteVector(ChVector<>(1,1,1),0,0);
					double b1=-MatCoeffB.Det();
					MatCoeffB.SetElement(0,1,nodes[0]->pos.y);
					MatCoeffB.SetElement(0,2,nodes[0]->pos.z);
					double b2=MatCoeffB.Det();
					MatCoeffB.SetElement(1,1,nodes[1]->pos.y);
					MatCoeffB.SetElement(1,2,nodes[1]->pos.z);
					double b3=-MatCoeffB.Det();
					MatCoeffB.SetElement(2,1,nodes[2]->pos.y);
					MatCoeffB.SetElement(2,2,nodes[2]->pos.z);
					double b4=MatCoeffB.Det();
					ChMatrixDynamic<> MatCoeffC = MatCoeff;
				MatCoeffC.PasteVector(ChVector<>(1,1,1),0,1);
					double c1=-MatCoeffC.Det();
					MatCoeffC.SetElement(0,0,nodes[0]->pos.x);
					MatCoeffC.SetElement(0,2,nodes[0]->pos.z);
					double c2=MatCoeffC.Det();
					MatCoeffC.SetElement(1,0,nodes[1]->pos.x);
					MatCoeffC.SetElement(1,2,nodes[1]->pos.z);
					double c3=-MatCoeffC.Det();
					MatCoeffC.SetElement(2,0,nodes[2]->pos.x);
					MatCoeffC.SetElement(2,2,nodes[2]->pos.z);
					double c4=MatCoeffC.Det();
					ChMatrixDynamic<> MatCoeffD = MatCoeff;
				MatCoeffD.PasteVector(ChVector<>(1,1,1),0,2);
					double d1=-MatCoeffD.Det();
					MatCoeffD.SetElement(0,0,nodes[0]->pos.x);
					MatCoeffD.SetElement(0,1,nodes[0]->pos.y);
					double d2=MatCoeffD.Det();
					MatCoeffD.SetElement(1,0,nodes[1]->pos.x);
					MatCoeffD.SetElement(1,1,nodes[1]->pos.y);
					double d3=-MatCoeffD.Det();
					MatCoeffD.SetElement(2,0,nodes[2]->pos.x);
					MatCoeffD.SetElement(2,1,nodes[2]->pos.y);
					double d4=MatCoeffD.Det();
					//setting matrix B
					MatrB.Resize(6,12);
					MatrB.SetElement(0,0,b1);	MatrB.SetElement(0,3,b2);	MatrB.SetElement(0,6,b3);	MatrB.SetElement(0,9,b4);	
					MatrB.SetElement(1,1,c1);	MatrB.SetElement(1,4,c2);	MatrB.SetElement(1,7,c3);	MatrB.SetElement(1,10,c4);
					MatrB.SetElement(2,2,d1);	MatrB.SetElement(2,5,d2);	MatrB.SetElement(2,8,d3);	MatrB.SetElement(2,11,d4);	
					MatrB.SetElement(3,0,c1);	MatrB.SetElement(3,1,b1);	MatrB.SetElement(3,3,c2);	MatrB.SetElement(3,4,b2);	MatrB.SetElement(3,6,c3);	MatrB.SetElement(3,7,b3);	MatrB.SetElement(3,9,c4);	MatrB.SetElement(3,10,b4);
					MatrB.SetElement(4,1,d1);	MatrB.SetElement(4,2,c1);	MatrB.SetElement(4,4,d2);	MatrB.SetElement(4,5,c2);	MatrB.SetElement(4,7,d3);	MatrB.SetElement(4,8,c3);	MatrB.SetElement(4,10,d4);	MatrB.SetElement(4,11,c4);
					MatrB.SetElement(5,0,d1);	MatrB.SetElement(5,2,b1);	MatrB.SetElement(5,3,d2);	MatrB.SetElement(5,5,b2);	MatrB.SetElement(5,6,d3);	MatrB.SetElement(5,8,b3);	MatrB.SetElement(5,9,d4);	MatrB.SetElement(5,11,b4);
					MatrB.MatrDivScale(6*Volume);
					ChMatrixDynamic<> temp(12,6);
					temp.MatrTMultiply(MatrB, Material->Get_StressStrainMatrix());
					StiffnessMatrix.Resize(12,12);
					StiffnessMatrix.MatrMultiply(temp,MatrB);
					StiffnessMatrix.MatrScale(Volume);
				}

				/// set up the element's parameters and matrices 
	virtual void SetupInitial() 
				{
					ComputeVolume();
					ComputeStiffnessMatrix();
				}

				/// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
				/// (For the spring matrix there is no need to corotate local matrices: we already know a closed form expression.)
	virtual void ComputeKRMmatricesGlobal	(ChMatrix<>& H, double Kfactor, double Rfactor=0, double Mfactor=0) 
				{
					assert((H.GetRows() == 12) && (H.GetColumns()==12));

					// For K stiffness matrix and R damping matrix:

					ChMatrixDynamic<> tempMatr = StiffnessMatrix;
					tempMatr.MatrScale( Kfactor );

					H.PasteMatrix(&tempMatr,0,0);

					//***TO DO*** corotate the stiffness/damping matrix

					// For M mass matrix:
					if (Mfactor)
					{
						double lumped_node_mass = (this->GetVolume() * this->Material->Get_density() ) / 4.0;
						for (int id = 0; id < 12; id++)
						{
							H(id,id)+= Mfactor * lumped_node_mass;
						}
					}
					//***TO DO*** better per-node lumping, or 12x12 consistent mass matrix.
				}

				/// Sets Hl as the local stiffness matrix K, scaled  by Kfactor. Optionally, also
				/// superimposes local damping matrix R, scaled by Rfactor, and local mass matrix M multiplied by Mfactor.
				/// This is usually called only once in the simulation. 
	virtual void ComputeKRMmatricesLocal (ChMatrix<>& Hl, double Kfactor, double Rfactor=0, double Mfactor=0)
				{
					assert((Hl.GetRows() == 12) && (Hl.GetColumns() == 12));

					// to keep things short, here local K is as global K (anyway, only global K is used in simulations)
					ComputeKRMmatricesGlobal (Hl, Kfactor, Rfactor, Mfactor);
				}

				/// Computes the internal forces (ex. the actual position of
				/// nodes is not in relaxed reference position) and set values
				/// in the Fi vector.
	virtual void ComputeInternalForces	(ChMatrixDynamic<>& Fi)
				{
					assert((Fi.GetRows() == 12) && (Fi.GetColumns()==1));

						// set up vector of nodal displacements
					ChMatrixDynamic<> displ(12,1);
					displ.PasteVector(nodes[0]->pos - nodes[0]->GetX0(), 0, 0);
					displ.PasteVector(nodes[1]->pos - nodes[1]->GetX0(), 3, 0);
					displ.PasteVector(nodes[2]->pos - nodes[2]->GetX0(), 6, 0);
					displ.PasteVector(nodes[3]->pos - nodes[3]->GetX0(), 9, 0);

						// [Internal Forces] = [K] * [displ]
					Fi.MatrMultiply(StiffnessMatrix,displ);

				}

			//
			// Custom properties functions
			//

				/// Set the material of the element
	void   SetMaterial( ChSharedPtr<ChContinuumElastic> my_material) { Material = my_material; }
	ChSharedPtr<ChContinuumElastic> GetMaterial() {return Material;}

				/// Get the partial derivatives matrix MatrB and the StiffnessMatrix
	ChMatrixDynamic<>   GetMatrB() { return MatrB;}
	ChMatrixDynamic<> GetStiffnessMatrix() {return StiffnessMatrix;}


			//
			// Functions for interfacing to the LCP solver 
			//            (***not needed, thank to bookkeeping in parent class ChElementGeneric)




};

}//___end of namespace fem___
}//___end of namespace chrono___

#endif
