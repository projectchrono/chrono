//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File authors: Andrea Favali, Alessandro Tasora


#include "core/ChMath.h"
#include "physics/ChObject.h"
#include "ChMesh.h"
// for the TetGen parsing:
#include "ChNodeFEMxyz.h"
#include "ChElementTetra_4.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
#include <functional> 


using namespace std;


namespace chrono 
{
namespace fem
{



void ChMesh::SetupInitial()
{
	n_dofs = 0;

	for (unsigned int i=0; i< vnodes.size(); i++)
	{
			//    - count the degrees of freedom 
		n_dofs += vnodes[i]->Get_ndof();
	}

	for (unsigned int i=0; i< velements.size(); i++)
	{
			//    - precompute matrices, such as the [Kl] local stiffness of each element, if needed, etc.
		velements[i]->SetupInitial();
	}

}


void ChMesh::Relax ()
{
	for (unsigned int i=0; i< vnodes.size(); i++)
	{
			//    - "relaxes" the structure by setting all X0 = 0, and null speeds
		vnodes[i]->Relax();
	}
}


void ChMesh::AddNode ( ChSharedPtr<ChNodeFEMbase> m_node)
{
	this->vnodes.push_back(m_node);
}

void ChMesh::AddElement ( ChSharedPtr<ChElementBase> m_elem)
{
	this->velements.push_back(m_elem);
}

void ChMesh::ClearElements ()
{
	velements.clear();
}

void ChMesh::ClearNodes ()
{
	velements.clear();
	vnodes.clear();
}





// Updates all time-dependant variables, if any...
// Ex: maybe the elasticity can increase in time, etc.

void ChMesh::Update (double m_time)
{
	// Parent class update
	ChIndexedNodes::Update(m_time);
	
	for (unsigned int i=0; i< velements.size(); i++)
	{
			//    - update auxiliary stuff, ex. update element's rotation matrices if corotational..
		velements[i]->Update();
	}

}


void ChMesh::LoadFromTetGenFile(char* filename_node, char* filename_ele, ChSharedPtr<ChContinuumElastic> my_material)
{
	int totnodes = 0;
	int nodes_offset = this->GetNnodes();
	int added_nodes = 0;

	// Load .node TetGen file
	{
		bool parse_header = true;
		bool parse_nodes = false;
		
		fstream fin(filename_node);
		if (!fin.good())
			throw ChException("ERROR opening TetGen .node file: " + std::string(filename_node) + "\n");
		string line;
		while(getline(fin, line)) 
		{
			//trims white space from the beginning of the string
			line.erase(line.begin(), find_if(line.begin(), line.end(), not1(ptr_fun<int, int>(isspace)))); 

			if(line[0] == '#') continue; // skip comment
			if(line[0] == 0) continue; // skip empty lines

			int nnodes, ndims, nattrs, nboundarymark;

			if (parse_header)
			{
				stringstream(line) >> nnodes >> ndims >> nattrs >> nboundarymark;
				if (ndims != 3)
					throw ChException("ERROR in TetGen .node file. Only 3 dimensional nodes supported: \n"+ line);
				if (nattrs != 0)
					throw ChException("ERROR in TetGen .node file. Only nodes with 0 attrs supported: \n"+ line);
				if (nboundarymark != 0)
					throw ChException("ERROR in TetGen .node file. Only nodes with 0 markers supported: \n"+ line);
				parse_header = false;
				parse_nodes = true;
				totnodes = nnodes;
				continue;
			}

			int idnode = 0;
			double x = -10e30;
			double y = -10e30;
			double z = -10e30;

			if (parse_nodes)
			{
				stringstream(line) >> idnode >> x >> y >> z;
				++added_nodes;
				if (idnode < 0 || idnode > nnodes)
					throw ChException("ERROR in TetGen .node file. Node ID not in range: \n"+ line +"\n");
				if (idnode != added_nodes)
					throw ChException("ERROR in TetGen .node file. Nodes IDs must be sequential (1 2 3 ..): \n"+ line+"\n");
				if (x == -10e30 || y == -10e30 || z == -10e30 )
					throw ChException("ERROR in TetGen .node file, in parsing x,y,z coordinates of node: \n"+ line+"\n");
				
				ChSharedPtr<ChNodeFEMxyz> mnode( new ChNodeFEMxyz(ChVector<>(x,y,z)) );
				this->AddNode(mnode);
			}

		} // end while
        
    }// end .node file

	// Load .ele TetGen file
	{
		bool parse_header = true;
		bool parse_tet  = false;

		fstream fin(filename_ele);
		if (!fin.good())
			throw ChException("ERROR opening TetGen .node file: " + std::string(filename_node) + "\n");
		string line;
		while(getline(fin, line)) 
		{
			//trims white space from the beginning of the string
			line.erase(line.begin(), find_if(line.begin(), line.end(), not1(ptr_fun<int, int>(isspace)))); 

			if(line[0] == '#') continue; // skip comment
			if(line[0] == 0) continue; // skip empty lines

			int ntets, nnodespertet, nattrs;

			if (parse_header)
			{
				stringstream(line) >> ntets >> nnodespertet >> nattrs;
				if (nnodespertet != 4)
					throw ChException("ERROR in TetGen .ele file. Only 4 -nodes per tes supported: \n"+ line+"\n");
				if (nattrs != 0)
					throw ChException("ERROR in TetGen .ele file. Only tets with 0 attrs supported: \n"+ line+"\n");
				parse_header = false;
				parse_tet = true;
				continue;
			}

			int idtet = 0;
			int n1,n2,n3,n4;

			if (parse_tet)
			{
				stringstream(line) >> idtet >> n1 >> n2 >> n3 >> n4;
				if (idtet < 0 || idtet > ntets)
					throw ChException("ERROR in TetGen .node file. Tetahedron ID not in range: \n"+ line+"\n");
				if (n1 > totnodes)
					throw ChException("ERROR in TetGen .node file, ID of 1st node is out of range: \n"+ line+"\n");
				if (n2 > totnodes)
					throw ChException("ERROR in TetGen .node file, ID of 2nd node is out of range: \n"+ line+"\n");
				if (n3 > totnodes)
					throw ChException("ERROR in TetGen .node file, ID of 3rd node is out of range: \n"+ line+"\n");
				if (n4 > totnodes)
					throw ChException("ERROR in TetGen .node file, ID of 4th node is out of range: \n"+ line+"\n");
				
				ChSharedPtr<ChElementTetra_4> mel( new ChElementTetra_4 );
				mel->SetNodes(
					this->GetNode(nodes_offset + n1-1), 
					this->GetNode(nodes_offset + n3-1), 
					this->GetNode(nodes_offset + n2-1), 
					this->GetNode(nodes_offset + n4-1) );
				mel->SetMaterial(my_material);
				this->AddElement(mel);
			}

		} // end while
        
    }// end .ele file

}





void ChMesh::InjectKRMmatrices(ChLcpSystemDescriptor& mdescriptor) 
{
	for (unsigned int ie = 0; ie < this->velements.size(); ie++)
		this->velements[ie]->InjectKRMmatrices(mdescriptor);
}

void ChMesh::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor)
{
	for (unsigned int ie = 0; ie < this->velements.size(); ie++)
		this->velements[ie]->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
}

void ChMesh::VariablesFbReset()
{
	for (unsigned int ie = 0; ie < this->vnodes.size(); ie++)
		this->vnodes[ie]->VariablesFbReset();
}

void ChMesh::VariablesFbLoadForces(double factor)
{
	// applied nodal forces
	for (unsigned int in = 0; in < this->vnodes.size(); in++)
		this->vnodes[in]->VariablesFbLoadForces(factor);

	// internal forces
	for (unsigned int ie = 0; ie < this->velements.size(); ie++)
		this->velements[ie]->VariablesFbLoadInternalForces(factor);
}

void ChMesh::VariablesQbLoadSpeed() 
{
	for (unsigned int ie = 0; ie < this->vnodes.size(); ie++)
		this->vnodes[ie]->VariablesQbLoadSpeed();
}

void ChMesh::VariablesFbIncrementMq() 
{
	// nodal masses
	for (unsigned int ie = 0; ie < this->vnodes.size(); ie++)
		this->vnodes[ie]->VariablesFbIncrementMq();

	// internal masses
	for (unsigned int ie = 0; ie < this->velements.size(); ie++)
		this->velements[ie]->VariablesFbIncrementMq();
}

void ChMesh::VariablesQbSetSpeed(double step) 
{
	for (unsigned int ie = 0; ie < this->vnodes.size(); ie++)
		this->vnodes[ie]->VariablesQbSetSpeed(step);
}

void ChMesh::VariablesQbIncrementPosition(double step)
{
	for (unsigned int ie = 0; ie < this->vnodes.size(); ie++)
		this->vnodes[ie]->VariablesQbIncrementPosition(step);
}

void ChMesh::InjectVariables(ChLcpSystemDescriptor& mdescriptor)
{
	for (unsigned int ie = 0; ie < this->vnodes.size(); ie++)
		mdescriptor.InsertVariables(&this->vnodes[ie]->Variables());
}






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

