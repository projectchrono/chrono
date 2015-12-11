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
#include "physics/ChLoad.h"
#include "physics/ChSystem.h"
#include "ChMesh.h"
// for the TetGen parsing:
#include "ChNodeFEAxyz.h"
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
namespace fea
{



void ChMesh::SetupInitial()
{
	n_dofs = 0;
	n_dofs_w = 0;

	for (unsigned int i=0; i< vnodes.size(); i++)
	{
		if (!vnodes[i]->GetFixed())
		{
				//    - count the degrees of freedom 
			n_dofs += vnodes[i]->Get_ndof_x();
			n_dofs_w += vnodes[i]->Get_ndof_w();
		}
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


void ChMesh::SetNoSpeedNoAcceleration()
{ 
	for (unsigned int i=0; i< vnodes.size(); i++)
	{
			//    -  set null speeds, null accelerations
		vnodes[i]->SetNoSpeedNoAcceleration();
	}
}


void ChMesh::AddNode ( ChSharedPtr<ChNodeFEAbase> m_node)
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
    vcontactsurfaces.clear();
}

void ChMesh::ClearNodes ()
{
	velements.clear();
	vnodes.clear();
    vcontactsurfaces.clear();
}

void ChMesh::AddContactSurface ( ChSharedPtr<ChContactSurface> m_surf)
{
    m_surf->SetMesh(this);
	this->vcontactsurfaces.push_back(m_surf);
}

void ChMesh::ClearContactSurfaces()
{
	vcontactsurfaces.clear();
}


void ChMesh::AddMeshSurface ( ChSharedPtr<ChMeshSurface> m_surf)
{
    m_surf->SetMesh(this);
	this->vmeshsurfaces.push_back(m_surf);
}


/// This recomputes the number of DOFs, constraints,
/// as well as state offsets of contained items 
void ChMesh::Setup()
{
	n_dofs = 0;
	n_dofs_w = 0;

	for (unsigned int i=0; i< vnodes.size(); i++)
	{
		if (!vnodes[i]->GetFixed())
		{
			vnodes[i]->NodeSetOffset_x(this->GetOffset_x() + n_dofs);
			vnodes[i]->NodeSetOffset_w(this->GetOffset_w() + n_dofs_w);

				//    - count the degrees of freedom 
			n_dofs += vnodes[i]->Get_ndof_x();
			n_dofs_w += vnodes[i]->Get_ndof_w();
		}
	}
}




// Updates all time-dependant variables, if any...
// Ex: maybe the elasticity can increase in time, etc.

void ChMesh::Update(double m_time, bool update_assets)
{
  // Parent class update
  ChIndexedNodes::Update(m_time, update_assets);

  for (unsigned int i = 0; i < velements.size(); i++)
  {
    //    - update auxiliary stuff, ex. update element's rotation matrices if corotational..
    velements[i]->Update();
  }
}


void ChMesh::SyncCollisionModels() {
    for (unsigned int j = 0; j < vcontactsurfaces.size(); j++) {
        this->vcontactsurfaces[j]->SurfaceSyncCollisionModels();
    }
}

void ChMesh::AddCollisionModelsToSystem() {
    assert(this->GetSystem());
    SyncCollisionModels();
    for (unsigned int j = 0; j < vcontactsurfaces.size(); j++) {
        this->vcontactsurfaces[j]->SurfaceAddCollisionModelsToSystem(this->GetSystem());
    }
}

void ChMesh::RemoveCollisionModelsFromSystem() {
    assert(this->GetSystem());
    for (unsigned int j = 0; j < vcontactsurfaces.size(); j++) {
        this->vcontactsurfaces[j]->SurfaceRemoveCollisionModelsFromSystem(this->GetSystem());
    }
}




void ChMesh::LoadFromTetGenFile(const char* filename_node, const char* filename_ele, ChSharedPtr<ChContinuumMaterial> my_material)
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

		int nnodes = 0;
		int ndims = 0;
		int nattrs = 0;
		int nboundarymark = 0;

		string line;
		while(getline(fin, line)) 
		{
			//trims white space from the beginning of the string
			line.erase(line.begin(), find_if(line.begin(), line.end(), not1(ptr_fun<int, int>(isspace)))); 

			if(line[0] == '#') continue; // skip comment
			if(line[0] == 0) continue; // skip empty lines

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
				if (idnode <= 0 || idnode > nnodes)
					throw ChException("ERROR in TetGen .node file. Node ID not in range: \n"+ line +"\n");
				if (idnode != added_nodes)
					throw ChException("ERROR in TetGen .node file. Nodes IDs must be sequential (1 2 3 ..): \n"+ line+"\n");
				if (x == -10e30 || y == -10e30 || z == -10e30 )
					throw ChException("ERROR in TetGen .node file, in parsing x,y,z coordinates of node: \n"+ line+"\n");
				
				if (my_material.IsType<ChContinuumElastic>() )
				{
					ChSharedPtr<ChNodeFEAxyz> mnode( new ChNodeFEAxyz(ChVector<>(x,y,z)) );
					this->AddNode(mnode);
				}
				else if (my_material.IsType<ChContinuumPoisson3D>() )
				{
					ChSharedPtr<ChNodeFEAxyzP> mnode( new ChNodeFEAxyzP(ChVector<>(x,y,z)) );
					this->AddNode(mnode);
				}
				else throw ChException("ERROR in TetGen generation. Material type not supported. \n");

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

		int ntets, nnodespertet, nattrs = 0;

		string line;
		while(getline(fin, line)) 
		{
			//trims white space from the beginning of the string
			line.erase(line.begin(), find_if(line.begin(), line.end(), not1(ptr_fun<int, int>(isspace)))); 

			if(line[0] == '#') continue; // skip comment
			if(line[0] == 0) continue; // skip empty lines

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
				if (idtet <= 0 || idtet > ntets)
					throw ChException("ERROR in TetGen .node file. Tetahedron ID not in range: \n"+ line+"\n");
				if (n1 > totnodes)
					throw ChException("ERROR in TetGen .node file, ID of 1st node is out of range: \n"+ line+"\n");
				if (n2 > totnodes)
					throw ChException("ERROR in TetGen .node file, ID of 2nd node is out of range: \n"+ line+"\n");
				if (n3 > totnodes)
					throw ChException("ERROR in TetGen .node file, ID of 3rd node is out of range: \n"+ line+"\n");
				if (n4 > totnodes)
					throw ChException("ERROR in TetGen .node file, ID of 4th node is out of range: \n"+ line+"\n");
				
				if (my_material.IsType<ChContinuumElastic>() )
				{
					ChSharedPtr<ChElementTetra_4> mel( new ChElementTetra_4 );
					mel->SetNodes(
						this->GetNode(nodes_offset + n1-1).DynamicCastTo<ChNodeFEAxyz>(), 
						this->GetNode(nodes_offset + n3-1).DynamicCastTo<ChNodeFEAxyz>(), 
						this->GetNode(nodes_offset + n2-1).DynamicCastTo<ChNodeFEAxyz>(), 
						this->GetNode(nodes_offset + n4-1).DynamicCastTo<ChNodeFEAxyz>() );
					mel->SetMaterial(my_material.DynamicCastTo<ChContinuumElastic>());
					this->AddElement(mel);
				} 
				else if (my_material.IsType<ChContinuumPoisson3D>() )
				{
					ChSharedPtr<ChElementTetra_4_P> mel( new ChElementTetra_4_P );
					mel->SetNodes(
						this->GetNode(nodes_offset + n1-1).DynamicCastTo<ChNodeFEAxyzP>(), 
						this->GetNode(nodes_offset + n3-1).DynamicCastTo<ChNodeFEAxyzP>(), 
						this->GetNode(nodes_offset + n2-1).DynamicCastTo<ChNodeFEAxyzP>(), 
						this->GetNode(nodes_offset + n4-1).DynamicCastTo<ChNodeFEAxyzP>() );
					mel->SetMaterial(my_material.DynamicCastTo<ChContinuumPoisson3D>());
					this->AddElement(mel);
				}
				else throw ChException("ERROR in TetGen generation. Material type not supported. \n");

			}

		} // end while
        
    }// end .ele file
}




void ChMesh::LoadFromAbaqusFile(const char* filename, 
								ChSharedPtr<ChContinuumMaterial> my_material, 
								std::vector< std::vector< ChSharedPtr<ChNodeFEAbase> > >& node_sets,
                                ChVector<> pos_transform, ChMatrix33<> rot_transform, 
                                bool discard_unused_nodes)
{
	node_sets.resize(0);

    std::vector< ChSharedPtr<ChNodeFEAbase> > parsed_nodes;
    std::vector< bool > parsed_nodes_used;

	int totnodes = 0;
	unsigned int nodes_offset = this->GetNnodes();
	int added_nodes = 0;
	int added_elements = 0;

	enum eChAbaqusParserSection {
				E_PARSE_UNKNOWN = 0,
				E_PARSE_NODES_XYZ,
				E_PARSE_TETS_10,
				E_PARSE_NODESET
	} e_parse_section = E_PARSE_UNKNOWN;
	
	fstream fin(filename);
	if (!fin.good())
		throw ChException("ERROR opening Abaqus .inp file: " + std::string(filename) + "\n");

	int nnodes = 0;
	int ndims =  0;
	int nattrs = 0;

	string line;
	while(getline(fin, line)) 
	{
		//trims white space from the beginning of the string
		line.erase(line.begin(), find_if(line.begin(), line.end(), not1(ptr_fun<int, int>(isspace)))); 

		if(line[0] == 0) continue; // skip empty lines

		if(line[0] == '*') 
		{
			e_parse_section  = E_PARSE_UNKNOWN;

			if (line.find("*NODE") == 0)
			{
				string::size_type nse = line.find("NSET=");
				if (nse > 0)
				{
					string::size_type ncom = line.find(",",nse);
					string s_node_set = line.substr(nse+5,ncom-(nse+5));
					GetLog() << "Parsing: nodes " << s_node_set << "\n";
				}
				e_parse_section  = E_PARSE_NODES_XYZ;
			}
			if (line.find("*ELEMENT") == 0)
			{
				string::size_type nty = line.find("TYPE=");
				if (nty > 0)
				{
					string::size_type ncom = line.find(",",nty);
					string s_ele_type = line.substr(nty+5,ncom-(nty+5));
					if (s_ele_type != "C3D10" &&
						s_ele_type != "DC3D10" )
						throw ChException("ERROR in .inp file, TYPE=" + s_ele_type + " (only C3D10 or DC3D10 tetahedrons supported) see: \n" + line + "\n");
				}
				string::size_type nse = line.find("ELSET=");
				if (nse > 0)
				{
					string::size_type ncom = line.find(",",nse);
					string s_ele_set = line.substr(nse+6,ncom-(nse+6));
					GetLog() << "Parsing: element set: " << s_ele_set << "\n";
				}
				e_parse_section  = E_PARSE_TETS_10;
			}
			if (line.find("*NSET") == 0)
			{
				GetLog() << "Parsing: nodeset.. ";
				string::size_type nse = line.find("NSET=",5);
				if (nse > 0)
				{
					string::size_type ncom = line.find(",",nse);
					string s_node_set = line.substr(nse+5,ncom-(nse+5));
					GetLog() << "Parsing: nodeset: " << s_node_set << "\n";
					
					std::vector< ChSharedPtr<ChNodeFEAbase> > empty_set;
					node_sets.push_back(empty_set);
				}
				e_parse_section  = E_PARSE_NODESET;
			}

			continue; // skip 
		}
		

		if (e_parse_section == E_PARSE_NODES_XYZ)
		{
			int idnode = 0;
			double x = -10e30;
			double y = -10e30;
			double z = -10e30;
			double tokenvals[20];
			int ntoken = 0;

			string token;
			std::istringstream ss(line);
			while(getline(ss, token,',') && ntoken < 20) 
			{
				std::istringstream stoken(token);
				stoken >> tokenvals[ntoken]; 
				++ntoken;
			}
			++added_nodes;

			if (ntoken != 4)
				throw ChException("ERROR in .inp file, nodes require ID and three x y z coords, see line:\n"+ line+"\n");
			idnode = (int) tokenvals[0];
			if (idnode != added_nodes)
				throw ChException("ERROR in .inp file. Nodes IDs must be sequential (1 2 3 ..): \n"+ line+"\n");
			x = tokenvals[1];
			y = tokenvals[2];
			z = tokenvals[3];
			if (x == -10e30 || y == -10e30 || z == -10e30 )
				throw ChException("ERROR in in .inp file, in parsing x,y,z coordinates of node: \n"+ line+"\n");
			
            ChVector<> node_position(x,y,z);
            node_position = rot_transform * node_position; // rotate/scale, if needed
            node_position = pos_transform + node_position; // move, if needed

			if (my_material.IsType<ChContinuumElastic>() )
			{
				ChSharedPtr<ChNodeFEAxyz> mnode( new ChNodeFEAxyz(node_position) );
				parsed_nodes.push_back(mnode);
                parsed_nodes_used.push_back(false);
			}
			else if (my_material.IsType<ChContinuumPoisson3D>() )
			{
				ChSharedPtr<ChNodeFEAxyzP> mnode( new ChNodeFEAxyzP(ChVector<>(x,y,z)) );
                parsed_nodes.push_back(mnode);
                parsed_nodes_used.push_back(false);
			}
			else throw ChException("ERROR in .inp generation. Material type not supported. \n");

		}


		if (e_parse_section == E_PARSE_TETS_10)
		{
			int idelem = 0;
			unsigned int tokenvals[20];
			int ntoken = 0;

			string token;
			std::istringstream ss(line);
			while(std::getline(ss, token, ',') && ntoken < 20) 
			{
				std::istringstream stoken(token);
				stoken >> tokenvals[ntoken]; 
				++ntoken;
			}
			++added_elements;

			if (ntoken != 11)
				throw ChException("ERROR in .inp file, tetahedrons require ID and 10 node IDs, see line:\n"+ line+"\n");
			idelem = (int) tokenvals[0];
			if (idelem != added_elements)
				throw ChException("ERROR in .inp file. Element IDs must be sequential (1 2 3 ..): \n"+ line+"\n");
			for (int in = 0; in<10; ++in)
				if (tokenvals[in+1] == -10e30)
					throw ChException("ERROR in in .inp file, in parsing IDs of tetahedron: \n"+ line+"\n");
			
			if (my_material.IsType<ChContinuumElastic>() )
			{
				ChSharedPtr<ChElementTetra_4> mel( new ChElementTetra_4 );
				mel->SetNodes(
                    parsed_nodes[ tokenvals[4]-1 ].DynamicCastTo<ChNodeFEAxyz>(),
                    parsed_nodes[ tokenvals[2]-1 ].DynamicCastTo<ChNodeFEAxyz>(),
                    parsed_nodes[ tokenvals[3]-1 ].DynamicCastTo<ChNodeFEAxyz>(),
                    parsed_nodes[ tokenvals[1]-1 ].DynamicCastTo<ChNodeFEAxyz>() );
				mel->SetMaterial(my_material.DynamicCastTo<ChContinuumElastic>());
				this->AddElement(mel);
                parsed_nodes_used[ tokenvals[1]-1 ] = true;
                parsed_nodes_used[ tokenvals[2]-1 ] = true;
                parsed_nodes_used[ tokenvals[3]-1 ] = true;
                parsed_nodes_used[ tokenvals[4]-1 ] = true;
			} 
			else if (my_material.IsType<ChContinuumPoisson3D>() )
			{
				ChSharedPtr<ChElementTetra_4_P> mel( new ChElementTetra_4_P );
				mel->SetNodes(
                    parsed_nodes[ tokenvals[1]-1 ].DynamicCastTo<ChNodeFEAxyzP>(),
                    parsed_nodes[ tokenvals[2]-1 ].DynamicCastTo<ChNodeFEAxyzP>(),
                    parsed_nodes[ tokenvals[3]-1 ].DynamicCastTo<ChNodeFEAxyzP>(),
                    parsed_nodes[ tokenvals[4]-1 ].DynamicCastTo<ChNodeFEAxyzP>() );
				mel->SetMaterial(my_material.DynamicCastTo<ChContinuumPoisson3D>());
				this->AddElement(mel);
                parsed_nodes_used[ tokenvals[1]-1 ] = true;
                parsed_nodes_used[ tokenvals[2]-1 ] = true;
                parsed_nodes_used[ tokenvals[3]-1 ] = true;
                parsed_nodes_used[ tokenvals[4]-1 ] = true;
			}
			else throw ChException("ERROR in TetGen generation. Material type not supported. \n");

		}

		if (e_parse_section == E_PARSE_NODESET)
		{
			int idelem = 0;

			unsigned int tokenvals[100];
			int ntoken = 0;

			string token;
			std::istringstream ss(line);
			while(std::getline(ss, token, ',') && ntoken < 100) 
			{
				std::istringstream stoken(token);
				stoken >> tokenvals[ntoken]; 
				++ntoken;
			}

			for (int nt = 0; nt< ntoken; ++nt)
			{
				int idnode = (int) tokenvals[nt];
				node_sets.back().push_back( parsed_nodes[idnode -1].DynamicCastTo<ChNodeFEAbase>() );
                parsed_nodes_used[idnode -1] = true;
			}

		}


	} // end while
      
    // Add nodes to the mesh (only those effectively used for elements or node sets)
    for (unsigned int i = 0; i< parsed_nodes.size(); ++i) {
        if (parsed_nodes_used[i] == true)
            this->AddNode(parsed_nodes[i]);
    }
}


//// STATE BOOKKEEPING FUNCTIONS

void ChMesh::IntStateGather(
					const unsigned int off_x,		///< offset in x state vector
					ChState& x,						///< state vector, position part
					const unsigned int off_v,		///< offset in v state vector
					ChStateDelta& v,				///< state vector, speed part
					double& T)						///< time
{
	unsigned int local_off_x=0;
	unsigned int local_off_v=0;
	for (unsigned int j = 0; j < vnodes.size(); j++)
	{
		if (!vnodes[j]->GetFixed())
		{
			vnodes[j]->NodeIntStateGather(	off_x+local_off_x, 
											x, 
											off_v+local_off_v, 
											v, 
											T);
			local_off_x += vnodes[j]->Get_ndof_x();
			local_off_v += vnodes[j]->Get_ndof_w();
		}
	}

	T = this->GetChTime();
}

void ChMesh::IntStateScatter(
					const unsigned int off_x,		///< offset in x state vector
					const ChState& x,				///< state vector, position part
					const unsigned int off_v,		///< offset in v state vector
					const ChStateDelta& v,			///< state vector, speed part
					const double T) 				///< time
{
	unsigned int local_off_x=0;
	unsigned int local_off_v=0;
	for (unsigned int j = 0; j < vnodes.size(); j++)
	{
		if (!vnodes[j]->GetFixed())
		{
			vnodes[j]->NodeIntStateScatter(	off_x+local_off_x, 
											x, 
											off_v+local_off_v, 
											v, 
											T);
			local_off_x += vnodes[j]->Get_ndof_x();
			local_off_v += vnodes[j]->Get_ndof_w();
		}
	}

	this->Update(T);
}

void ChMesh::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a)
{
	unsigned int local_off_a=0;
	for (unsigned int j = 0; j < vnodes.size(); j++)
	{
		if (!vnodes[j]->GetFixed())
		{
			vnodes[j]->NodeIntStateGatherAcceleration(	off_a+local_off_a,  a);
			local_off_a += vnodes[j]->Get_ndof_w();
		}
	}
}

void ChMesh::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a)
{
	unsigned int local_off_a=0;
	for (unsigned int j = 0; j < vnodes.size(); j++)
	{
		if (!vnodes[j]->GetFixed())
		{
			vnodes[j]->NodeIntStateScatterAcceleration(	off_a+local_off_a, a);
			local_off_a += vnodes[j]->Get_ndof_w();
		}
	}
}

void ChMesh::IntStateIncrement(
					const unsigned int off_x,		///< offset in x state vector
					ChState& x_new,					///< state vector, position part, incremented result
					const ChState& x,				///< state vector, initial position part
					const unsigned int off_v,		///< offset in v state vector
					const ChStateDelta& Dv)  		///< state vector, increment
{
	unsigned int local_off_x=0;
	unsigned int local_off_v=0;
	for (unsigned int j = 0; j < vnodes.size(); j++)
	{
		if (!vnodes[j]->GetFixed())
		{
			vnodes[j]->NodeIntStateIncrement(	off_x+local_off_x, 
												x_new, 
												x, 
												off_v+local_off_v, 
												Dv);
			local_off_x += vnodes[j]->Get_ndof_x();
			local_off_v += vnodes[j]->Get_ndof_w();
		}
	}
}

void ChMesh::IntLoadResidual_F(
					const unsigned int off,		 ///< offset in R residual (not used here! use particle's offsets)
					ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*F 
					const double c				 ///< a scaling factor
					)
{
	// applied nodal forces
	unsigned int local_off_v=0;
	for (unsigned int j = 0; j < vnodes.size(); j++)
	{
		if (!vnodes[j]->GetFixed())
		{
			this->vnodes[j]->NodeIntLoadResidual_F(	off+local_off_v, 
													R, 
													c);
			local_off_v += vnodes[j]->Get_ndof_w();
		}
	}

	// internal forces
	for (unsigned int ie = 0; ie < this->velements.size(); ie++)
	{
		this->velements[ie]->EleIntLoadResidual_F(R, c);
	}

    // Apply gravity loads without the need of adding 
    // a ChLoad object to each element: just instance here a single ChLoad and reuse 
    // it for all 'volume' objects.
    if (automatic_gravity_load) {
        ChSharedPtr< ChLoadableUVW > mloadable;// still null
        ChSharedPtr< ChLoad< ChLoaderGravity > > common_gravity_loader(new ChLoad< ChLoaderGravity >( mloadable ));
        common_gravity_loader->loader.Set_G_acc( this->GetSystem()->Get_G_acc() );
		common_gravity_loader->loader.SetNumIntPoints(num_points_gravity);

        for (unsigned int ie = 0; ie < this->velements.size(); ie++) {
            mloadable = this->velements[ie].DynamicCastTo<ChLoadableUVW>();
            if (mloadable) {
                if (mloadable->GetDensity()) {
                    // temporary set loader target and compute generalized forces term
                    common_gravity_loader->loader.loadable = mloadable;
                    common_gravity_loader->ComputeQ(0,0);
                    common_gravity_loader->LoadIntLoadResidual_F(R, c);
                }
            }
	    }
    }
}


void ChMesh::IntLoadResidual_Mv(
					const unsigned int off,		 ///< offset in R residual
					ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*M*v 
					const ChVectorDynamic<>& w,  ///< the w vector 
					const double c				 ///< a scaling factor
					)
{
	// nodal masses
	unsigned int local_off_v=0;
	for (unsigned int j = 0; j < vnodes.size(); j++)
	{
		if (!vnodes[j]->GetFixed())
		{
			vnodes[j]->NodeIntLoadResidual_Mv(	off+local_off_v,  R, w, c);
			local_off_v += vnodes[j]->Get_ndof_w();
		}
	}

	// internal masses
	for (unsigned int ie = 0; ie < this->velements.size(); ie++)
	{
		this->velements[ie]->EleIntLoadResidual_Mv(R, w, c);
	}
}

void ChMesh::IntToLCP(
					const unsigned int off_v,			///< offset in v, R
					const ChStateDelta& v,
					const ChVectorDynamic<>& R,
					const unsigned int off_L,			///< offset in L, Qc
					const ChVectorDynamic<>& L,
					const ChVectorDynamic<>& Qc
					)
{
	unsigned int local_off_v=0;
	for (unsigned int j = 0; j < vnodes.size(); j++)
	{
		if (!vnodes[j]->GetFixed())
		{
			vnodes[j]->NodeIntToLCP(off_v + local_off_v,  v, R);
			local_off_v += vnodes[j]->Get_ndof_w();
		}
	}
}

void ChMesh::IntFromLCP(
					const unsigned int off_v,			///< offset in v
					ChStateDelta& v,
					const unsigned int off_L,			///< offset in L
					ChVectorDynamic<>& L
					)
{
	unsigned int local_off_v=0;
	for (unsigned int j = 0; j < vnodes.size(); j++)
	{
		if (!vnodes[j]->GetFixed())
		{
			vnodes[j]->NodeIntFromLCP(off_v + local_off_v,  v);
			local_off_v += vnodes[j]->Get_ndof_w();
		}
	}
}




//// LCP SOLVER

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
		this->vnodes[ie]->InjectVariables(mdescriptor);

		//mdescriptor.InsertVariables(&this->vnodes[ie]->Variables());
}






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

