// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Andrea Favali, Alessandro Tasora
// =============================================================================
// Utilities for loading meshes from file
// =============================================================================

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
#include <functional> 

#include "chrono/core/ChMath.h"
#include "chrono/physics/ChObject.h"
#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_fea/ChMeshFileLoader.h"
#include "chrono_fea/ChNodeFEAxyz.h"
#include "chrono_fea/ChElementTetra_4.h"


using namespace std;


namespace chrono  {
namespace fea {

void ChMeshFileLoader::FromTetGenFile(ChSharedPtr<ChMesh> mesh,
                                      const char* filename_node,
                                      const char* filename_ele,
                                      ChSharedPtr<ChContinuumMaterial> my_material,
                                      ChVector<> pos_transform,
                                      ChMatrix33<> rot_transform) {
    int totnodes = 0;
    int nodes_offset = mesh->GetNnodes();
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
        while (getline(fin, line)) {
            // trims white space from the beginning of the string
            line.erase(line.begin(), find_if(line.begin(), line.end(), not1(ptr_fun<int, int>(isspace))));

            if (line[0] == '#')
                continue;  // skip comment
            if (line[0] == 0)
                continue;  // skip empty lines

            if (parse_header) {
                stringstream(line) >> nnodes >> ndims >> nattrs >> nboundarymark;
                if (ndims != 3)
                    throw ChException("ERROR in TetGen .node file. Only 3 dimensional nodes supported: \n" + line);
                if (nattrs != 0)
                    throw ChException("ERROR in TetGen .node file. Only nodes with 0 attrs supported: \n" + line);
                if (nboundarymark != 0)
                    throw ChException("ERROR in TetGen .node file. Only nodes with 0 markers supported: \n" + line);
                parse_header = false;
                parse_nodes = true;
                totnodes = nnodes;
                continue;
            }

            int idnode = 0;
			double x = -10e30;
			double y = -10e30;
			double z = -10e30;

            if (parse_nodes) {
                stringstream(line) >> idnode >> x >> y >> z;
                ++added_nodes;
                if (idnode <= 0 || idnode > nnodes)
                    throw ChException("ERROR in TetGen .node file. Node ID not in range: \n" + line + "\n");
                if (idnode != added_nodes)
                    throw ChException("ERROR in TetGen .node file. Nodes IDs must be sequential (1 2 3 ..): \n" + line +
                                      "\n");
                if (x == -10e30 || y == -10e30 || z == -10e30)
                    throw ChException("ERROR in TetGen .node file, in parsing x,y,z coordinates of node: \n" + line +
                                      "\n");

                ChVector<> node_position(x, y, z);
                node_position = rot_transform * node_position;  // rotate/scale, if needed
                node_position = pos_transform + node_position;  // move, if needed

                if (my_material.IsType<ChContinuumElastic>()) {
                    ChSharedPtr<ChNodeFEAxyz> mnode(new ChNodeFEAxyz(node_position));
                    mesh->AddNode(mnode);
                } else if (my_material.IsType<ChContinuumPoisson3D>()) {
                    ChSharedPtr<ChNodeFEAxyzP> mnode(new ChNodeFEAxyzP(node_position));
                    mesh->AddNode(mnode);
                } else
                    throw ChException("ERROR in TetGen generation. Material type not supported. \n");
            }

        }  // end while

    } // end .node file

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
                    throw ChException("ERROR in TetGen .node file, ID of 4th node is out of range: \n" + line + "\n");

                if (my_material.IsType<ChContinuumElastic>()) {
                    ChSharedPtr<ChElementTetra_4> mel(new ChElementTetra_4);
                    mel->SetNodes(mesh->GetNode(nodes_offset + n1 - 1).DynamicCastTo<ChNodeFEAxyz>(),
                                  mesh->GetNode(nodes_offset + n3 - 1).DynamicCastTo<ChNodeFEAxyz>(),
                                  mesh->GetNode(nodes_offset + n2 - 1).DynamicCastTo<ChNodeFEAxyz>(),
                                  mesh->GetNode(nodes_offset + n4 - 1).DynamicCastTo<ChNodeFEAxyz>());
                    mel->SetMaterial(my_material.DynamicCastTo<ChContinuumElastic>());
                    mesh->AddElement(mel);
                } else if (my_material.IsType<ChContinuumPoisson3D>()) {
                    ChSharedPtr<ChElementTetra_4_P> mel(new ChElementTetra_4_P);
                    mel->SetNodes(mesh->GetNode(nodes_offset + n1 - 1).DynamicCastTo<ChNodeFEAxyzP>(),
                                  mesh->GetNode(nodes_offset + n3 - 1).DynamicCastTo<ChNodeFEAxyzP>(),
                                  mesh->GetNode(nodes_offset + n2 - 1).DynamicCastTo<ChNodeFEAxyzP>(),
                                  mesh->GetNode(nodes_offset + n4 - 1).DynamicCastTo<ChNodeFEAxyzP>());
                    mel->SetMaterial(my_material.DynamicCastTo<ChContinuumPoisson3D>());
                    mesh->AddElement(mel);
                } else
                    throw ChException("ERROR in TetGen generation. Material type not supported. \n");
            }

        }  // end while

    } // end .ele file
}

void ChMeshFileLoader::FromAbaqusFile(ChSharedPtr<ChMesh> mesh,
                                      const char* filename,
                                      ChSharedPtr<ChContinuumMaterial> my_material,
                                      std::vector<std::vector<ChSharedPtr<ChNodeFEAbase> > >& node_sets,
                                      ChVector<> pos_transform,
                                      ChMatrix33<> rot_transform,
                                      bool discard_unused_nodes) {
    node_sets.resize(0);

    std::vector< ChSharedPtr<ChNodeFEAbase> > parsed_nodes;
    std::vector< bool > parsed_nodes_used;

	int totnodes = 0;
	unsigned int nodes_offset = mesh->GetNnodes();
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
    while (getline(fin, line)) {
        // trims white space from the beginning of the string
        line.erase(line.begin(), find_if(line.begin(), line.end(), not1(ptr_fun<int, int>(isspace))));

        if (line[0] == 0)
            continue;  // skip empty lines

        if (line[0] == '*') {
            e_parse_section = E_PARSE_UNKNOWN;

            if (line.find("*NODE") == 0) {
                string::size_type nse = line.find("NSET=");
                if (nse > 0) {
                    string::size_type ncom = line.find(",", nse);
                    string s_node_set = line.substr(nse + 5, ncom - (nse + 5));
                    GetLog() << "Parsing: nodes " << s_node_set << "\n";
                }
                e_parse_section = E_PARSE_NODES_XYZ;
            }
            if (line.find("*ELEMENT") == 0) {
                string::size_type nty = line.find("TYPE=");
                if (nty > 0) {
                    string::size_type ncom = line.find(",", nty);
                    string s_ele_type = line.substr(nty + 5, ncom - (nty + 5));
                    if (s_ele_type != "C3D10" && s_ele_type != "DC3D10")
                        throw ChException("ERROR in .inp file, TYPE=" + s_ele_type +
                                          " (only C3D10 or DC3D10 tetahedrons supported) see: \n" + line + "\n");
                }
                string::size_type nse = line.find("ELSET=");
                if (nse > 0) {
                    string::size_type ncom = line.find(",", nse);
                    string s_ele_set = line.substr(nse + 6, ncom - (nse + 6));
                    GetLog() << "Parsing: element set: " << s_ele_set << "\n";
                }
                e_parse_section = E_PARSE_TETS_10;
            }
            if (line.find("*NSET") == 0) {
                GetLog() << "Parsing: nodeset.. ";
                string::size_type nse = line.find("NSET=", 5);
                if (nse > 0) {
                    string::size_type ncom = line.find(",", nse);
                    string s_node_set = line.substr(nse + 5, ncom - (nse + 5));
                    GetLog() << "Parsing: nodeset: " << s_node_set << "\n";

                    std::vector<ChSharedPtr<ChNodeFEAbase> > empty_set;
                    node_sets.push_back(empty_set);
                }
                e_parse_section = E_PARSE_NODESET;
            }

            continue;  // skip
        }

        if (e_parse_section == E_PARSE_NODES_XYZ) {
            int idnode = 0;
            double x = -10e30;
            double y = -10e30;
            double z = -10e30;
            double tokenvals[20];
            int ntoken = 0;

            string token;
            std::istringstream ss(line);
            while (getline(ss, token, ',') && ntoken < 20) {
                std::istringstream stoken(token);
                stoken >> tokenvals[ntoken];
                ++ntoken;
            }
            ++added_nodes;

            if (ntoken != 4)
                throw ChException("ERROR in .inp file, nodes require ID and three x y z coords, see line:\n" + line +
                                  "\n");
            idnode = (int)tokenvals[0];
            if (idnode != added_nodes)
                throw ChException("ERROR in .inp file. Nodes IDs must be sequential (1 2 3 ..): \n" + line + "\n");
            x = tokenvals[1];
            y = tokenvals[2];
            z = tokenvals[3];
            if (x == -10e30 || y == -10e30 || z == -10e30)
                throw ChException("ERROR in in .inp file, in parsing x,y,z coordinates of node: \n" + line + "\n");

            ChVector<> node_position(x, y, z);
            node_position = rot_transform * node_position;  // rotate/scale, if needed
            node_position = pos_transform + node_position;  // move, if needed

            if (my_material.IsType<ChContinuumElastic>()) {
                ChSharedPtr<ChNodeFEAxyz> mnode(new ChNodeFEAxyz(node_position));
                parsed_nodes.push_back(mnode);
                parsed_nodes_used.push_back(false);
            } else if (my_material.IsType<ChContinuumPoisson3D>()) {
                ChSharedPtr<ChNodeFEAxyzP> mnode(new ChNodeFEAxyzP(ChVector<>(x, y, z)));
                parsed_nodes.push_back(mnode);
                parsed_nodes_used.push_back(false);
            } else
                throw ChException("ERROR in .inp generation. Material type not supported. \n");
        }

        if (e_parse_section == E_PARSE_TETS_10) {
            int idelem = 0;
            unsigned int tokenvals[20];
            int ntoken = 0;

            string token;
            std::istringstream ss(line);
            while (std::getline(ss, token, ',') && ntoken < 20) {
                std::istringstream stoken(token);
                stoken >> tokenvals[ntoken];
                ++ntoken;
            }
            ++added_elements;

            if (ntoken != 11)
                throw ChException("ERROR in .inp file, tetahedrons require ID and 10 node IDs, see line:\n" + line +
                                  "\n");
            idelem = (int)tokenvals[0];
            if (idelem != added_elements)
                throw ChException("ERROR in .inp file. Element IDs must be sequential (1 2 3 ..): \n" + line + "\n");
            for (int in = 0; in < 10; ++in)
                if (tokenvals[in + 1] == -10e30)
                    throw ChException("ERROR in in .inp file, in parsing IDs of tetahedron: \n" + line + "\n");

            if (my_material.IsType<ChContinuumElastic>()) {
                ChSharedPtr<ChElementTetra_4> mel(new ChElementTetra_4);
                mel->SetNodes(parsed_nodes[tokenvals[4] - 1].DynamicCastTo<ChNodeFEAxyz>(),
                              parsed_nodes[tokenvals[2] - 1].DynamicCastTo<ChNodeFEAxyz>(),
                              parsed_nodes[tokenvals[3] - 1].DynamicCastTo<ChNodeFEAxyz>(),
                              parsed_nodes[tokenvals[1] - 1].DynamicCastTo<ChNodeFEAxyz>());
                mel->SetMaterial(my_material.DynamicCastTo<ChContinuumElastic>());
                mesh->AddElement(mel);
                parsed_nodes_used[tokenvals[1] - 1] = true;
                parsed_nodes_used[tokenvals[2] - 1] = true;
                parsed_nodes_used[tokenvals[3] - 1] = true;
                parsed_nodes_used[tokenvals[4] - 1] = true;
            } else if (my_material.IsType<ChContinuumPoisson3D>()) {
                ChSharedPtr<ChElementTetra_4_P> mel(new ChElementTetra_4_P);
                mel->SetNodes(parsed_nodes[tokenvals[1] - 1].DynamicCastTo<ChNodeFEAxyzP>(),
                              parsed_nodes[tokenvals[2] - 1].DynamicCastTo<ChNodeFEAxyzP>(),
                              parsed_nodes[tokenvals[3] - 1].DynamicCastTo<ChNodeFEAxyzP>(),
                              parsed_nodes[tokenvals[4] - 1].DynamicCastTo<ChNodeFEAxyzP>());
                mel->SetMaterial(my_material.DynamicCastTo<ChContinuumPoisson3D>());
                mesh->AddElement(mel);
                parsed_nodes_used[tokenvals[1] - 1] = true;
                parsed_nodes_used[tokenvals[2] - 1] = true;
                parsed_nodes_used[tokenvals[3] - 1] = true;
                parsed_nodes_used[tokenvals[4] - 1] = true;
            } else
                throw ChException("ERROR in TetGen generation. Material type not supported. \n");
        }

        if (e_parse_section == E_PARSE_NODESET) {
            int idelem = 0;

            unsigned int tokenvals[100];
            int ntoken = 0;

            string token;
            std::istringstream ss(line);
            while (std::getline(ss, token, ',') && ntoken < 100) {
                std::istringstream stoken(token);
                stoken >> tokenvals[ntoken];
                ++ntoken;
            }

            for (int nt = 0; nt < ntoken; ++nt) {
                int idnode = (int)tokenvals[nt];
                if (idnode > 0) {
                    node_sets.back().push_back(parsed_nodes[idnode - 1].DynamicCastTo<ChNodeFEAbase>());
                    parsed_nodes_used[idnode - 1] = true;
                }
            }
        }

    }  // end while

    // Add nodes to the mesh (only those effectively used for elements or node sets)
    for (unsigned int i = 0; i < parsed_nodes.size(); ++i) {
        if (parsed_nodes_used[i] == true)
            mesh->AddNode(parsed_nodes[i]);
    }
}

} // end namespace fea
} // end namespace chrono

