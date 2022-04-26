// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
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

#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <cctype>

#include "chrono/core/ChMath.h"
#include "chrono/physics/ChSystem.h"

#include <array>
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChNodeFEAxyz.h"

#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace chrono {
namespace fea {

void ChMeshFileLoader::FromTetGenFile(std::shared_ptr<ChMesh> mesh,
                                      const char* filename_node,
                                      const char* filename_ele,
                                      std::shared_ptr<ChContinuumMaterial> my_material,
                                      ChVector<> pos_transform,
                                      ChMatrix33<> rot_transform) {
    int totnodes = 0;
    int nodes_offset = mesh->GetNnodes();
    int added_nodes = 0;

    // Load .node TetGen file
    {
        bool parse_header = true;
        bool parse_nodes = false;

        std::ifstream fin(filename_node);
        if (!fin.good())
            throw ChException("ERROR opening TetGen .node file: " + std::string(filename_node) + "\n");

        int nnodes = 0;
        int ndims = 0;
        int nattrs = 0;
        int nboundarymark = 0;

        std::string line;
        while (std::getline(fin, line)) {
            // trims white space from the beginning of the string
            line.erase(line.begin(),
                       std::find_if(line.begin(), line.end(), [](unsigned char c) { return !std::isspace(c); }));

            if (line[0] == '#')
                continue;  // skip comment
            if (line[0] == 0)
                continue;  // skip empty lines

            if (parse_header) {
                std::stringstream(line) >> nnodes >> ndims >> nattrs >> nboundarymark;
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
                std::stringstream(line) >> idnode >> x >> y >> z;
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

                if (std::dynamic_pointer_cast<ChContinuumElastic>(my_material)) {
                    auto mnode = chrono_types::make_shared<ChNodeFEAxyz>(node_position);
                    mesh->AddNode(mnode);
                } else if (std::dynamic_pointer_cast<ChContinuumPoisson3D>(my_material)) {
                    auto mnode = chrono_types::make_shared<ChNodeFEAxyzP>(node_position);
                    mesh->AddNode(mnode);
                } else
                    throw ChException("ERROR in TetGen generation. Material type not supported. \n");
            }

        }  // end while

    }  // end .node file

    // Load .ele TetGen file
    {
        bool parse_header = true;
        bool parse_tet = false;

        std::ifstream fin(filename_ele);
        if (!fin.good())
            throw ChException("ERROR opening TetGen .node file: " + std::string(filename_node) + "\n");

        int ntets = 0, nnodespertet, nattrs = 0;

        std::string line;
        while (std::getline(fin, line)) {
            // trims white space from the beginning of the string
            line.erase(line.begin(),
                       std::find_if(line.begin(), line.end(), [](unsigned char c) { return !std::isspace(c); }));

            if (line[0] == '#')
                continue;  // skip comment
            if (line[0] == 0)
                continue;  // skip empty lines

            if (parse_header) {
                std::stringstream(line) >> ntets >> nnodespertet >> nattrs;
                if (nnodespertet != 4)
                    throw ChException("ERROR in TetGen .ele file. Only 4 -nodes per tes supported: \n" + line + "\n");
                if (nattrs != 0)
                    throw ChException("ERROR in TetGen .ele file. Only tets with 0 attrs supported: \n" + line + "\n");
                parse_header = false;
                parse_tet = true;
                continue;
            }

            int idtet = 0;
            int n1, n2, n3, n4;

            if (parse_tet) {
                std::stringstream(line) >> idtet >> n1 >> n2 >> n3 >> n4;
                if (idtet <= 0 || idtet > ntets)
                    throw ChException("ERROR in TetGen .node file. Tetrahedron ID not in range: \n" + line + "\n");
                if (n1 > totnodes)
                    throw ChException("ERROR in TetGen .node file, ID of 1st node is out of range: \n" + line + "\n");
                if (n2 > totnodes)
                    throw ChException("ERROR in TetGen .node file, ID of 2nd node is out of range: \n" + line + "\n");
                if (n3 > totnodes)
                    throw ChException("ERROR in TetGen .node file, ID of 3rd node is out of range: \n" + line + "\n");
                if (n4 > totnodes)
                    throw ChException("ERROR in TetGen .node file, ID of 4th node is out of range: \n" + line + "\n");
                if (std::dynamic_pointer_cast<ChContinuumElastic>(my_material)) {
                    auto mel = chrono_types::make_shared<ChElementTetraCorot_4>();
                    mel->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(nodes_offset + n1 - 1)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(nodes_offset + n3 - 1)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(nodes_offset + n2 - 1)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(nodes_offset + n4 - 1)));
                    mel->SetMaterial(std::static_pointer_cast<ChContinuumElastic>(my_material));
                    mesh->AddElement(mel);
                } else if (std::dynamic_pointer_cast<ChContinuumPoisson3D>(my_material)) {
                    auto mel = chrono_types::make_shared<ChElementTetraCorot_4_P>();
                    mel->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzP>(mesh->GetNode(nodes_offset + n1 - 1)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzP>(mesh->GetNode(nodes_offset + n3 - 1)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzP>(mesh->GetNode(nodes_offset + n2 - 1)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzP>(mesh->GetNode(nodes_offset + n4 - 1)));
                    mel->SetMaterial(std::static_pointer_cast<ChContinuumPoisson3D>(my_material));
                    mesh->AddElement(mel);
                } else
                    throw ChException("ERROR in TetGen generation. Material type not supported. \n");
            }

        }  // end while

    }  // end .ele file
}

void ChMeshFileLoader::FromAbaqusFile(std::shared_ptr<ChMesh> mesh,
                                      const char* filename,
                                      std::shared_ptr<ChContinuumMaterial> my_material,
                                      std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase>>>& node_sets,
                                      ChVector<> pos_transform,
                                      ChMatrix33<> rot_transform,
                                      bool discard_unused_nodes) {
    std::map<unsigned int, std::pair<std::shared_ptr<ChNodeFEAbase>, bool>> parsed_nodes;
    std::vector<std::shared_ptr<ChNodeFEAbase>>* current_nodeset_vector = nullptr;

    enum eChAbaqusParserSection {
        E_PARSE_UNKNOWN = 0,
        E_PARSE_NODES_XYZ,
        E_PARSE_TETS_4,
        E_PARSE_TETS_10,
        E_PARSE_NODESET
    } e_parse_section = E_PARSE_UNKNOWN;

    std::ifstream fin(filename);
    if (fin.good())
        GetLog() << "Parsing Abaqus INP file: " << filename << "\n";
    else
        throw ChException("ERROR opening Abaqus .inp file: " + std::string(filename) + "\n");

    std::string line;
    while (std::getline(fin, line)) {
        // trims white space from the beginning of the string
        line.erase(line.begin(),
                   std::find_if(line.begin(), line.end(), [](unsigned char c) { return !std::isspace(c); }));

        // convert parsed line to uppercase (since string::find is case sensitive and Abaqus INP is not)
        std::for_each(line.begin(), line.end(), [](char& c) { c = toupper(static_cast<unsigned char>(c)); });

        // skip empty lines
        if (line[0] == 0)
            continue;

        // check if the current line opens a new section
        if (line[0] == '*') {
            e_parse_section = E_PARSE_UNKNOWN;

            if (line.find("*NODE") == 0) {
                std::string::size_type nse = line.find("NSET=");
                if (nse > 0) {
                    std::string::size_type ncom = line.find(",", nse);
                    std::string s_node_set = line.substr(nse + 5, ncom - (nse + 5));
                    GetLog() << "| parsing nodes " << s_node_set << "\n";
                }
                e_parse_section = E_PARSE_NODES_XYZ;
            }

            if (line.find("*ELEMENT") == 0) {
                std::string::size_type nty = line.find("TYPE=");
                if (nty > 0) {
                    std::string::size_type ncom = line.find(",", nty);
                    std::string s_ele_type = line.substr(nty + 5, ncom - (nty + 5));
                    e_parse_section = E_PARSE_UNKNOWN;
                    if (s_ele_type == "C3D10") {
                        e_parse_section = E_PARSE_TETS_10;
                    } else if (s_ele_type == "DC3D10") {
                        e_parse_section = E_PARSE_TETS_10;
                    } else if (s_ele_type == "C3D4") {
                        e_parse_section = E_PARSE_TETS_4;
                    }
                    if (e_parse_section == E_PARSE_UNKNOWN) {
                        throw ChException("ERROR in .inp file, TYPE=" + s_ele_type +
                                          " (only C3D10 or DC3D10 or C3D4 tetrahedrons supported) see: \n" + line +
                                          "\n");
                    }
                }
                std::string::size_type nse = line.find("ELSET=");
                if (nse > 0) {
                    std::string::size_type ncom = line.find(",", nse);
                    std::string s_ele_set = line.substr(nse + 6, ncom - (nse + 6));
                    GetLog() << "| parsing element set: " << s_ele_set << "\n";
                }
            }

            if (line.find("*NSET") == 0) {
                std::string::size_type nse = line.find("NSET=", 5);
                if (nse > 0) {
                    std::string::size_type ncom = line.find(",", nse);
                    std::string s_node_set = line.substr(nse + 5, ncom - (nse + 5));
                    GetLog() << "| parsing nodeset: " << s_node_set << "\n";
                    auto new_node =
                        node_sets.insert(std::pair<std::string, std::vector<std::shared_ptr<ChNodeFEAbase>>>(
                            s_node_set, std::vector<std::shared_ptr<ChNodeFEAbase>>()));
                    if (new_node.second) {
                        current_nodeset_vector = &new_node.first->second;
                    } else
                        throw ChException("ERROR in .inp file, multiple NSET with same name has been specified\n");
                }
                e_parse_section = E_PARSE_NODESET;
            }

            continue;
        }

        // node parsing
        if (e_parse_section == E_PARSE_NODES_XYZ) {
            int idnode = 0;
            double x = -10e30;
            double y = -10e30;
            double z = -10e30;
            double tokenvals[20];
            int ntoken = 0;

            std::string token;
            std::istringstream ss(line);
            while (std::getline(ss, token, ',') && ntoken < 20) {
                std::istringstream stoken(token);
                stoken >> tokenvals[ntoken];
                ++ntoken;
            }

            if (ntoken != 4)
                throw ChException("ERROR in .inp file, nodes require ID and three x y z coords, see line:\n" + line +
                                  "\n");

            x = tokenvals[1];
            y = tokenvals[2];
            z = tokenvals[3];
            if (x == -10e30 || y == -10e30 || z == -10e30)
                throw ChException("ERROR in .inp file, in parsing x,y,z coordinates of node: \n" + line + "\n");

            // TODO: is it worth to keep a so specific routine inside this function?
            // especially considering that is affecting only some types of elements...
            ChVector<> node_position(x, y, z);
            node_position = rot_transform * node_position;  // rotate/scale, if needed
            node_position = pos_transform + node_position;  // move, if needed

            idnode = static_cast<unsigned int>(tokenvals[0]);
            if (std::dynamic_pointer_cast<ChContinuumElastic>(my_material)) {
                auto mnode = chrono_types::make_shared<ChNodeFEAxyz>(node_position);
                mnode->SetIndex(idnode);
                parsed_nodes[idnode] = std::make_pair(mnode, false);
                if (!discard_unused_nodes)
                    mesh->AddNode(mnode);
            } else if (std::dynamic_pointer_cast<ChContinuumPoisson3D>(my_material)) {
                auto mnode = chrono_types::make_shared<ChNodeFEAxyzP>(ChVector<>(x, y, z));
                mnode->SetIndex(idnode);
                parsed_nodes[idnode] = std::make_pair(mnode, false);
                if (!discard_unused_nodes)
                    mesh->AddNode(mnode);
            } else
                throw ChException("ERROR in .inp generation. Material type not supported. \n");
        }

        // element parsing
        if (e_parse_section == E_PARSE_TETS_10 || e_parse_section == E_PARSE_TETS_4) {
            ////int idelem = 0;
            unsigned int tokenvals[20];
            int ntoken = 0;

            std::string token;
            std::istringstream ss(line);
            while (std::getline(ss, token, ',') && ntoken < 20) {
                std::istringstream stoken(token);
                stoken >> tokenvals[ntoken];
                ++ntoken;
            }

            if (e_parse_section == E_PARSE_TETS_10) {
                if (ntoken != 11)
                    throw ChException("ERROR in .inp file, tetrahedrons require ID and 10 node IDs, see line:\n" +
                                      line + "\n");
                // TODO: 'idelem' might be stored in an index in ChElementBase in order to provide consistency with the
                // INP file
                ////idelem = (int)tokenvals[0];

                for (int in = 0; in < 10; ++in)
                    if (tokenvals[in + 1] == -10e30)
                        throw ChException("ERROR in in .inp file, in parsing IDs of tetrahedron: \n" + line + "\n");
            } else if (e_parse_section == E_PARSE_TETS_4) {
                if (ntoken != 5)
                    throw ChException("ERROR in .inp file, tetrahedrons require ID and 4 node IDs, see line:\n" + line +
                                      "\n");

                // TODO: 'idelem' might be stored in an index in ChElementBase in order to provide consistency with the
                // INP file
                ////idelem = (int)tokenvals[0];

                for (int in = 0; in < 4; ++in)
                    if (tokenvals[in + 1] == -10e30)
                        throw ChException("ERROR in in .inp file, in parsing IDs of tetrahedron: \n" + line + "\n");
            }

            if (std::dynamic_pointer_cast<ChContinuumElastic>(my_material) ||
                std::dynamic_pointer_cast<ChContinuumPoisson3D>(my_material)) {
                std::array<std::shared_ptr<ChNodeFEAbase>, 4> element_nodes;
                for (auto node_sel = 0; node_sel < 4; ++node_sel) {
                    // check if the nodes required by the current element exist
                    std::pair<std::shared_ptr<ChNodeFEAbase>, bool>& node_found =
                        parsed_nodes.at(static_cast<unsigned int>(tokenvals[node_sel + 1]));

                    element_nodes[node_sel] = node_found.first;
                    node_found.second = true;
                }

                if (std::dynamic_pointer_cast<ChContinuumElastic>(my_material)) {
                    auto mel = chrono_types::make_shared<ChElementTetraCorot_4>();
                    mel->SetNodes(std::static_pointer_cast<ChNodeFEAxyz>(element_nodes[3]),
                                  std::static_pointer_cast<ChNodeFEAxyz>(element_nodes[1]),
                                  std::static_pointer_cast<ChNodeFEAxyz>(element_nodes[2]),
                                  std::static_pointer_cast<ChNodeFEAxyz>(element_nodes[0]));
                    mel->SetMaterial(std::static_pointer_cast<ChContinuumElastic>(my_material));
                    mesh->AddElement(mel);

                } else if (std::dynamic_pointer_cast<ChContinuumPoisson3D>(my_material)) {
                    auto mel = chrono_types::make_shared<ChElementTetraCorot_4_P>();
                    mel->SetNodes(std::static_pointer_cast<ChNodeFEAxyzP>(element_nodes[0]),
                                  std::static_pointer_cast<ChNodeFEAxyzP>(element_nodes[1]),
                                  std::static_pointer_cast<ChNodeFEAxyzP>(element_nodes[2]),
                                  std::static_pointer_cast<ChNodeFEAxyzP>(element_nodes[3]));
                    mel->SetMaterial(std::static_pointer_cast<ChContinuumPoisson3D>(my_material));
                    mesh->AddElement(mel);
                }

            } else
                throw ChException("ERROR in .inp generation. Material type not supported.\n");
        }

        // parsing nodesets
        if (e_parse_section == E_PARSE_NODESET) {
            unsigned int tokenvals[20];  // strictly speaking, the maximum is 16 nodes for each line
            int ntoken = 0;

            std::string token;
            std::istringstream ss(line);
            while (std::getline(ss, token, ',') && ntoken < 20) {
                if (!token.empty() && token.back() == '\r') {
                    // Fix for possible Windows line ending (\r\n)
                    break;
                }
                std::istringstream stoken(token);
                stoken >> tokenvals[ntoken];
                ++ntoken;
            }

            for (auto node_sel = 0; node_sel < ntoken; ++node_sel) {
                auto idnode = static_cast<int>(tokenvals[node_sel]);
                if (idnode > 0) {
                    // check if the nodeset is asking for an existing node
                    std::pair<std::shared_ptr<ChNodeFEAbase>, bool>& node_found =
                        parsed_nodes.at(static_cast<unsigned int>(tokenvals[node_sel]));

                    current_nodeset_vector->push_back(node_found.first);
                    // flag the node to be saved later into the mesh
                    node_found.second = true;

                } else
                    throw ChException("ERROR in .inp file, negative node ID: " + std::to_string(tokenvals[node_sel]));
            }
        }

    }  // end while

    // only used nodes have been saved in 'parsed_nodes_used' and are now inserted in the mesh node list
    if (discard_unused_nodes) {
        for (auto node_it = parsed_nodes.begin(); node_it != parsed_nodes.end(); ++node_it) {
            if (node_it->second.second)
                mesh->AddNode(node_it->second.first);
        }
    }
}

void ChMeshFileLoader::ANCFShellFromGMFFile(std::shared_ptr<ChMesh> mesh,
                                            const char* filename,
                                            std::shared_ptr<ChMaterialShellANCF> my_material,
                                            std::vector<double>& node_ave_area,
                                            std::vector<int>& Boundary_nodes,
                                            ChVector<> pos_transform,
                                            ChMatrix33<> rot_transform,
                                            double scaleFactor,
                                            bool printNodes,
                                            bool printElements) {
    int added_nodes = 0;
    int added_elements = 0;
    double dx, dy;
    int nodes_offset = mesh->GetNnodes();
    printf("Current number of nodes in mesh is %d \n", nodes_offset);
    ChMatrixDynamic<double> nodesXYZ(1, 4);
    ChMatrixDynamic<int> NumBEdges(1, 3);  // To store boundary nodes
    ChMatrixNM<double, 1, 6> BoundingBox;  // (xmin xmax ymin ymax zmin zmax) bounding box of the mesh
    std::vector<ChVector<>> Normals;       // To store the normal vectors
    std::vector<int> num_Normals;
    ChVector<double> pos1, pos2, pos3, pos4;                  // Position of nodes in each element
    ChVector<double> vec1, vec2, vec3;                        // intermediate vectors for calculation of normals
    std::vector<std::shared_ptr<ChNodeFEAxyzD>> nodesVector;  // To store intermediate nodes
    std::vector<std::vector<int>> elementsVector;             // nodes of each element
    std::vector<std::vector<double>> elementsdxdy;            // dx, dy of elements

    int TotalNumNodes = 0, TotalNumElements = 0, TotalNumBEdges = 0;
    BoundingBox.setZero();

    std::ifstream fin(filename);
    if (!fin.good())
        throw ChException("ERROR opening Mesh file: " + std::string(filename) + "\n");

    std::string line;
    while (std::getline(fin, line)) {
        // trims white space from the beginning of the string
        line.erase(line.begin(),
                   std::find_if(line.begin(), line.end(), [](unsigned char c) { return !std::isspace(c); }));

        if (line[0] == 0)
            continue;  // skip empty linesnodes_offset
        if (line.find("Vertices") == 0) {
            std::getline(fin, line);
            TotalNumNodes = atoi(line.c_str());
            printf("Found  %d nodes\n", TotalNumNodes);
            GetLog() << "Parsing information from \"Vertices\" \n";
            std::getline(fin, line);
            Normals.resize(TotalNumNodes);
            node_ave_area.resize(nodes_offset + TotalNumNodes);
            num_Normals.resize(TotalNumNodes);
            for (int inode = 0; inode < TotalNumNodes; inode++) {
                double loc_x, loc_y, loc_z;
                double dir_x, dir_y, dir_z;

                int ntoken = 0;
                std::string token;
                std::istringstream ss(line);
                while (std::getline(ss, token, ' ') && ntoken < 20) {
                    std::istringstream stoken(token);
                    stoken >> nodesXYZ(0, ntoken);
                    nodesXYZ(0, ntoken) *= scaleFactor;
                    ++ntoken;
                }

                loc_x = nodesXYZ(0, 0);
                loc_y = nodesXYZ(0, 1);
                loc_z = nodesXYZ(0, 2);
                dir_x = 1.0;
                dir_y = 1.0;
                dir_z = 1.0;

                ChVector<> node_position(loc_x, loc_y, loc_z);
                node_position = rot_transform * node_position;  // rotate/scale, if needed
                node_position = pos_transform + node_position;  // move, if needed
                auto node = chrono_types::make_shared<ChNodeFEAxyzD>(node_position, ChVector<>(dir_x, dir_y, dir_z));
                nodesVector.push_back(node);

                if (loc_x < BoundingBox(0, 0) || added_nodes == 0)
                    BoundingBox(0, 0) = loc_x;
                if (loc_x > BoundingBox(0, 1) || added_nodes == 0)
                    BoundingBox(0, 1) = loc_x;
                if (loc_y < BoundingBox(0, 2) || added_nodes == 0)
                    BoundingBox(0, 2) = loc_y;
                if (loc_y > BoundingBox(0, 3) || added_nodes == 0)
                    BoundingBox(0, 3) = loc_y;

                if (loc_z < BoundingBox(0, 4) || added_nodes == 0)
                    BoundingBox(0, 4) = loc_z;
                if (loc_z > BoundingBox(0, 5) || added_nodes == 0)
                    BoundingBox(0, 5) = loc_z;
                ++added_nodes;

                if (ntoken != 4)
                    throw ChException("ERROR in .mesh file, Quadrilaterals require 4 node IDs, see line:\n" + line +
                                      "\n");

                std::getline(fin, line);
            }
        }

        // Reading the Boundary nodes ...
        if (line.find("Edges") == 0) {
            std::getline(fin, line);
            TotalNumBEdges = atoi(line.c_str());
            printf("Found %d Edges.\n", TotalNumBEdges);
            GetLog() << "Parsing edges from \"Edges\" \n";
            std::getline(fin, line);

            for (int edge = 0; edge < TotalNumBEdges; edge++) {
                int ntoken = 0;
                std::string token;
                std::istringstream ss(line);
                while (std::getline(ss, token, ' ') && ntoken < 20) {
                    std::istringstream stoken(token);
                    stoken >> NumBEdges(0, ntoken);
                    ++ntoken;
                }

                if (ntoken != 3)
                    throw ChException("ERROR in .mesh file, Edges require 3 node IDs, see line:\n" + line + "\n");

                std::getline(fin, line);
            }
        }

        //
        if (line.find("Quadrilaterals") == 0) {
            std::getline(fin, line);
            TotalNumElements = atoi(line.c_str());
            printf("Found %d elements.\n", TotalNumElements);
            GetLog() << "Parsing nodeset from \"Quadrilaterals\" \n";
            std::getline(fin, line);

            for (int ele = 0; ele < TotalNumElements; ele++) {
                int ntoken = 0;
                std::string token;
                std::istringstream ss(line);
                elementsVector.resize(ele + 1);
                elementsVector[ele].resize(5);
                elementsdxdy.resize(ele + 1);
                elementsdxdy[ele].resize(2);
                while (std::getline(ss, token, ' ') && ntoken < 20) {
                    std::istringstream stoken(token);
                    stoken >> elementsVector[ele][ntoken];
                    ++ntoken;
                }

                // Calculating the true surface normals based on the nodal information
                pos1 = nodesVector[elementsVector[ele][0] - 1]->GetPos();
                pos2 = nodesVector[elementsVector[ele][1] - 1]->GetPos();
                pos4 = nodesVector[elementsVector[ele][2] - 1]->GetPos();
                pos3 = nodesVector[elementsVector[ele][3] - 1]->GetPos();

                // For the first node
                vec1 = (pos1 - pos2);
                vec2 = (pos1 - pos3);
                Normals[elementsVector[ele][0] - 1] += vec1 % vec2;
                num_Normals[elementsVector[ele][0] - 1]++;
                // For the second node
                vec1 = (pos2 - pos4);
                vec2 = (pos2 - pos1);
                Normals[elementsVector[ele][1] - 1] += vec1 % vec2;
                num_Normals[elementsVector[ele][1] - 1]++;
                // For the third node
                vec1 = (pos3 - pos1);
                vec2 = (pos3 - pos4);
                Normals[elementsVector[ele][2] - 1] += vec1 % vec2;
                num_Normals[elementsVector[ele][2] - 1]++;
                // For the forth node
                vec1 = (pos4 - pos3);
                vec2 = (pos4 - pos2);
                Normals[elementsVector[ele][3] - 1] += vec1 % vec2;
                num_Normals[elementsVector[ele][3] - 1]++;

                vec1 = pos1 - pos2;
                vec2 = pos3 - pos4;
                dx = (vec1.Length() + vec2.Length()) / 2;
                vec1 = pos1 - pos3;
                vec2 = pos2 - pos4;
                dy = (vec1.Length() + vec2.Length()) / 2;

                // Set element dimensions
                elementsdxdy[ele][0] = dx;
                elementsdxdy[ele][1] = dy;
                ++added_elements;
                if (ntoken != 5)
                    throw ChException("ERROR in .mesh file, Quadrilaterals require 4 node IDs, see line:\n" + line +
                                      "\n");
                std::getline(fin, line);
            }
        }
    }

    printf("Mesh Bounding box is x [%f %f %f %f %f %f]\n", BoundingBox(0, 0), BoundingBox(0, 1), BoundingBox(0, 2),
           BoundingBox(0, 3), BoundingBox(0, 4), BoundingBox(0, 5));

    GetLog() << "-----------------------------------------------------------\n\n";
    //
    for (int inode = 0; inode < TotalNumNodes; inode++) {
        ChVector<> node_normal = (Normals[inode] / num_Normals[inode]);
        // Very useful information to store: 1/4 of area of neighbouring elements contribute to each node's average area
        node_ave_area[nodes_offset + inode] = Normals[inode].Length() / 4;

        if (num_Normals[inode] <= 2)
            Boundary_nodes.push_back(nodes_offset + inode);
        node_normal.Normalize();

        ChVector<> node_position = nodesVector[inode]->GetPos();
        auto node = chrono_types::make_shared<ChNodeFEAxyzD>(node_position, node_normal);
        node->SetMass(0);
        // Add node to mesh
        mesh->AddNode(node);
        if (printNodes) {
            GetLog() << node->GetPos().x() << "  " << node->GetPos().y() << "  " << node->GetPos().z() << "\n";
        }
    }
    GetLog() << "-----------------------------------------------------------\n";
    for (int ielem = 0; ielem < 0 + TotalNumElements; ielem++) {
        auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
        element->SetNodes(
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodes_offset + elementsVector[ielem][0] - 1)),
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodes_offset + elementsVector[ielem][1] - 1)),
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodes_offset + elementsVector[ielem][2] - 1)),
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodes_offset + elementsVector[ielem][3] - 1)));
        dx = elementsdxdy[ielem][0];
        dy = elementsdxdy[ielem][1];
        element->SetDimensions(dx, dy);
        // Add element to mesh
        mesh->AddElement(element);
        if (printElements) {
            std::cout << ielem << " ";
            for (int i = 0; i < 4; i++)
                std::cout << elementsVector[ielem][i] << " ";
            std::cout << std::endl;
        }
    }
}

void ChMeshFileLoader::BSTShellFromObjFile(
    std::shared_ptr<ChMesh> mesh,                           // destination mesh
    const char* filename,                                   // .obj mesh complete filename
    std::shared_ptr<ChMaterialShellKirchhoff> my_material,  // material to be given to the shell elements
    double my_thickness,                                    // thickness to be given to shell elements
    ChVector<> pos_transform,                               // optional displacement of imported mesh
    ChMatrix33<> rot_transform                              // optional rotation/scaling of imported mesh
) {
    auto mmesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(std::string(filename), false, false);
    const auto& v_indices = mmesh->m_face_v_indices;

    std::map<std::pair<int, int>, std::pair<int, int>> winged_edges;
    mmesh->ComputeWingedEdges(winged_edges);

    std::vector<std::shared_ptr<ChNodeFEAxyz>> shapenodes;
    for (size_t i = 0; i < mmesh->m_vertices.size(); i++) {
        ChVector<double> pos = mmesh->m_vertices[i];
        pos = rot_transform * pos;
        pos += pos_transform;
        auto mnode = chrono_types::make_shared<ChNodeFEAxyz>();
        mnode->SetPos(pos);
        mesh->AddNode(mnode);
        shapenodes.push_back(mnode);  // for future reference when adding faces
    }

    for (size_t j = 0; j < v_indices.size(); j++) {
        int i0 = v_indices[j][0];
        int i1 = v_indices[j][1];
        int i2 = v_indices[j][2];
        // GetLog() << "nodes 012 ids= " << i0 << " " << i1 << " " << i2 << " " << "\n";

        std::pair<int, int> medge0(i1, i2);
        std::pair<int, int> medge1(i2, i0);
        std::pair<int, int> medge2(i0, i1);
        if (medge0.first > medge0.second)
            medge0 = std::pair<int, int>(medge0.second, medge0.first);
        if (medge1.first > medge1.second)
            medge1 = std::pair<int, int>(medge1.second, medge1.first);
        if (medge0.first > medge0.second)
            medge2 = std::pair<int, int>(medge2.second, medge2.first);
        std::shared_ptr<ChNodeFEAxyz> node3 = nullptr;
        std::shared_ptr<ChNodeFEAxyz> node4 = nullptr;
        std::shared_ptr<ChNodeFEAxyz> node5 = nullptr;
        int itri = -1;
        int ivert = -1;
        if (winged_edges[medge0].second == j)
            itri = winged_edges[medge0].first;
        else
            itri = winged_edges[medge0].second;
        for (int vi = 0; vi < 3; ++vi) {
            if (v_indices[itri][vi] != medge0.first && v_indices[itri][vi] != medge0.second)
                ivert = v_indices[itri][vi];
        }
        if (ivert != -1)
            node3 = shapenodes[ivert];

        itri = -1;
        ivert = -1;
        if (winged_edges[medge1].second == j)
            itri = winged_edges[medge1].first;
        else
            itri = winged_edges[medge1].second;
        for (int vi = 0; vi < 3; ++vi) {
            if (v_indices[itri][vi] != medge1.first && v_indices[itri][vi] != medge1.second)
                ivert = v_indices[itri][vi];
        }
        if (ivert != -1)
            node4 = shapenodes[ivert];

        itri = -1;
        ivert = -1;
        if (winged_edges[medge2].second == j)
            itri = winged_edges[medge2].first;
        else
            itri = winged_edges[medge2].second;
        for (int vi = 0; vi < 3; ++vi) {
            if (v_indices[itri][vi] != medge2.first && v_indices[itri][vi] != medge2.second)
                ivert = v_indices[itri][vi];
        }
        if (ivert != -1)
            node5 = shapenodes[ivert];

        auto melement = chrono_types::make_shared<ChElementShellBST>();
        melement->SetNodes(shapenodes[i0], shapenodes[i1], shapenodes[i2], node3, node4, node5);
        mesh->AddElement(melement);
        melement->AddLayer(my_thickness, 0, my_material);
    }
}

}  // end namespace fea
}  // end namespace chrono
