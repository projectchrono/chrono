

#include "core/ChMath.h"
#include "physics/ChObject.h"
#include "physics/ChLoad.h"
#include "physics/ChSystem.h"
#include "ChMeshImport.h"
#include "../ChMesh.h"
#include "chrono_fea/ChNodeFEAxyz.h"
#include "chrono_fea/ChElementShellANCF.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
#include <functional>

using namespace std;

namespace chrono {
namespace fea {

/// Load tetahedrons, if any, saved in a .inp file for Abaqus.

void ChMesh::LoadANCFShellFromGMFFile(const char* filename,
                                      ChSharedPtr<ChMaterialShellANCF> my_material,
                                      std::vector<int>& BC_nodes,
                                      ChVector<> pos_transform,
                                      ChMatrix33<> rot_transform,
                                      double scaleFactor,
                                      bool printNodes,
                                      bool printBC,
                                      bool printElements) {
    int added_nodes = 0;
    int added_elements = 0;
    double dx, dy;
    int nodes_offset = this->GetNnodes();
    ChMatrixDynamic<double> nodesXYZ(1, 4);
    ChMatrixDynamic<int> NumBEdges(1, 3);  // To store boundary nodes
    ChMatrixNM<double, 1, 6> BoundingBox;  // (xmin xmax ymin ymax zmin zmax) bounding box of the mesh
    std::vector<ChVector<>> Normals;       // To store the normal vectors
    std::vector<int> num_Normals;
    ChVector<double> pos1, pos2, pos3, pos4;              // Position of nodes in each element
    ChVector<double> vec1, vec2, vec3;                    // intermediate vectors for calculation of normals
    std::vector<ChSharedPtr<ChNodeFEAxyzD>> nodesVector;  // To store intermediate nodes
    std::vector<std::vector<int>> elementsVector;         // nodes of each element
    std::vector<std::vector<double>> elementsdxdy;        // dx, dy of elements

    int TotalNumNodes, TotalNumElements, TottalNumBEdges;
    BoundingBox.FillElem(0);

    std::fstream fin(filename);
    if (!fin.good())
        throw ChException("ERROR opening Mesh file: " + std::string(filename) + "\n");

    std::string line;
    while (getline(fin, line)) {
        // trims white space from the beginning of the string
        line.erase(line.begin(), find_if(line.begin(), line.end(), not1(ptr_fun<int, int>(isspace))));

        if (line[0] == 0)
            continue;  // skip empty linesnodes_offset
        if (line.find("Vertices") == 0) {
            getline(fin, line);
            TotalNumNodes = atoi(line.c_str());
            printf("Found  %d nodes\n", TotalNumNodes);
            GetLog() << "Parsing information from \"Vertices\" \n";
            cout << "Reading nodal information ..." << endl;
            getline(fin, line);
            Normals.resize(TotalNumNodes);
            num_Normals.resize(TotalNumNodes);
            for (int inode = 0; inode < TotalNumNodes; inode++) {
                double loc_x, loc_y, loc_z;
                double dir_x, dir_y, dir_z;
                unsigned int tokenvals[20];

                int ntoken = 0;
                string token;
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
                ChSharedPtr<ChNodeFEAxyzD> node(new ChNodeFEAxyzD(node_position, ChVector<>(dir_x, dir_y, dir_z)));
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

                getline(fin, line);
            }
        }

        // Reading the Boundary nodes ...
        if (line.find("Edges") == 0) {
            getline(fin, line);
            TottalNumBEdges = atoi(line.c_str());
            printf("Found %d Edges.\n", TottalNumBEdges);
            GetLog() << "Parsing edges from \"Edges\" \n";
            getline(fin, line);

            for (int edge = 0; edge < TottalNumBEdges; edge++) {
                unsigned int tokenvals[20];
                int ntoken = 0;
                string token;
                std::istringstream ss(line);
                while (std::getline(ss, token, ' ') && ntoken < 20) {
                    std::istringstream stoken(token);
                    stoken >> NumBEdges(0, ntoken);
                    ++ntoken;
                }

                BC_nodes.push_back(NumBEdges(0, 0) - 1);

                if (ntoken != 3)
                    throw ChException("ERROR in .mesh file, Edges require 3 node IDs, see line:\n" + line + "\n");
                if (printBC) {
                    cout << edge << " ";
                    for (int i = 0; i < 2; i++)
                        cout << NumBEdges(0, i) << " ";
                    cout << endl;
                }
                getline(fin, line);
            }
        }

        //
        /////////////////
        if (line.find("Quadrilaterals") == 0) {
            getline(fin, line);
            TotalNumElements = atoi(line.c_str());
            printf("Found %d elements.\n", TotalNumElements);
            GetLog() << "Parsing nodeset from \"Quadrilaterals\" \n";
            getline(fin, line);
            cout << "Reading elemental information ..." << endl;

            for (int ele = 0; ele < TotalNumElements; ele++) {
                unsigned int tokenvals[20];
                int ntoken = 0;
                string token;
                std::istringstream ss(line);
                elementsVector.resize(ele + 1);
                elementsVector[ele].resize(4);
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
                getline(fin, line);
            }
        }
    }

    printf("Mesh Bounding box is x [%f %f %f %f %f %f]\n", BoundingBox(0, 0), BoundingBox(0, 1), BoundingBox(0, 2),
           BoundingBox(0, 3), BoundingBox(0, 4), BoundingBox(0, 5));

    GetLog() << "-----------------------------------------------------------\n\n";
    //
    for (int inode = 0; inode < 0 + TotalNumNodes; inode++) {
        ChVector<> node_normal = (Normals[inode] / num_Normals[inode]);
        node_normal.Normalize();
        ChVector<> node_position = nodesVector[inode]->GetPos();
        ChSharedPtr<ChNodeFEAxyzD> node(new ChNodeFEAxyzD(node_position, node_normal));
        node->SetMass(0);
        // Add node to mesh
        this->AddNode(node);
        if (printNodes) {
            GetLog() << node->GetPos().x << "  " << node->GetPos().y << "  " << node->GetPos().z << "\n";
        }
    }
    GetLog() << "-----------------------------------------------------------\n";
    for (int ielem = 0; ielem < 0 + TotalNumElements; ielem++) {
        ChSharedPtr<ChElementShellANCF> element(new ChElementShellANCF);
        element->SetNodes(GetNode(nodes_offset + elementsVector[ielem][0] - 1).DynamicCastTo<ChNodeFEAxyzD>(),
                          GetNode(nodes_offset + elementsVector[ielem][1] - 1).DynamicCastTo<ChNodeFEAxyzD>(),
                          GetNode(nodes_offset + elementsVector[ielem][3] - 1).DynamicCastTo<ChNodeFEAxyzD>(),
                          GetNode(nodes_offset + elementsVector[ielem][2] - 1).DynamicCastTo<ChNodeFEAxyzD>());
        dx = elementsdxdy[ielem][0];
        dy = elementsdxdy[ielem][1];
        element->SetDimensions(dx, dy);
        // Add element to mesh
        this->AddElement(element);
        if (printElements) {
            cout << ielem << " ";
            for (int i = 0; i < 4; i++)
                cout << elementsVector[ielem][i] << " ";
            cout << endl;
        }
    }
}
}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____
