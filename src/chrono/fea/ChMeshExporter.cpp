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
// Authors: Milad Rakhsha
// =============================================================================
#include "chrono/fea/ChMeshExporter.h"

namespace chrono {
namespace fea {

void ChMeshExporter::writeMesh(std::shared_ptr<ChMesh> my_mesh, std::string SaveAs) {
    std::ofstream MESH;  // output file stream
    MESH.open(SaveAs, std::ios::out);
    MESH.precision(7);
    MESH << std::scientific;
    std::vector<std::vector<int>> CableElemNodes;
    std::vector<std::vector<int>> ShellElemNodes;
    std::vector<std::vector<int>> BrickElemNodes;

    std::vector<std::shared_ptr<ChNodeFEAbase>> myvector;
    myvector.resize(my_mesh->GetNnodes());

    for (unsigned int i = 0; i < my_mesh->GetNnodes(); i++) {
        myvector[i] = std::dynamic_pointer_cast<ChNodeFEAbase>(my_mesh->GetNode(i));
    }

    int numCables = 0;
    int numShells = 0;
    int numBricks = 0;
    for (unsigned int iele = 0; iele < my_mesh->GetNelements(); iele++) {
        if (std::dynamic_pointer_cast<ChElementCableANCF>(my_mesh->GetElement(iele)))
            numCables++;
        if (std::dynamic_pointer_cast<ChElementShellANCF_3423>(my_mesh->GetElement(iele)))
            numShells++;
        if (std::dynamic_pointer_cast<ChElementHexaANCF_3813>(my_mesh->GetElement(iele)))
            numBricks++;
    }
    MESH << "\nCELLS " << my_mesh->GetNelements() << " "
         << (unsigned int)(numCables * 3 + numShells * 5 + numBricks * 9) << "\n";

    for (unsigned int iele = 0; iele < my_mesh->GetNelements(); iele++) {
        std::vector<int> mynodes;

        if (auto elementC = std::dynamic_pointer_cast<ChElementCableANCF>(my_mesh->GetElement(iele))) {
            mynodes.resize(2);
            MESH << "2 ";
            int nodeOrder[] = {0, 1};
            mynodes[0] = elementC->GetNodeN(nodeOrder[0])->GetIndex();
            mynodes[1] = elementC->GetNodeN(nodeOrder[1])->GetIndex();
            CableElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++) {
                auto nodeA = (elementC->GetNodeN(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myvector.begin(), myvector.end(), nodeA);
                if (it == myvector.end()) {
                    // name not in vector
                } else {
                    auto index = std::distance(myvector.begin(), it);
                    MESH << (unsigned int)index << " ";
                }
            }
            MESH << "\n";
        } else if (auto elementS = std::dynamic_pointer_cast<ChElementShellANCF_3423>(my_mesh->GetElement(iele))) {
            mynodes.resize(4);
            MESH << "4 ";
            int nodeOrder[] = {0, 1, 2, 3};
            mynodes[0] = elementS->GetNodeN(nodeOrder[0])->GetIndex();
            mynodes[1] = elementS->GetNodeN(nodeOrder[1])->GetIndex();
            mynodes[2] = elementS->GetNodeN(nodeOrder[2])->GetIndex();
            mynodes[3] = elementS->GetNodeN(nodeOrder[3])->GetIndex();
            ShellElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++) {
                auto nodeA = (elementS->GetNodeN(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myvector.begin(), myvector.end(), nodeA);
                if (it == myvector.end()) {
                    // name not in vector
                } else {
                    auto index = std::distance(myvector.begin(), it);
                    MESH << (unsigned int)index << " ";
                }
            }
            MESH << "\n";
        } else if (auto elementB = std::dynamic_pointer_cast<ChElementHexaANCF_3813>(my_mesh->GetElement(iele))) {
            mynodes.resize(8);
            MESH << "8 ";
            int nodeOrder[] = {0, 1, 2, 3, 4, 5, 6, 7};
            mynodes[0] = elementB->GetNodeN(nodeOrder[0])->GetIndex();
            mynodes[1] = elementB->GetNodeN(nodeOrder[1])->GetIndex();
            mynodes[2] = elementB->GetNodeN(nodeOrder[2])->GetIndex();
            mynodes[3] = elementB->GetNodeN(nodeOrder[3])->GetIndex();
            mynodes[4] = elementB->GetNodeN(nodeOrder[4])->GetIndex();
            mynodes[5] = elementB->GetNodeN(nodeOrder[5])->GetIndex();
            mynodes[6] = elementB->GetNodeN(nodeOrder[6])->GetIndex();
            mynodes[7] = elementB->GetNodeN(nodeOrder[7])->GetIndex();
            BrickElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++) {
                auto nodeA = (elementB->GetNodeN(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myvector.begin(), myvector.end(), nodeA);
                if (it == myvector.end()) {
                    // name not in vector
                } else {
                    auto index = std::distance(myvector.begin(), it);
                    MESH << (unsigned int)index << " ";
                }
            }
            MESH << "\n";
        }
    }

    MESH << "\nCELL_TYPES " << my_mesh->GetNelements() << "\n";

    for (unsigned int iele = 0; iele < my_mesh->GetNelements(); iele++) {
        if (std::dynamic_pointer_cast<ChElementCableANCF>(my_mesh->GetElement(iele)))
            MESH << "3\n";
        else if (std::dynamic_pointer_cast<ChElementShellANCF_3423>(my_mesh->GetElement(iele)))
            MESH << "9\n";
        else if (std::dynamic_pointer_cast<ChElementHexaANCF_3813>(my_mesh->GetElement(iele)))
            MESH << "12\n";
    }

    MESH.close();
    // MESH.write_to_file(SaveAs);
}

void ChMeshExporter::writeFrame(std::shared_ptr<ChMesh> my_mesh, char SaveAsBuffer[256], std::string MeshFileBuffer) {
    std::ofstream output;
    std::string SaveAsBuffer_string(SaveAsBuffer);
    SaveAsBuffer_string.erase(SaveAsBuffer_string.length() - 4, 4);
    std::cout << SaveAsBuffer_string << std::endl;
    snprintf(SaveAsBuffer, sizeof(char) * 256, ("%s"), (SaveAsBuffer_string + ".vtk").c_str());
    output.open(SaveAsBuffer, std::ios::trunc);

    output << "# vtk DataFile Version 2.0" << std::endl;
    output << "Unstructured Grid Example" << std::endl;
    output << "ASCII" << std::endl;
    output << "DATASET UNSTRUCTURED_GRID" << std::endl;

    output << "POINTS " << my_mesh->GetNnodes() << " float\n";
    for (unsigned int i = 0; i < my_mesh->GetNnodes(); i++) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(i));
        output << node->GetPos().x() << " " << node->GetPos().y() << " " << node->GetPos().z() << "\n";
    }

    std::ifstream CopyFrom(MeshFileBuffer);
    output << CopyFrom.rdbuf();

    int numCell = 0;
    for (unsigned int iele = 0; iele < my_mesh->GetNelements(); iele++) {
        if (std::dynamic_pointer_cast<ChElementCableANCF>(my_mesh->GetElement(iele)))
            numCell++;
        else if (std::dynamic_pointer_cast<ChElementShellANCF_3423>(my_mesh->GetElement(iele)))
            numCell++;
    }

    output << "\nCELL_DATA " << numCell << "\n";
    output << "SCALARS Deflection float 1\n";
    output << "LOOKUP_TABLE default\n";

    double scalar = 0;
    for (unsigned int iele = 0; iele < my_mesh->GetNelements(); iele++) {
        if (auto elementC = std::dynamic_pointer_cast<ChElementCableANCF>(my_mesh->GetElement(iele)))
            scalar = elementC->GetCurrLength() - elementC->GetRestLength();
        else if (auto elementS = std::dynamic_pointer_cast<ChElementShellANCF_3423>(my_mesh->GetElement(iele)))
            elementS->EvaluateDeflection(scalar);
        output << scalar + 1e-20 << "\n";
    }

    output << "VECTORS Strain float\n";
    ChVector<> StrainV;
    for (unsigned int iele = 0; iele < my_mesh->GetNelements(); iele++) {
        if (auto elementC = std::dynamic_pointer_cast<ChElementCableANCF>(my_mesh->GetElement(iele)))
            elementC->EvaluateSectionStrain(0.0, StrainV);
        else if (auto elementS = std::dynamic_pointer_cast<ChElementShellANCF_3423>(my_mesh->GetElement(iele))) {
            const ChStrainStress3D strainStressOut =
                elementS->EvaluateSectionStrainStress(ChVector<double>(0, 0, 0), 0);
            StrainV.Set(strainStressOut.strain[0], strainStressOut.strain[1], strainStressOut.strain[3]);
        }
        StrainV += ChVector<>(1e-20);
        output << StrainV.x() << " " << StrainV.y() << " " << StrainV.z() << "\n";
    }

    output << "\nPOINT_DATA " << my_mesh->GetNnodes() << "\n";

    output << "VECTORS Velocity float\n";
    for (unsigned int i = 0; i < my_mesh->GetNnodes(); i++) {
        ChVector<> vel = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(i))->GetPos_dt();
        vel += ChVector<>(1e-20);
        output << (double)vel.x() << " " << (double)vel.y() << " " << (double)vel.z() << "\n";
    }

    output << "VECTORS Acceleration float\n";

    for (unsigned int i = 0; i < my_mesh->GetNnodes(); i++) {
        ChVector<> acc = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(i))->GetPos_dtdt();
        acc += ChVector<>(1e-20);
        output << (double)acc.x() << " " << (double)acc.y() << " " << (double)acc.z() << "\n";
    }

    output.close();
}  // namespace fea

}  // namespace fea
}  // namespace chrono
