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

void ChMeshExporter::WriteMesh(std::shared_ptr<ChMesh> mesh, const std::string& mesh_filename) {
    std::ofstream out_stream;
    out_stream.open(mesh_filename, std::ios::out);
    out_stream.precision(7);
    out_stream << std::scientific;

    std::vector<std::vector<int>> CableElemNodes;
    std::vector<std::vector<int>> ShellElemNodes;
    std::vector<std::vector<int>> BrickElemNodes;

    std::vector<std::shared_ptr<ChNodeFEAbase>> myvector;
    myvector.resize(mesh->GetNnodes());

    for (unsigned int i = 0; i < mesh->GetNnodes(); i++) {
        myvector[i] = std::dynamic_pointer_cast<ChNodeFEAbase>(mesh->GetNode(i));
    }

    int numCables = 0;
    int numShells = 0;
    int numBricks = 0;
    for (unsigned int iele = 0; iele < mesh->GetNelements(); iele++) {
        if (std::dynamic_pointer_cast<ChElementCableANCF>(mesh->GetElement(iele)))
            numCables++;
        if (std::dynamic_pointer_cast<ChElementShellANCF_3423>(mesh->GetElement(iele)))
            numShells++;
        if (std::dynamic_pointer_cast<ChElementHexaANCF_3813>(mesh->GetElement(iele)))
            numBricks++;
    }
    out_stream << "\nCELLS " << mesh->GetNelements() << " "
               << (unsigned int)(numCables * 3 + numShells * 5 + numBricks * 9) << "\n";

    for (unsigned int iele = 0; iele < mesh->GetNelements(); iele++) {
        std::vector<int> mynodes;

        if (auto elementC = std::dynamic_pointer_cast<ChElementCableANCF>(mesh->GetElement(iele))) {
            mynodes.resize(2);
            out_stream << "2 ";
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
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
        } else if (auto elementS = std::dynamic_pointer_cast<ChElementShellANCF_3423>(mesh->GetElement(iele))) {
            mynodes.resize(4);
            out_stream << "4 ";
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
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
        } else if (auto elementB = std::dynamic_pointer_cast<ChElementHexaANCF_3813>(mesh->GetElement(iele))) {
            mynodes.resize(8);
            out_stream << "8 ";
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
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
        }
    }

    out_stream << "\nCELL_TYPES " << mesh->GetNelements() << "\n";

    for (unsigned int iele = 0; iele < mesh->GetNelements(); iele++) {
        if (std::dynamic_pointer_cast<ChElementCableANCF>(mesh->GetElement(iele)))
            out_stream << "3\n";
        else if (std::dynamic_pointer_cast<ChElementShellANCF_3423>(mesh->GetElement(iele)))
            out_stream << "9\n";
        else if (std::dynamic_pointer_cast<ChElementHexaANCF_3813>(mesh->GetElement(iele)))
            out_stream << "12\n";
    }

    out_stream.close();
}

void ChMeshExporter::WriteFrame(std::shared_ptr<ChMesh> mesh,
                                const std::string& mesh_filename,
                                const std::string& vtk_filename) {
    std::ofstream out_stream;
    out_stream.open(vtk_filename, std::ios::trunc);

    out_stream << "# vtk DataFile Version 2.0" << std::endl;
    out_stream << "Unstructured Grid Example" << std::endl;
    out_stream << "ASCII" << std::endl;
    out_stream << "DATASET UNSTRUCTURED_GRID" << std::endl;

    out_stream << "POINTS " << mesh->GetNnodes() << " float\n";
    for (unsigned int i = 0; i < mesh->GetNnodes(); i++) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i));
        out_stream << node->GetPos().x() << " " << node->GetPos().y() << " " << node->GetPos().z() << "\n";
    }

    std::ifstream in_stream(mesh_filename);
    out_stream << in_stream.rdbuf();

    int numCell = 0;
    for (unsigned int iele = 0; iele < mesh->GetNelements(); iele++) {
        if (std::dynamic_pointer_cast<ChElementCableANCF>(mesh->GetElement(iele)))
            numCell++;
        else if (std::dynamic_pointer_cast<ChElementShellANCF_3423>(mesh->GetElement(iele)))
            numCell++;
    }

    out_stream << "\nCELL_DATA " << numCell << "\n";
    out_stream << "SCALARS Deflection float 1\n";
    out_stream << "LOOKUP_TABLE default\n";

    double scalar = 0;
    for (unsigned int iele = 0; iele < mesh->GetNelements(); iele++) {
        if (auto elementC = std::dynamic_pointer_cast<ChElementCableANCF>(mesh->GetElement(iele)))
            scalar = elementC->GetCurrLength() - elementC->GetRestLength();
        else if (auto elementS = std::dynamic_pointer_cast<ChElementShellANCF_3423>(mesh->GetElement(iele)))
            elementS->EvaluateDeflection(scalar);
        out_stream << scalar + 1e-20 << "\n";
    }

    out_stream << "VECTORS Strain float\n";
    ChVector<> StrainV;
    for (unsigned int iele = 0; iele < mesh->GetNelements(); iele++) {
        if (auto elementC = std::dynamic_pointer_cast<ChElementCableANCF>(mesh->GetElement(iele)))
            elementC->EvaluateSectionStrain(0.0, StrainV);
        else if (auto elementS = std::dynamic_pointer_cast<ChElementShellANCF_3423>(mesh->GetElement(iele))) {
            const ChStrainStress3D strainStressOut =
                elementS->EvaluateSectionStrainStress(ChVector<double>(0, 0, 0), 0);
            StrainV.Set(strainStressOut.strain[0], strainStressOut.strain[1], strainStressOut.strain[3]);
        }
        StrainV += ChVector<>(1e-20);
        out_stream << StrainV.x() << " " << StrainV.y() << " " << StrainV.z() << "\n";
    }

    out_stream << "\nPOINT_DATA " << mesh->GetNnodes() << "\n";

    out_stream << "VECTORS Velocity float\n";
    for (unsigned int i = 0; i < mesh->GetNnodes(); i++) {
        ChVector<> vel = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))->GetPos_dt();
        vel += ChVector<>(1e-20);
        out_stream << (double)vel.x() << " " << (double)vel.y() << " " << (double)vel.z() << "\n";
    }

    out_stream << "VECTORS Acceleration float\n";

    for (unsigned int i = 0; i < mesh->GetNnodes(); i++) {
        ChVector<> acc = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(i))->GetPos_dtdt();
        acc += ChVector<>(1e-20);
        out_stream << (double)acc.x() << " " << (double)acc.y() << " " << (double)acc.z() << "\n";
    }

    out_stream.close();
}

}  // namespace fea
}  // namespace chrono
