#include "chrono/fea/ChMeshExporter.h"

namespace chrono {
namespace fea {

void ChMeshExporter::writeMesh(std::shared_ptr<ChMesh> my_mesh,
                               std::string SaveAs,
                               std::vector<std::vector<int>>& NodeNeighborElement) {
    std::ofstream MESH;  // output file stream
    MESH.open(SaveAs, std::ios::out);
    MESH.precision(7);
    MESH << std::scientific;
    NodeNeighborElement.resize(my_mesh->GetNnodes());
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
        if (auto element = std::dynamic_pointer_cast<ChElementCableANCF>(my_mesh->GetElement(iele)))
            numCables++;
        if (auto element = std::dynamic_pointer_cast<ChElementShellANCF>(my_mesh->GetElement(iele)))
            numShells++;
        if (auto element = std::dynamic_pointer_cast<ChElementBrick>(my_mesh->GetElement(iele)))
            numBricks++;
    }
    MESH << "\nCELLS " << my_mesh->GetNelements() << " "
         << (unsigned int)(numCables * 3 + numShells * 5 + numBricks * 9) << "\n";

    for (unsigned int iele = 0; iele < my_mesh->GetNelements(); iele++) {
        std::vector<int> mynodes;

        if (auto element = std::dynamic_pointer_cast<ChElementCableANCF>(my_mesh->GetElement(iele))) {
            mynodes.resize(2);
            MESH << "2 ";
            int nodeOrder[] = {0, 1};
            mynodes[0] = element->GetNodeN(nodeOrder[0])->GetIndex();
            mynodes[1] = element->GetNodeN(nodeOrder[1])->GetIndex();
            CableElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++) {
                auto nodeA = (element->GetNodeN(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myvector.begin(), myvector.end(), nodeA);
                if (it == myvector.end()) {
                    // name not in vector
                } else {
                    auto index = std::distance(myvector.begin(), it);
                    MESH << (unsigned int)index << " ";
                    NodeNeighborElement[index].push_back(iele);
                }
            }
            MESH << "\n";
        } else if (auto element = std::dynamic_pointer_cast<ChElementShellANCF>(my_mesh->GetElement(iele))) {
            mynodes.resize(4);
            MESH << "4 ";
            int nodeOrder[] = {0, 1, 2, 3};
            mynodes[0] = element->GetNodeN(nodeOrder[0])->GetIndex();
            mynodes[1] = element->GetNodeN(nodeOrder[1])->GetIndex();
            mynodes[2] = element->GetNodeN(nodeOrder[2])->GetIndex();
            mynodes[3] = element->GetNodeN(nodeOrder[3])->GetIndex();
            ShellElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++) {
                auto nodeA = (element->GetNodeN(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myvector.begin(), myvector.end(), nodeA);
                if (it == myvector.end()) {
                    // name not in vector
                } else {
                    auto index = std::distance(myvector.begin(), it);
                    MESH << (unsigned int)index << " ";
                    NodeNeighborElement[index].push_back(iele);
                }
            }
            MESH << "\n";
        } else if (auto element = std::dynamic_pointer_cast<ChElementBrick>(my_mesh->GetElement(iele))) {
            mynodes.resize(8);
            MESH << "8 ";
            int nodeOrder[] = {0, 1, 2, 3, 4, 5, 6, 7};
            mynodes[0] = element->GetNodeN(nodeOrder[0])->GetIndex();
            mynodes[1] = element->GetNodeN(nodeOrder[1])->GetIndex();
            mynodes[2] = element->GetNodeN(nodeOrder[2])->GetIndex();
            mynodes[3] = element->GetNodeN(nodeOrder[3])->GetIndex();
            mynodes[4] = element->GetNodeN(nodeOrder[4])->GetIndex();
            mynodes[5] = element->GetNodeN(nodeOrder[5])->GetIndex();
            mynodes[6] = element->GetNodeN(nodeOrder[6])->GetIndex();
            mynodes[7] = element->GetNodeN(nodeOrder[7])->GetIndex();
            BrickElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++) {
                auto nodeA = (element->GetNodeN(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myvector.begin(), myvector.end(), nodeA);
                if (it == myvector.end()) {
                    // name not in vector
                } else {
                    auto index = std::distance(myvector.begin(), it);
                    MESH << (unsigned int)index << " ";
                    NodeNeighborElement[index].push_back(iele);
                }
            }
            MESH << "\n";
        }
    }

    MESH << "\nCELL_TYPES " << my_mesh->GetNelements() << "\n";

    for (unsigned int iele = 0; iele < my_mesh->GetNelements(); iele++) {
        if (auto element = std::dynamic_pointer_cast<ChElementCableANCF>(my_mesh->GetElement(iele)))
            MESH << "3\n";
        else if (auto element = std::dynamic_pointer_cast<ChElementShellANCF>(my_mesh->GetElement(iele)))
            MESH << "9\n";
        else if (auto element = std::dynamic_pointer_cast<ChElementBrick>(my_mesh->GetElement(iele)))
            MESH << "12\n";
    }

    // MESH.close();
    // MESH.write_to_file(SaveAs);
}

void ChMeshExporter::writeFrame(std::shared_ptr<ChMesh> my_mesh,
                                char SaveAsBuffer[256],
                                std::string MeshFileBuffer,
                                std::vector<std::vector<int>> NodeNeighborElement) {
    std::ofstream output;
    std::string SaveAsBuffer_string(SaveAsBuffer);
    SaveAsBuffer_string.erase(SaveAsBuffer_string.length() - 4, 4);
    std::cout << SaveAsBuffer_string << std::endl;
    snprintf(SaveAsBuffer, sizeof(char) * 256, ("%s"), (SaveAsBuffer_string + ".vtk").c_str());
    output.open(SaveAsBuffer, std::ios::app);

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
        if (auto element = std::dynamic_pointer_cast<ChElementCableANCF>(my_mesh->GetElement(iele)))
            numCell++;
        else if (auto element = std::dynamic_pointer_cast<ChElementShellANCF>(my_mesh->GetElement(iele)))
            numCell++;
    }

    output << "\nCELL_DATA " << numCell << "\n";
    output << "SCALARS Deflection float 1\n";
    output << "LOOKUP_TABLE default\n";

    double scalar = 0;
    for (unsigned int iele = 0; iele < my_mesh->GetNelements(); iele++) {
        if (auto element = std::dynamic_pointer_cast<ChElementCableANCF>(my_mesh->GetElement(iele)))
            scalar = element->GetCurrLength() - element->GetRestLength();
        else if (auto element = std::dynamic_pointer_cast<ChElementShellANCF>(my_mesh->GetElement(iele)))
            element->EvaluateDeflection(scalar);
        output << scalar + 1e-20 << "\n";
    }

    output << "VECTORS Strain float\n";
    ChVector<> StrainV;
    for (unsigned int iele = 0; iele < my_mesh->GetNelements(); iele++) {
        if (auto element = std::dynamic_pointer_cast<ChElementCableANCF>(my_mesh->GetElement(iele)))
            element->EvaluateSectionStrain(0.0, StrainV);
        else if (auto element = std::dynamic_pointer_cast<ChElementShellANCF>(my_mesh->GetElement(iele)))
            StrainV = element->EvaluateSectionStrains();

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

    //    output << "VECTORS Strain float\n";
    //    for (int i = 0; i < my_mesh->GetNnodes(); i++) {
    //        ChVector<> StrainV;
    //        ChVector<> areaAve(0, 0, 0);
    //        double myarea = 0;
    //        for (int j = 0; j < NodeNeighborElement[i].size(); j++) {
    //            int iele = NodeNeighborElement[i][j];
    //            /// ChElementCableANCF
    //            if (auto element = std::dynamic_pointer_cast<ChElementCableANCF>(my_mesh->GetElement(iele))) {
    //                element->EvaluateSectionStrain(0.0, StrainV);
    //                double dx =
    //                std::dynamic_pointer_cast<ChElementCableANCF>(my_mesh->GetElement(iele))->GetCurrLength(); areaAve
    //                += StrainV * dx; myarea += dx;
    //                /// ChElementShellANCF
    //            } else if (auto element = std::dynamic_pointer_cast<ChElementShellANCF>(my_mesh->GetElement(iele))) {
    //                StrainV = element->EvaluateSectionStrains();
    //                double dx =
    //                std::dynamic_pointer_cast<fea::ChElementShellANCF>(my_mesh->GetElement(iele))->GetLengthX();
    //                double dy =
    //                std::dynamic_pointer_cast<fea::ChElementShellANCF>(my_mesh->GetElement(iele))->GetLengthY();
    //                areaAve += StrainV * dx * dy;
    //                myarea += dx * dy;
    //            }
    //        }
    //        StrainV /= myarea;
    //        areaAve += ChVector<>(1e-20);
    //        output << (double)areaAve.x() << " " << (double)areaAve.y() << " " << (double)areaAve.z() << "\n";
    //    }
    //
    //    output << "SCALARS Point_Deflection float 1\n";
    //    for (int i = 0; i < my_mesh->GetNnodes(); i++) {
    //        double areaAve = 0;
    //        double scalar = 0;
    //        double myarea = 0;
    //        for (int j = 0; j < NodeNeighborElement[i].size(); j++) {
    //            int jele = NodeNeighborElement[i][j];
    //            /// ChElementCableANCF
    //            if (auto element = std::dynamic_pointer_cast<ChElementCableANCF>(my_mesh->GetElement(jele))) {
    //                scalar = element->GetCurrLength() - element->GetRestLength();
    //                double dx = element->GetCurrLength();
    //                myarea += dx;
    //                areaAve += scalar * dx;
    //
    //                /// ChElementShellANCF
    //            } else if (auto element = std::dynamic_pointer_cast<ChElementShellANCF>(my_mesh->GetElement(jele))) {
    //                element->EvaluateDeflection(scalar);
    //                double dx = element->GetLengthX();
    //                double dy = element->GetLengthY();
    //                myarea += dx * dy;
    //                areaAve += scalar * dx * dy;
    //            }
    //        }
    //
    //        areaAve /= myarea;
    //        areaAve += 1e-20;
    //        output << (double)areaAve << "\n";
    //    }

    output.close();
}  // namespace fea

}  // namespace fea
}  // namespace chrono
