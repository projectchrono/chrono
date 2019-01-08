#ifndef CHMESHEXPORTER_H_
#define CHMESHEXPORTER_H_
#include "chrono/fea/ChElementBrick.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementShellANCF.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyz.h"
namespace chrono {
namespace fea {
/// Collection of mesh file loader utilities.
class ChApi ChMeshExporter {
  public:
    static void writeMesh(std::shared_ptr<ChMesh> my_mesh,
                          std::string SaveAs,
                          std::vector<std::vector<int>>& NodeNeighborElement);

    static void writeFrame(std::shared_ptr<ChMesh> my_mesh,
                           char SaveAsBuffer[256],
                           std::string MeshFileBuffer,
                           std::vector<std::vector<int>> NodeNeighborElement);
};

}  // namespace fea
}  // namespace chrono

#endif
