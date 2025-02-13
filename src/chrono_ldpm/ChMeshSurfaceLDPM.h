#ifndef CHMESHSURFACELDPM_H
#define CHMESHSURFACELDPM_H

#include "chrono_ldpm/ChLdpmApi.h"
#include "chrono/physics/ChLoadable.h"
#include "chrono/fea/ChElementBase.h"

#include "chrono/fea/ChMeshSurface.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChTetrahedronFace.h"
#include "chrono/fea/ChHexahedronFace.h"

using namespace chrono::fea;


namespace chrono {
namespace ldpm {

/// @addtogroup chrono_fea
/// @{

// Forward references (for parent hierarchy pointer)
//class ChMesh;

class ChLdpmApi ChMeshSurfaceLDPM : public ChMeshSurface {
public:
    // Constructor
    ChMeshSurfaceLDPM(ChMesh* parentmesh = nullptr) : ChMeshSurface(parentmesh) {}

    // Destructor
    virtual ~ChMeshSurfaceLDPM() {}
	
	
    /// Get owner mesh.
    ChMesh* GetMesh() { return ChMeshSurface::GetMesh(); }

    /// Set owner mesh.
    void SetMesh(ChMesh* mm) { ChMeshSurface::SetMesh(mm); }
    
    
    // Additional functionality can be added here
    void AdvancedFunctionality() {
        // Implementation of additional functionality
    }
    
    
    /// Add multiple faces of FEM elements given a set of nodes at vertexes.
    /// Scan all the finite elements already added in the parent ChMesh, and check if any has a face whose vertexes are
    /// all in the given node set; if so, add it to this mesh surface, with these rules:
    /// - surface elements inherited from ChLoadableUV: the element is added
    /// - face of ChElementTetrahedron : a ChTetrahedronFace proxy is created and added
    /// - face of ChElementHexahedron : a ChHexahedronFace proxy is created and added
    virtual void AddFacesFromNodeSet(std::vector<std::shared_ptr<ChNodeFEAbase> >& node_set) override {
    	ChMeshSurface::AddFacesFromNodeSet(node_set);
    	
    	auto mmesh=this->GetMesh();
    	std::unordered_set<size_t> node_set_map;
    	for (int i = 0; i < node_set.size(); ++i)
        	node_set_map.insert((size_t)node_set[i].get());

    	for (unsigned int ie = 0; ie < mmesh->GetNumElements(); ++ie) {
        	auto element = mmesh->GetElement(ie);

        if (auto tet = std::dynamic_pointer_cast<ChElementLDPM>(element)) {
            unsigned char nodes = 0;  // if the i-th bit is 1, the corressponding hex node is in the set
            for (int in = 0; in < 4; in++) {
                if (node_set_map.find((size_t)tet->GetTetrahedronNode(in).get()) != node_set_map.end())
                    nodes |= 1 << in;
            }
            unsigned char masks[] = {
                0x0E,  // 1110 face 0
                0x0D,  // 1101 face 1
                0x0B,  // 1011 face 2
                0x07   // 0111 face 3
            };
            for (int j = 0; j < 4; j++) {
                if ((nodes & masks[j]) == masks[j])
                    AddFace(chrono_types::make_shared<ChLDPMFace>(tet, j));
            }
          }

        
    	}
    }
    
    
    // Overriding a method from the base class
    virtual void AddFace(std::shared_ptr<ChLoadableUV> mface) override {
        // Custom implementation or additional checks
        ChMeshSurface::AddFace(mface);  // Call base class method
    }

    // Overriding a method to add custom behavior
    virtual void AddFacesFromBoundary() override {
        // Custom implementation for adding faces from boundary
        // Optionally call base class implementation
        ChMeshSurface::AddFacesFromBoundary();
        // Additional custom behavior
    }

private:
    // Additional members can be added here
    std::vector<std::shared_ptr<ChLoadableUV>> faces;
    
};


/// @} chrono_fea

}  // end namespace ldpm
}  // end namespace chrono

#endif

