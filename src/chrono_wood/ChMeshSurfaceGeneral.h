#ifndef CHMESHSURFACE_GENERAL_H
#define CHMESHSURFACE_GENERAL_H

//#include "chrono_ldpm/ChLdpmApi.h"
#include "chrono/physics/ChLoadable.h"
#include "chrono/fea/ChElementBase.h"

#include "chrono/fea/ChMeshSurface.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChTetrahedronFace.h"
#include "chrono/fea/ChHexahedronFace.h"
#include "chrono/geometry/ChTriangleMesh.h"

#include "delaunator.h"

using namespace chrono::fea;


//namespace chrono {
//namespace ldpm {

/// @addtogroup chrono_fea
/// @{

// Forward references (for parent hierarchy pointer)
//class ChMesh;
class MyTriangleFace : public chrono::ChLoadableUV {
public:
    std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> nodeA, nodeB, nodeC;

    MyTriangleFace(std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> nA, 
                   std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> nB, 
                   std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> nC) 
        : nodeA(nA), nodeB(nB), nodeC(nC) {}

    // Implement required methods of ChLoadableUV
	
	std::shared_ptr<ChNodeFEAxyzrot> GetNode(unsigned int i) const {
       
        switch (i) {
            case 0:
                return nodeA;
            case 1:
                return nodeB;
            case 2:
                return nodeC;          
        }
        return nullptr;
    }
	
	
    /// Fills the N shape function vector (size 3) with the values of shape functions at r,s 'area' coordinates, all
    /// ranging in [0...1].
    void ShapeFunctions(ChVectorN<double, 3>& N, double r, double s) {
        N(0) = 1.0 - r - s;
        N(1) = r;
        N(2) = s;
    };

    // Functions for ChLoadable interface

    /// Get the number of DOFs affected by this element (position part).
    virtual unsigned int GetLoadableNumCoordsPosLevel() override { return 3 * 3; }

    /// Get the number of DOFs affected by this element (speed part).
    virtual unsigned int GetLoadableNumCoordsVelLevel() override { return 3 * 3; }


     /// Get all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) override {
        mD.segment(block_offset + 0, 3) = GetNode(0)->GetPos().eigen();
        mD.segment(block_offset + 3, 3) = GetNode(1)->GetPos().eigen();
        mD.segment(block_offset + 6, 3) = GetNode(2)->GetPos().eigen();
    }

    /// Get all the DOFs packed in a single vector (speed part).
    virtual void LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) override {
        mD.segment(block_offset + 0, 3) = GetNode(0)->GetPosDt().eigen();
        mD.segment(block_offset + 3, 3) = GetNode(1)->GetPosDt().eigen();
        mD.segment(block_offset + 6, 3) = GetNode(2)->GetPosDt().eigen();
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override {
        for (int i = 0; i < 3; ++i) {
            GetNode(i)->NodeIntStateIncrement(off_x + i * 3, x_new, x, off_v + i * 3, Dv);
        }
    }

    /// Number of coordinates in the interpolated field: here the {x,y,z} displacement.
    virtual unsigned int GetNumFieldCoords() override { return 3; }

    /// Get the number of DOFs sub-blocks.
    virtual unsigned int GetNumSubBlocks() override { return 3; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(unsigned int nblock) override {
        return GetNode(nblock)->NodeGetOffsetVelLevel();
    }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(unsigned int nblock) override { return 3; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(unsigned int nblock) const override { return !GetNode(nblock)->IsFixed(); }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override {
        for (int i = 0; i < 3; ++i)
            mvars.push_back(&GetNode(i)->Variables());
    };

    /// Evaluate N'*F , where N is some type of shape function evaluated at U,V coordinates of the surface, each ranging
    /// in 0..+1 F is a load, N'*F is the resulting generalized load. Returns also det[J] with J=[dx/du,..], which may
    /// be useful in Gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in surface
                           const double V,              ///< parametric coordinate in surface
                           ChVectorDynamic<>& Qi,       ///< result of N'*F , maybe with offset block_offset
                           double& detJ,                ///< det[J]
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override {
        // evaluate shape functions (in compressed vector), btw. not dependant on state
        // note: U,V in 0..1 range, thanks to IsTriangleIntegrationNeeded() {return true;}
        ChVectorN<double, 3> N;
        this->ShapeFunctions(N, U, V);

        // determinant of jacobian is also =2*areaoftriangle, also length of cross product of sides
        ChVector3d p0 = GetNode(0)->GetPos();
        ChVector3d p1 = GetNode(1)->GetPos();
        ChVector3d p2 = GetNode(2)->GetPos();
        detJ = (Vcross(p2 - p0, p1 - p0)).Length();

        Qi(0) = N(0) * F(0);
        Qi(1) = N(0) * F(1);
        Qi(2) = N(0) * F(2);
        Qi(3) = N(1) * F(0);
        Qi(4) = N(1) * F(1);
        Qi(5) = N(1) * F(2);
        Qi(6) = N(2) * F(0);
        Qi(7) = N(2) * F(1);
        Qi(8) = N(2) * F(2);
    }

    /// If true, use quadrature over u,v in [0..1] range as triangle volumetric coords.
    virtual bool IsTriangleIntegrationNeeded() override { return true; }

    /// Get the normal to the surface at the parametric coordinate u,v.
    /// Normal must be considered pointing outside in case the surface is a boundary to a volume.
    virtual ChVector3d ComputeNormal(const double U, const double V) override {
        ChVector3d p0 = GetNode(0)->GetPos();
        ChVector3d p1 = GetNode(1)->GetPos();
        ChVector3d p2 = GetNode(2)->GetPos();
        return Vcross(p1 - p0, p2 - p0).GetNormalized();
    }

  
};

class ChMeshSurfaceGeneral : public ChMeshSurface {
public:
    // Constructor
    ChMeshSurfaceGeneral(ChMesh* parentmesh = nullptr) : ChMeshSurface(parentmesh) {}

    // Destructor
    virtual ~ChMeshSurfaceGeneral() {}
	
	
    /// Get owner mesh.
    ChMesh* GetMesh() { return ChMeshSurface::GetMesh(); }

    /// Set owner mesh.
    void SetMesh(ChMesh* mm) { ChMeshSurface::SetMesh(mm); }
    
    
    // Additional functionality can be added here
    void AdvancedFunctionality() {
        // Implementation of additional functionality
    }
    
		// Comparator for ChVector2d to use as map key
	struct ChVector2dComparator {
		bool operator()(const chrono::ChVector2d& lhs, const chrono::ChVector2d& rhs) const {
			if (lhs.x() == rhs.x()) {
				return lhs.y() < rhs.y();
			}
			return lhs.x() < rhs.x();
		}
	};


	// Function to compute a normal vector of the best-fit plane
	Eigen::Vector3d calculatePlaneNormal(const std::vector<std::shared_ptr<chrono::fea::ChNodeFEAbase>>& node_set) {
		Eigen::MatrixXd A(node_set.size(), 3);
		for (size_t i = 0; i < node_set.size(); ++i) {
			auto node=std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(node_set[i]);
			auto pos = node->GetPos();  // Get 3D position from ChNodeFEAxyzrot
			A(i, 0) = pos.x();
			A(i, 1) = pos.y();
			A(i, 2) = pos.z();
		}
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
		return svd.matrixV().col(2);  // Normal vector is the last column of V
	}


	// Project points onto a 2D plane based on the normal vector and enforce positive coordinates
	std::vector<chrono::ChVector2d> projectTo2D(const std::vector<std::shared_ptr<chrono::fea::ChNodeFEAbase>>& node_set, const Eigen::Vector3d& normal) {
		auto node=std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(node_set[0]);
		Eigen::Vector3d origin(node->GetPos().x(), node->GetPos().y(), node->GetPos().z());
		Eigen::Vector3d u, v;

		// Enforce an orientation for u to help yield positive results:
		if (normal.z() != 0) {
			u = Eigen::Vector3d(1, 0, -normal.x() / normal.z()).normalized();
		} else {
			u = Eigen::Vector3d(1, 0, 0);
		}
		v = normal.cross(u).normalized();

		std::vector<chrono::ChVector2d> projectedPoints;
		for (const auto& p : node_set) {
			auto node=std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(p);
			Eigen::Vector3d vec(node->GetPos().x(), node->GetPos().y(), node->GetPos().z());
			Eigen::Vector3d relVec = vec - origin;
			chrono::ChVector2d projPoint(relVec.dot(u), relVec.dot(v));
						
			projectedPoints.push_back(projPoint);
		}
		return projectedPoints;
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
    	for (int i = 0; i < node_set.size(); ++i){
        	node_set_map.insert((size_t)node_set[i].get());
		}
		
		// Step 1: Calculate the best-fit plane normal
		Eigen::Vector3d normal = calculatePlaneNormal(node_set);
		//std::cout << "Plane normal: (" << normal.x() << ", " << normal.y() << ", " << normal.z() << ")\n";

		// Step 2: Project points to 2D coordinates in the plane
		std::vector<ChVector2d> points2D = projectTo2D(node_set, normal);
		//std::cout << "Projected 2D points:\n";
		//for (const auto& pt : points2D) {
		//	std::cout << "(" << pt.x() << ", " << pt.y() << ")\n";
		//}
		
		// Map with custom comparator
		std::map<chrono::ChVector2d, std::shared_ptr<chrono::fea::ChNodeFEAbase>, ChVector2dComparator> mapto3D;
		//std::vector<chrono::ChVector2d> points2D; 
		int ind=0;
		for (const auto& node : node_set) {
			//chrono::ChVector2d projected_point(point.x(), point.y());       
			mapto3D[points2D[ind]] = node;
			ind++;
			//points2D.push_back(projected_point);        
		}

		std::vector<double> coords;
		for (const auto& vec : points2D) {
			coords.push_back(vec[0]);
			coords.push_back(vec[1]);
		}

		// Triangulation
		delaunator::Delaunator d(coords);

		for (size_t i = 0; i < d.triangles.size(); i += 3) {
			chrono::ChVector2d p1(d.coords[2 * d.triangles[i]], d.coords[2 * d.triangles[i] + 1]);
			chrono::ChVector2d p2(d.coords[2 * d.triangles[i + 1]], d.coords[2 * d.triangles[i + 1] + 1]);
			chrono::ChVector2d p3(d.coords[2 * d.triangles[i + 2]], d.coords[2 * d.triangles[i + 2] + 1]);
			auto node1=std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(mapto3D[p1]);
			auto node2=std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(mapto3D[p2]);
			auto node3=std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(mapto3D[p3]);
			auto face = chrono_types::make_shared<MyTriangleFace>(node1, node2, node3);
   
			this->AddFace(face);
			
			ChVector3<double> pos1= node1->GetPos();
			ChVector3<double> pos2= node2->GetPos();
			ChVector3<double> pos3= node3->GetPos();
			/*printf(
				"Triangle points: [[%f, %f, %f], [%f, %f, %f], [%f, %f, %f]]\n",
				pos1.x(), pos1.y(), pos1.z(),
				pos2.x(), pos2.y(), pos2.z(),
				pos3.x(), pos3.y(), pos3.z()
			);*/
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

//}  // end namespace ldpm
//}  // end namespace chrono

#endif

