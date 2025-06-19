#ifdef _WIN32
#include <windows.h>
#endif
#include <iostream>
#include <fstream>
#include <unordered_map>

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/utils/ChUtilsInputOutput.h"


#include "chrono_wood/ChRepresentedVolumeElement.h"
//#include "ChElementRVE.h"
#include "chrono_wood/ChElementCurvilinearBeamBezier.h"
#include "chrono_wood/ChElementCBLCON.h"

#include <omp.h>

#define EPS 1e-20
#define EPS_TRIDEGEN 1e-10




bool vector_in_vector(const std::vector<ChVector3d>& outer, const ChVector3d& target) {
    for (const auto& vec : outer) {
        // Check if the target vector is equal to the current vector in the outer vector
        if (vec == target) {
            return true; // Found the target vector inside the outer vector
        }
    }
    return false; // Target vector not found
}


// Equality operator defined outside the class
bool operator==(const ChVector3d& lhs, const ChVector3d& rhs) {
    return (lhs.x() == rhs.x()) && (lhs.y() == rhs.y()) && (lhs.z() == rhs.z());
}

// Specialize std::hash for ChVector3d
namespace std {
    template <>
    struct hash<ChVector3d> {
        std::size_t operator()(const ChVector3d& v) const {
            // Use the components x, y, and z to calculate a unique hash value
            std::size_t h1 = std::hash<double>{}(v.x());
            std::size_t h2 = std::hash<double>{}(v.y());
            std::size_t h3 = std::hash<double>{}(v.z());
            return h1 ^ (h2 << 1) ^ (h3 << 2);  // Combine the hash values
        }
    };
}



namespace chrono {
namespace wood {

// Define tolerance for floating-point comparisons
const double TOLERANCE = 1e-6;
//





std::string getCurrentDirectory() {
    char buffer[260];

#ifdef _WIN32
    GetModuleFileName(NULL, buffer, 260);
    std::string path(buffer);
    return path.substr(0, path.find_last_of("\\/"));
#else
    ssize_t count = readlink("/proc/self/exe", buffer, 260);
    std::string path(buffer);
    return path.substr(0, path.find_last_of("/")-5);
#endif
}

void writeSurfaceMeshtoVtk(const std::string& filename, const std::vector<ChVector3d>& vertices, const std::vector<ChVector3<int>>& faces) {
    // Open the file for writing
	std::string current_dir;
    current_dir=getCurrentDirectory();
	
    std::ofstream vtk_file(current_dir+"/"+filename+".vtk");
    if (!vtk_file.is_open()) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }

    // Write the header
    vtk_file << "# vtk DataFile Version 3.0\n";
    vtk_file << "3D Mesh\n";
    vtk_file << "ASCII\n";
    vtk_file << "DATASET POLYDATA\n";

    // Write the vertices
    vtk_file << "POINTS " << vertices.size() << " float\n";
    for (const auto& v : vertices) {
        vtk_file << v[0] << " " << v[1] << " " << v[2] << "\n";
    }

    // Write the faces
    vtk_file << "POLYGONS " << faces.size() << " " << faces.size() * 4 << "\n";
    for (const auto& face : faces) {
        vtk_file << 3 << " " << face[0] << " " << face[1] << " " << face[2] << "\n";
    }

    // Close the file
    vtk_file.close();
}


void VisualizeSurfaceMesh(const std::shared_ptr<ChMeshSurface>& surfaceMesh, const std::string& filename) {
	const auto& faces = surfaceMesh->GetFaces();
	std::vector<ChVector3<int>> Faces;
	std::vector<ChVector3d> vertices;
	std::unordered_map<ChVector3d, int> NodeMap;
    //double total_area=0;
	int knode=0;
    for (const auto& face : faces) {
        if (face) {
			auto Lface=std::dynamic_pointer_cast<MyTriangleFace>(face);
            // Access the vertices of the face
            ChVector3d v1 = Lface->GetNode(0)->GetPos();  // First vertex
            ChVector3d v2 = Lface->GetNode(1)->GetPos();  // Second vertex
            ChVector3d v3 = Lface->GetNode(2)->GetPos();  // Third vertex
			
			
			for (int in=0; in<3; in++){
				ChVector3d target=Lface->GetNode(in)->GetPos();
				if (vector_in_vector(vertices, target)) {
					continue;
				} else {
					vertices.push_back(target);			
					NodeMap[target]=knode;
					knode += 1;
				}
		    }
			
			Faces.push_back(ChVector3<int> {NodeMap[v1], NodeMap[v2], NodeMap[v3]});
			//std::cout<<"vertex: "<<v1<<"\t"<<v2<<"\t"<<v3<<std::endl;
            // Calculate the area of the triangular face
            /*auto edge1 = v2 - v1;
            auto edge2 = v3 - v1;
            double area = 0.5 * (edge1.Cross(edge2)).Length();  // Triangle area formula
            total_area += area;*/
        }
    }
	
	
	/*std::cout << "\nUsing iterators:" <<NodeMap.size()<< std::endl;
    // Print out the contents of the unordered_map
    for (const auto& pair : NodeMap) {
        std::cout << "Node: (" << pair.first.x() << ", " << pair.first.y() << ", " << pair.first.z() 
                  << ") -> Value: " << pair.second << std::endl;
    }*/
	
	//exit(0);	
	writeSurfaceMeshtoVtk(filename, vertices, Faces);

}



double manhattan_dist(std::shared_ptr<ChNodeFEAxyzrot>& nodeA){
	ChVector3d Pos=nodeA->GetPos();
	return Pos.x()+Pos.y()*10E+6+Pos.z()*10E+12;
}

// Function to check if a value is "almost equal" to a given reference
bool is_equal(double a, double b) {
    return std::abs(a - b) < TOLERANCE;
}

// Function to check for duplicates in vector
bool contains(const std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& vec, std::shared_ptr<ChNodeFEAxyzrot>& node ) {
    return std::find(vec.begin(), vec.end(), node) != vec.end();
}


template <typename T>
void RemoveItemsFromVector(std::vector<T>& source, const std::vector<T>& items_to_remove) {
    // Remove elements from 'source' that are present in 'items_to_remove'
    source.erase(std::remove_if(source.begin(), source.end(),
                                [&items_to_remove](const T& item) {
                                    return std::find(items_to_remove.begin(), items_to_remove.end(), item) != items_to_remove.end();
                                }),
                 source.end());
}


double PointTriangleDistance(const ChVector3d& B,
                             const ChVector3d& A1,
                             const ChVector3d& A2,
                             const ChVector3d& A3,
                             double& mu,
                             double& mv,
                             bool& is_into,
                             ChVector3d& Bprojected) {
    // defaults
    is_into = false;
    mu = mv = -1;
    double mdistance = 10e22;

    ChVector3d Dx, Dy, Dz, T1, T1p;

    Dx = Vsub(A2, A1);
    Dz = Vsub(A3, A1);
    Dy = Vcross(Dz, Dx);

    double dylen = Vlength(Dy);

    if (fabs(dylen) < EPS_TRIDEGEN)  // degenerate triangle
        return mdistance;

    Dy = Vmul(Dy, 1.0 / dylen);

    ChMatrix33<> mA(Dx, Dy, Dz);

    // invert triangle coordinate matrix -if singular matrix, was degenerate triangle-.
    if (std::abs(mA.determinant()) < 1E-8)
        return mdistance;

    ChMatrix33<> mAi = mA.inverse();
    T1 = mAi * (B - A1);
    T1p = T1;
    //std::cout<<"T1: "<<T1<<"\t";
    T1p.y() = 0;
    Bprojected = A1 + mA * T1p;
    //
    T1 = mAi * (Bprojected - A1);
    T1p = T1;
    T1p.y() = 0;
    mu = T1.x();
    mv = T1.z();
    mdistance = -T1.y();
    
    if (mu >= 0 && mv >= 0 && mv <= 1.0 - mu) {
        is_into = true;   
        //Bprojected = A1 + mA * T1p;     
    }

    return mdistance;
}
//
// Create the face-node constraint between a xyzRot node and triangular face in a mesh (having xyz node)
//
bool TieNodeToTriFace(ChSystem& sys, std::shared_ptr<ChMeshSurface> m_surface, std::shared_ptr<ChNodeFEAxyzrot> m_node,  double OffsetVal, double max_dist=0.0001) {    
    // Check all faces if the considered node is close enough to face    
    for (const auto& face : m_surface->GetFaces()) {  
		//auto face_tetra1 = std::dynamic_pointer_cast<ChLDPMFace>(face);
		auto face_tetra = std::dynamic_pointer_cast<MyTriangleFace>(face);
        if ( face_tetra ) {
            double val, u, v, w;
            bool is_into;
            ChVector3d p_projected;
            //
            // Get face nodes
            //
            auto node0 = std::static_pointer_cast<ChNodeFEAxyzrot>(face_tetra->GetNode(0));
            auto node1 = std::static_pointer_cast<ChNodeFEAxyzrot>(face_tetra->GetNode(1));
            auto node2 = std::static_pointer_cast<ChNodeFEAxyzrot>(face_tetra->GetNode(2)); 
            //
            // coordinate of the nodes
            //
            ChVector3d p0 = node0->GetPos();
            ChVector3d p1 = node1->GetPos();
            ChVector3d p2 = node2->GetPos();
            //std::cout<< "p0: "<<p0<<"\t";
            //std::cout<< "p1: "<<p1<<"\t";
            //std::cout<< "p2: "<<p2<<"\t";
            //
            //std::cout<< "pN: "<<m_node->GetPos()<<std::endl;
            ChVector3d pN = m_node->GetPos();            
            // check if node is on the surface
            val = PointTriangleDistance(
                pN, p0, p1, p2, u, v, is_into, p_projected);
            //std::cout<<"PN: "<<pN<<"\t";
            //std::cout<<" p_projected: "<<p_projected<<" is_into: "<< is_into <<std::endl;
            val = fabs(val);
			//std::cout<<"is_into: "<<is_into<<"  "<<"val: "<<val<<"\n";
            w = 1 - u - v;
            if (!is_into)
                // val += std::max(std::max(0.0,u-1.0),-ChMin(0.0,u)) + std::max(std::max(0.0,v-1.0),-ChMin(0.0,v));
                val += std::max(0.0, -u) + std::max(0.0, -v) + std::max(0.0, -w);
            if (val < max_dist) { 
            	if (u>=0.9999){            		
    			auto constraint1=chrono_types::make_shared<ChLinkNodeNodeRot>();		
    			constraint1->Initialize(m_node, node1);  
    			sys.Add(constraint1);
    			//std::cout<<"U=1: "<<m_node->GetPos()<<"\t"<<node1->GetPos()<<"\t";
				//std::cout<<" p_projected: "<<p_projected<<std::endl;
    			
            	}else if(v>=0.9999){
            		auto constraint1=chrono_types::make_shared<ChLinkNodeNodeRot>();
    			constraint1->Initialize(m_node, node2);
    			sys.Add(constraint1);
    			//std::cout<<"v=1: "<<m_node->GetPos()<<"\t"<<node2->GetPos()<<"\t";
				//std::cout<<" p_projected: "<<p_projected<<std::endl;
    		}else if(w>=0.9999){
            		auto constraint1=chrono_types::make_shared<ChLinkNodeNodeRot>();
    			constraint1->Initialize(m_node, node0);
    			sys.Add(constraint1);
    			//std::cout<<"w=1: "<<m_node->GetPos()<<"\t"<<node0->GetPos()<<"\t";
				//std::cout<<" p_projected: "<<p_projected<<std::endl;
            	}else{
            	auto constraint1 = std::make_shared<ChLinkNodeRotFaceRot>();
            	constraint1->SetOffset(OffsetVal);
    	    	constraint1->Initialize(m_node, node0, node1, node2);  	    		    	    	    	
    	    	sys.Add(constraint1);     	    	
    	    	//std::cout<<"inside face "<<m_node->GetPos()<<"\t"<<node0->GetPos()<<"\t"<<node1->GetPos()<<"\t"<<node2->GetPos()<<"\t";
				//std::cout<<" p_projected: "<<p_projected<<std::endl;
    	    	
    	    	}   	    	
    	    	return true;
    	    }
            
        }
        //// TODO: other types of elements
    }
	
	
	std::cout<<"Unseccesful "<<m_node->GetPos()<<"\n";
	//exit(0);			
    return false; 
}




bool TieNodeToLine(ChSystem& sys, std::shared_ptr<ChMeshLine> m_edge, std::shared_ptr<ChNodeFEAxyzrot> m_node,  double OffsetVal, double max_dist=0.0001) {    
    // Check all faces if the considered node is close enough to face    
    for (const auto& edge : m_edge->GetBeamsList()) {    
		//std::cout<<"edge: "<<edge<<std::endl;
        if (auto edge_beam = std::dynamic_pointer_cast<ChElementBeamEuler>(edge)) {
            double val, u;
            bool is_insegment;
            ChVector3d p_projected;
            //
            // Get face nodes
            //
            auto node0 = std::static_pointer_cast<ChNodeFEAxyzrot>(edge_beam->GetNode(0));
            auto node1 = std::static_pointer_cast<ChNodeFEAxyzrot>(edge_beam->GetNode(1));            
            //
            // coordinate of the nodes
            //
            ChVector3d p0 = node0->GetPos();
            ChVector3d p1 = node1->GetPos();            
            //std::cout<< "p0: "<<p0<<"\t";
            //std::cout<< "p1: "<<p1<<"\t";            
            //
            //std::cout<< "pN: "<<m_node->GetPos()<<"\t";
            ChVector3d pN = m_node->GetPos();            
            // check if node is on the surface
            val = utils::PointLineDistance(
                pN, p0, p1, u, is_insegment);
            //std::cout<<" p_projected: "<<p_projected<<std::endl;
            val = fabs(val);   
             
                    
            if (!is_insegment)                
                val += std::max(0.0, -u);
            if (is_insegment ) { 
            	
            	if (u<=0.0001){
            		auto constraint1=chrono_types::make_shared<ChLinkNodeNodeRot>();
    			constraint1->Initialize(m_node, node0);
    			sys.Add(constraint1);
    			//std::cout<<"u: "<<u<<" node to node0 "<<std::endl;
            	}else if(u>=0.9999){
            		auto constraint1=chrono_types::make_shared<ChLinkNodeNodeRot>();
    			constraint1->Initialize(m_node, node1);
    			sys.Add(constraint1);
    			//std::cout<<"u: "<<u<<" node to node1 "<<std::endl;
            	}else{
            		auto constraint1 = std::make_shared<ChLinkPoint2LineRot>();
            		constraint1->SetOffset(OffsetVal);
    	    		constraint1->Initialize(m_node, node0, node1);  
    	    		sys.Add(constraint1);
    	    		//std::cout<<"u: "<<u<<" node to line "<<std::endl;
    	    	}    	    	  	    	
    	    	return true;
    	    }    	     
            
        }
        //// TODO: other types of elements
    }
    return false; 
}



void ChRepresentedVolumeElement::SetMacroStrainOfElements() {
#pragma omp parallel for        
        for (int i = 0; i < mmesh->GetNumElements(); ++i) {
            //
            // CSL ElEMENTS
            //	
					
            
			
            if( auto connector = std::dynamic_pointer_cast<ChElementCBLCON>(mmesh->GetElement(i)) ){
				connector->macro_strain = macro_strain;
			}else if ( auto beam = std::dynamic_pointer_cast<ChElementCurvilinearBeamBezier>(mmesh->GetElement(i)) ){
				beam->macro_strain = macro_strain;
			}
            //
            // CSL ElEMENTS
            //
            
            // to do
            
            //
            // Connector ElEMENTS
            //
            
            // to do
            
            
            //
            // Bezier Beam ElEMENTS
            //
            
            // to do
        	
    	}
    }

void ChRepresentedVolumeElement::SetVertices(std::vector<ChVector3d> myvertices) {
    
    x_min=1000000;
    y_min=1000000;
    z_min=1000000;
    x_max=-1000000;
    y_max=-1000000;
    z_max=-1000000;
    for (ChVector3d vertice:myvertices){
    	if (vertice.x()<x_min)
    		x_min=vertice.x();
    	if (vertice.x()>x_max)
    		x_max=vertice.x();
    	if (vertice.y()<y_min)
    		y_min=vertice.y();
    	if (vertice.y()>y_max)
    		y_max=vertice.y();
    	if (vertice.z()<z_min)
    		z_min=vertice.z();
    	if (vertice.z()>z_max)
    		z_max=vertice.z();
    }
    
	std::cout<<x_min<<"\t";std::cout<<x_max<<"\n";
	std::cout<<y_min<<"\t";std::cout<<y_max<<"\n";
	std::cout<<z_min<<"\t";std::cout<<z_max<<"\n";
	
    this->SetCubeEdgeLength(x_max-x_min);
    
    vertices = {
    {"v1", {x_min, y_min, z_min}},
    {"v2", {x_max, y_min, z_min}},
    {"v3", {x_max, y_max, z_min}},
    {"v4", {x_min, y_max, z_min}},    
    {"v5", {x_min, y_min, z_max}},
    {"v6", {x_max, y_min, z_max}},
    {"v7", {x_max, y_max, z_max}},
    {"v8", {x_min, y_max, z_max}}    
	};

}



// Function to categorize nodes into nodesets based on surface
void ChRepresentedVolumeElement::CreateSurfaceNodeSets(const std::shared_ptr<ChMesh> mesh,
                    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& nodeset_xmin,
                    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& nodeset_xmax,
                    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& nodeset_ymin,
                    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& nodeset_ymax,
                    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& nodeset_zmin,
                    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& nodeset_zmax) {
    // Find the min and max bounds of the cube
    double xmin = 1E+20, xmax = -1E+20;
    double ymin = 1E+20, ymax = -1E+20;
    double zmin = 1E+20, zmax = -1E+20;

    for (int idn=0; idn<mesh->GetNumNodes(); idn++ ) {
		auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(idn));
		auto pos = node -> GetPos();
        if (pos.x() < xmin) xmin = pos.x();
        if (pos.x() > xmax) xmax = pos.x();
        if (pos.y() < ymin) ymin = pos.y();
        if (pos.y() > ymax) ymax = pos.y();
        if (pos.z() < zmin) zmin = pos.z();
        if (pos.z() > zmax) zmax = pos.z();
    }
	
	std::cout<<"xmin: "<<xmin<<"\t"<<"xmax: "<<xmax<<"\n";
	std::cout<<"ymin: "<<ymin<<"\t"<<"ymax: "<<ymax<<"\n";
	std::cout<<"zmin: "<<zmin<<"\t"<<"zmax: "<<zmax<<"\n";
    
    // Classify nodes into respective nodesets
    for (int idn=0; idn<mesh->GetNumNodes(); idn++ ) {
		auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh->GetNode(idn));
		auto pos = node -> GetPos();
        if (is_equal(pos.x(), xmin) && !contains(nodeset_xmin, node )) nodeset_xmin.push_back(node);
        if (is_equal(pos.x(), xmax) && !contains(nodeset_xmax, node )) nodeset_xmax.push_back(node);
        if (is_equal(pos.y(), ymin) && !contains(nodeset_ymin, node )) nodeset_ymin.push_back(node);
        if (is_equal(pos.y(), ymax) && !contains(nodeset_ymax, node )) nodeset_ymax.push_back(node);
        if (is_equal(pos.z(), zmin) && !contains(nodeset_zmin, node )) nodeset_zmin.push_back(node);
        if (is_equal(pos.z(), zmax) && !contains(nodeset_zmax, node )) nodeset_zmax.push_back(node);
    }
}



ChRepresentedVolumeElement::ChRepresentedVolumeElement(ChSystem& sys, std::shared_ptr<ChMesh> mesh) : msystem(sys), mmesh(mesh) {	
    	
    	static_analysis.SetCorrectionTolerance(1e-4, 1e-8);
    	static_analysis.SetIncrementalSteps(10);
    	static_analysis.SetMaxIterations(100);
    	static_analysis.SetVerbose(true); 
    	
	this->SetupInitial();
}

ChRepresentedVolumeElement::~ChRepresentedVolumeElement() {}

void ChRepresentedVolumeElement::SetupInitial() {
     //
     macro_strain = std::make_shared<chrono::ChMatrixNM<double, 1, 9>>();
     macro_stress = std::make_shared<chrono::ChMatrixNM<double, 1, 9>>();
     macro_strain->setZero();  // Initialize with zeros
     macro_stress->setZero();  // Initialize with zeros
     //
     // Geometrical definition of cube
     //
	 std::cout<<"vertices "<<vertices["v1"]<<std::endl;
	 std::cout<<"vertices "<<vertices["v2"]<<std::endl;
	 std::cout<<"vertices "<<vertices["v3"]<<std::endl;
	 std::cout<<"vertices "<<vertices["v4"]<<std::endl;
	 std::cout<<"vertices "<<vertices["v5"]<<std::endl;
	 std::cout<<"vertices "<<vertices["v6"]<<std::endl;
	 std::cout<<"vertices "<<vertices["v7"]<<std::endl;
	 std::cout<<"vertices "<<vertices["v8"]<<std::endl;
    if (vertices.empty()){    
    vertices = {
    {"v1", {x_min, y_min, z_min}},
    {"v2", {x_max, y_min, z_min}},
    {"v3", {x_max, y_max, z_min}},
    {"v4", {x_min, y_max, z_min}},    
    {"v5", {x_min, y_min, z_max}},
    {"v6", {x_max, y_min, z_max}},
    {"v7", {x_max, y_max, z_max}},
    {"v8", {x_min, y_max, z_max}}    
	};
    }
	
    // Define the 12 edges by pairs of vertices
    CubEdges = {
    {"v1", "v2"}, {"v1", "v4"}, {"v1", "v5"},
    {"v3", "v4"}, {"v7", "v8"}, {"v5", "v6"},
    {"v2", "v3"}, {"v6", "v7"}, {"v5", "v8"},
    {"v2", "v6"}, {"v3", "v7"}, {"v4", "v8"}};
    
    
    // Define the 6 faces using 4 vertices
    CubFaces = {
    {"v1", "v5", "v8", "v4"}, 
    {"v1", "v2", "v6", "v5"}, 
    {"v1", "v4", "v3", "v2"}, 
    {"v2", "v3", "v7", "v6"},
    {"v3", "v4", "v8", "v7"},
    {"v5", "v6", "v7", "v8"} };
    // 
    //
    //
    //
    //ApplyPeriodicBoundaryConditions(); 
    //    
    //
    SetMacroStrainOfElements();
    std::cout<<"Macrostrain asseignment is completed\n";

}

/*
void ChRepresentedVolumeElement::ApplyPeriodicBoundaryConditions() {   
    ///
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Define interface constraint between periodic boundaries
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    
     //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Create node sets for periodic boundaries
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    
    
    //exit(9);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Create node sets for outer surfaces 
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //    
    
    // Nodes of the load surface are those of the nodeset with label BC_SURF:
    std::vector<std::shared_ptr<ChNodeFEAbase> > nodes_left_surf;
    std::vector<std::shared_ptr<ChNodeFEAbase> > nodes_back_surf;
    std::vector<std::shared_ptr<ChNodeFEAbase> > nodes_bottom_surf;
    //
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > nodes_right_surf;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > nodes_front_surf;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > nodes_top_surf;
    std::vector<std::shared_ptr<ChNodeFEAbase> > nodes_bottom_left_edge;
    std::shared_ptr<ChNodeFEAxyzrot> center_node;
    //
    //
    // Select primary surfaces 
    //
    //
    auto MeshNodeList=mmesh->GetNodes();
    std::shared_ptr<ChNodeFEAxyzrot> node_v1;
    std::shared_ptr<ChNodeFEAxyzrot> node_v2;
    std::shared_ptr<ChNodeFEAxyzrot> node_v5;
    std::shared_ptr<ChNodeFEAxyzrot> node_v3, node_v4, node_v6, node_v7, node_v8;
    std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAxyzrot>>> cube_faces;
    for (int i=0; i< MeshNodeList.size(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(MeshNodeList[i]);
    	ChVector3d p = node->GetPos();  
    	//
    	// Left surface ----> minusX
    	//    
    	if(abs(p.x()-x_min)<TOLERANCE ){
    		//std::cout<<"node i="<<i<<"  p: "<<p<<std::endl;
    		nodes_left_surf.push_back(node);   
    		if (abs(p.y()-y_min)<TOLERANCE &  abs(p.z()-z_min)<TOLERANCE )
    			node_v1= node;
    		if (abs(p.z()-z_max)<TOLERANCE & abs(p.y()-y_min)<TOLERANCE)
    			node_v5= node;	
    		if (abs(p.y()-y_max)<TOLERANCE & abs(p.z()-z_min)<TOLERANCE)
    			node_v4= node;
    		if (abs(p.y()-y_max)<TOLERANCE & abs(p.z()-z_max)<TOLERANCE)
    			node_v8= node;	
    	}  
    	
    	//
    	// Rear surface ----> minusZ
    	//    
    	if(abs(p.z()-z_min)<TOLERANCE){
    		//std::cout<<"node i="<<i<<"  p: "<<p<<std::endl;
    		nodes_back_surf.push_back(node);
    		if (abs(p.x()-x_max)<TOLERANCE & abs(p.y()-y_min)<TOLERANCE){
    			node_v2= node;
    		}
    		
    		if (abs(p.x()-x_max)<TOLERANCE & abs(p.y()-y_max)<TOLERANCE)
    			node_v3= node;
    	}  
    	
    	//
    	// Bottom surface ----> minusY
    	//     	
    	if(abs(p.y()-y_min)<TOLERANCE){
    		//std::cout<<"node i="<<i<<"  p: "<<p<<std::endl;
    		nodes_bottom_surf.push_back(node);
    		if (abs(p.x()-x_max)<TOLERANCE & abs(p.z()-z_max)<TOLERANCE)
    			node_v6= node;
    	}   
    	
    	
    	if (abs(p.x()-x_max)<TOLERANCE & abs(p.y()-y_max)<TOLERANCE & abs(p.z()-z_max)<TOLERANCE)
    			node_v7= node;
    	
    }
    
    
    //node_v1->SetFixed(true);
   
    //
    // make a vector from  members of primary surfaces and remove them from total node list
    //
    std::vector<std::shared_ptr<ChNodeFEAbase> > used_nodes=nodes_left_surf;
    used_nodes.insert(used_nodes.end(), nodes_back_surf.begin(), nodes_back_surf.end());
    used_nodes.insert(used_nodes.end(), nodes_bottom_surf.begin(), nodes_bottom_surf.end());
    
    RemoveItemsFromVector(MeshNodeList, used_nodes);
   
    //
    //
    // Select secondary surfaces 
    //
    // 
   
    for (int i=0; i< MeshNodeList.size(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(MeshNodeList[i]);
    	ChVector3d p = node->GetPos();  
    	//
    	// Left surface ----> minusX
    	//    
    	if(abs(p.x()-x_max)<TOLERANCE ){
    		//std::cout<<"node i="<<i<<"  p: "<<p<<std::endl;
    		nodes_right_surf.push_back(node);     		
    	}  
    	
    	//
    	// Rear surface ----> minusZ
    	//    
    	if(abs(p.z()-z_max)<TOLERANCE){
    		//std::cout<<"node i="<<i<<"  p: "<<p<<std::endl;
    		nodes_front_surf.push_back(node);
    	}  
    	
    	//
    	// Bottom surface ----> minusY
    	//     	
    	if(abs(p.y()-y_max)<TOLERANCE){
    		//std::cout<<"node i="<<i<<"  p: "<<p<<std::endl;
    		nodes_top_surf.push_back(node);
    	}   
    	
    }
    
    std::cout<<" nodes_left_surf "<<nodes_left_surf.size()<<std::endl;
    std::cout<<" nodes_right_surf "<<nodes_right_surf.size()<<std::endl;
    std::cout<<" nodes_back_surf "<<nodes_back_surf.size()<<std::endl;
    std::cout<<" nodes_front_surf "<<nodes_front_surf.size()<<std::endl;
    std::cout<<" nodes_bottom_surf "<<nodes_bottom_surf.size()<<std::endl;
    std::cout<<" nodes_top_surf "<<nodes_top_surf.size()<<std::endl;
    //std::cout << "center_node->GetIndex() : "<<center_node->GetIndex()<<std::endl;
    
    
   
   
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Apply constraints
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    // Tie node to fem surface
    //   
    auto mtruss = chrono_types::make_shared<ChBody>();
    mtruss->SetFixed(true);  
    mtruss->EnableCollision(false);	
    msystem.Add(mtruss);
     
    std::string NS_name_left="SET-CONCRETE-LS";
    double offsetVal=CubeEdgeLength;
    double max_dist=TOLERANCE;
    int const_num=0;
    for (int i = 0; i < nodes_right_surf.size(); ++i) {    	
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(nodes_right_surf[i]);   
    	node->SetFixed(true);
    	continue; 	
    	auto constr=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);		
    	constr->Initialize(node, mtruss, false, node->Frame(), node->Frame());     
    	msystem.Add(constr);
    	++const_num;      	  		
    }
    
    std::cout<<nodes_right_surf.size()<<" out of "<< const_num<< " node is attached to left side of core part (tet surface) "<<std::endl;
    
    const_num=0;
    for (int i = 0; i < nodes_front_surf.size(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(nodes_front_surf[i]);       
    	node->SetFixed(true);
    	continue; 		       
    	auto constr=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);		
    	constr->Initialize(node, mtruss, false, node->Frame(), node->Frame());     
    	msystem.Add(constr);
    	++const_num;      	      		
    }
    std::cout<<nodes_front_surf.size()<<" out of "<<const_num<< " node is attached to right side of core part (tet surface) "<<std::endl;   
    
    
    const_num=0;
    for (int i = 0; i < nodes_top_surf.size(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(nodes_top_surf[i]);  
    	node->SetFixed(true);
    	continue;    	      
    	auto constr=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);		
    	constr->Initialize(node, mtruss, false, node->Frame(), node->Frame());     
    	msystem.Add(constr);
    	++const_num;      	     		
    }
    std::cout<<nodes_top_surf.size()<<" out of "<<const_num<< " node is attached to top side of core part (tet surface) "<<std::endl;
    
   
   
   
    const_num=0;
    for (int i = 0; i < nodes_left_surf.size(); ++i) {    	
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(nodes_left_surf[i]);    
    	node->SetFixed(true);
    	continue; 	
    	auto constr=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);		
    	constr->Initialize(node, mtruss, false, node->Frame(), node->Frame());     
    	msystem.Add(constr);
    	++const_num;      	  		
    }
    
    std::cout<<nodes_right_surf.size()<<" out of "<< const_num<< " node is attached to left side of core part (tet surface) "<<std::endl;
    
    const_num=0;
    for (int i = 0; i < nodes_back_surf.size(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(nodes_back_surf[i]);    
    	node->SetFixed(true);
    	continue;    		       
    	auto constr=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);		
    	constr->Initialize(node, mtruss, false, node->Frame(), node->Frame());     
    	msystem.Add(constr);
    	++const_num;      	      		
    }
    std::cout<<nodes_front_surf.size()<<" out of "<<const_num<< " node is attached to right side of core part (tet surface) "<<std::endl;   
    
    
    const_num=0;
    for (int i = 0; i < nodes_bottom_surf.size(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(nodes_bottom_surf[i]);     
    	node->SetFixed(true);
    	continue; 	      
    	auto constr=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);		
    	constr->Initialize(node, mtruss, false, node->Frame(), node->Frame());     
    	msystem.Add(constr);
    	++const_num;      	     		
    }
    std::cout<<nodes_top_surf.size()<<" out of "<<const_num<< " node is attached to top side of core part (tet surface) "<<std::endl;
    
  
}

*/






void ChRepresentedVolumeElement::ApplyPeriodicBoundaryConditions() {   
    ///
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Define interface constraint between periodic boundaries
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
     // Nodesets for each surface
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodeset_xmin, nodeset_xmax;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodeset_ymin, nodeset_ymax;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodeset_zmin, nodeset_zmax;

    // Categorize nodes into nodesets
    CreateSurfaceNodeSets(this->mmesh, nodeset_xmin, nodeset_xmax, nodeset_ymin, nodeset_ymax, nodeset_zmin, nodeset_zmax);

    // Output the nodesets
    std::cout << "Nodeset XMIN: ";
    for (auto node : nodeset_xmin) std::cout << node->GetPos() << "\n";
    std::cout << "\n";

    std::cout << "Nodeset XMAX: ";
    for (auto node : nodeset_xmax) std::cout << node->GetPos() << "\n ";
    std::cout << "\n";
	
    std::cout << "Nodeset YMIN: ";
    for (auto node : nodeset_ymin) std::cout << node->GetPos() << "\n ";
    std::cout << "\n";

    std::cout << "Nodeset YMAX: ";
    for (auto node : nodeset_ymax) std::cout << node->GetPos() << "\n ";
    std::cout << "\n";
    
    std::cout << "Nodeset ZMIN: ";
    for (auto node : nodeset_zmin) std::cout << node->GetPos() << "\n ";
    std::cout << "\n";

    std::cout << "Nodeset ZMAX: ";
    for (auto node : nodeset_zmax) std::cout << node->GetPos() << "\n ";
    std::cout << "\n";
	//exit(0);
     //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Create node sets for periodic boundaries
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //
	double max_dist= TOLERANCE*CubeEdgeLength;
	std::cout<<"max_dist: "<<max_dist<<std::endl;
	
    std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAxyzrot>>> edge_sets;
	double val, u;
	bool is_insegment;
	for (size_t i = 0; i < CubEdges.size(); ++i) {
	    std::cout<<i<<"  CubEdges[i].first: "<<CubEdges[i].first<<"\tCubEdges[i].second: "<<CubEdges[i].second<<std::endl;
	    const auto& p0 = vertices[CubEdges[i].first];
	    const auto& p1 = vertices[CubEdges[i].second];
		std::cout<<"p0: "<<p0<<"  p1: "<<p1<<std::endl;
	    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> edge_nodes;
	
	    for (size_t ind=0; ind< mmesh->GetNumNodes(); ind++) {
	        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mmesh->GetNode(ind));
	    	ChVector3d pN = node->GetPos();			
	    	val = utils::PointLineDistance(pN, p0, p1, u, is_insegment);
			
			if (val<TOLERANCE) {		
				edge_nodes.push_back(node);
			}
	    }
	    std::sort(edge_nodes.begin(), edge_nodes.end(), []( std::shared_ptr<ChNodeFEAxyzrot>& nodeA, std::shared_ptr<ChNodeFEAxyzrot>& nodeB) { return manhattan_dist(nodeA) < manhattan_dist(nodeB); });
		std::cout<<"edge_nodes: "<<edge_nodes.size()<<std::endl;		
	    if (i>2 && edge_nodes.size()>0){
			edge_nodes.erase(edge_nodes.begin());
			edge_nodes.erase(edge_nodes.end() - 1);
	    }
	    edge_sets["edge_" + std::to_string(i+1)] = edge_nodes;
	}

	for (size_t i = 0; i < CubEdges.size(); ++i) {
		auto edge=edge_sets["edge_" + std::to_string(i+1)];
		std::cout<<"edge_" << std::to_string(i+1)<<"\t"<<edge.size()<<std::endl;
		for (auto node : edge)
			std::cout<<"node_pos: "<<node->GetPos()<<std::endl;
		
	}
	
	//exit(8);
	
    for (int iedg=0; iedg<3; iedg++){
    	auto primary_line=chrono_types::make_shared<ChMeshLine>();  
    	primary_line->SetMesh(mmesh.get()); 
    	//primary_line->AddBeamsFromNodeSet(edge_sets["edge_" + std::to_string(iedg+1)]); 
		if (edge_sets["edge_" + std::to_string(iedg+1)].size()>0) {
			primary_line->AddBeamsFromGivenNodeSet(edge_sets["edge_" + std::to_string(iedg+1)]);
		}else{
			continue;
		}
    	std::cout<<"//////////////////////////   Primary edge:" <<"\t" <<this->GetRVEdge(iedg).first<< "-" << this->GetRVEdge(iedg).second << "  /////////////////////////////////\n" << std::endl;
    	std::cout<<"edge_sets{} "<<edge_sets["edge_" + std::to_string(iedg+1)].size()<<std::endl;
    	//
    	// secondary lines:
    	//
    	
    	double OffsetVal=abs(CubeEdgeLength);
    	for (int jedg=iedg*3+3; jedg<iedg*3+6; jedg++){
    		auto secondary_line=edge_sets["edge_" + std::to_string(jedg+1)];
    		std::cout<<"---------------------Secondary edge:"<<"\t" <<CubEdges[jedg].first<< "-" << CubEdges[jedg].second <<"------------------------\n"<<std::endl;
	    	int const_num=0;
			for (int i = 0; i < secondary_line.size(); ++i) {        	
	    		auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(secondary_line[i]);
	    		std::cout<<"*********posNode:\t"<<node->GetPos().x()<<" "<<node->GetPos().y()<<" "<<node->GetPos().z()<<"******:\n";
	    		bool is_tied=TieNodeToLine( msystem, primary_line, node, OffsetVal, TOLERANCE); 
	    		if (is_tied)
	    			++const_num;      	  		
	    	}
			
			std::cout<< edge_sets["edge_" + std::to_string(jedg+1)].size() <<" out of "<<const_num<<" is tied to "<<"edge_" + std::to_string(iedg+1)<<std::endl;
    
    	}
		
		
    }
    
    //exit(9);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Create node sets for outer surfaces 
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //    
    
    // Nodes of the load surface are those of the nodeset with label BC_SURF:
    std::vector<std::shared_ptr<ChNodeFEAbase> > nodes_left_surf;
    std::vector<std::shared_ptr<ChNodeFEAbase> > nodes_back_surf;
    std::vector<std::shared_ptr<ChNodeFEAbase> > nodes_bottom_surf;
    //
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > nodes_right_surf;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > nodes_front_surf;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > nodes_top_surf;
    //
    std::shared_ptr<ChNodeFEAxyzrot> center_node;
    //
    //
    // Select primary surfaces 
    //
    //
    auto MeshNodeList=mmesh->GetNodes();
    std::shared_ptr<ChNodeFEAxyzrot> node_v1;
    std::shared_ptr<ChNodeFEAxyzrot> node_v2;
    std::shared_ptr<ChNodeFEAxyzrot> node_v5;
    std::shared_ptr<ChNodeFEAxyzrot> node_v3, node_v4, node_v6, node_v7, node_v8;
    std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAxyzrot>>> cube_faces;
    for (int i=0; i< MeshNodeList.size(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(MeshNodeList[i]);
    	ChVector3d p = node->GetPos();  
    	//
    	// Left surface ----> minusX
    	//    
    	if(abs(p.x()-x_min)<max_dist ){
    		//std::cout<<"node i="<<i<<"  p: "<<p<<std::endl;
    		nodes_left_surf.push_back(node);   
    		if (abs(p.y()-y_min)<max_dist &&  abs(p.z()-z_min)<max_dist )
    			node_v1= node;
    		if (abs(p.z()-z_max)<max_dist && abs(p.y()-y_min)<max_dist)
    			node_v5= node;	
    		if (abs(p.y()-y_max)<max_dist && abs(p.z()-z_min)<max_dist)
    			node_v4= node;
    		if (abs(p.y()-y_max)<max_dist && abs(p.z()-z_max)<max_dist)
    			node_v8= node;	
    	}  
    	
    	//
    	// Rear surface ----> minusZ
    	//    
    	if(abs(p.z()-z_min)<max_dist){
    		//std::cout<<"node i="<<i<<"  p: "<<p<<std::endl;
    		nodes_back_surf.push_back(node);
    		if (abs(p.x()-x_max)<max_dist && abs(p.y()-y_min)<max_dist){
    			node_v2= node;
    		}
    		
    		if (abs(p.x()-x_max)<max_dist && abs(p.y()-y_max)<max_dist)
    			node_v3= node;
    	}  
    	
    	//
    	// Bottom surface ----> minusY
    	//     	
    	if(abs(p.y()-y_min)<max_dist){
    		//std::cout<<"node i="<<i<<"  p: "<<p<<std::endl;
    		nodes_bottom_surf.push_back(node);
    		if (abs(p.x()-x_max)<max_dist && abs(p.z()-z_max)<max_dist)
    			node_v6= node;
    	}   
    	
    	
    	if (abs(p.x()-x_max)<max_dist && abs(p.y()-y_max)<max_dist && abs(p.z()-z_max)<max_dist)
    			node_v7= node;
    	
    }
    
    
	if (node_v1) {
        node_v1->SetFixed(true);	        

    }else{
        std::cout << "ERROR: mesh is not compatible with RVE definitions, try another mesh with points on outer surface! "  << "\n";
		exit(1);
    }
		
   
    //auto constr_v2=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);	
    auto constr_v2=chrono_types::make_shared<ChLinkNodeNodeRot>();	
    	constr_v2->Initialize(node_v1, node_v2);     
    	msystem.Add(constr_v2);
    
    //auto constr_v5=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);
    auto constr_v5=chrono_types::make_shared<ChLinkNodeNodeRot>();		
    	constr_v5->Initialize(node_v1, node_v5);     
    	msystem.Add(constr_v5);
    
    //auto constr_v3=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);
    auto constr_v3=chrono_types::make_shared<ChLinkNodeNodeRot>();		
    	constr_v3->Initialize(node_v1, node_v3);     
    	msystem.Add(constr_v3);
    
    //auto constr_v4=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);	
    auto constr_v4=chrono_types::make_shared<ChLinkNodeNodeRot>();	
    	constr_v4->Initialize(node_v1, node_v4);     
    	msystem.Add(constr_v4);
    
    //auto constr_v6=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);	
    auto constr_v6=chrono_types::make_shared<ChLinkNodeNodeRot>();	
    	constr_v6->Initialize(node_v1, node_v6);     
    	msystem.Add(constr_v6);
    
    //auto constr_v7=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);	
    auto constr_v7=chrono_types::make_shared<ChLinkNodeNodeRot>();	
    	constr_v7->Initialize(node_v1, node_v7);     
    	msystem.Add(constr_v7);
    
    //auto constr_v8=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, true, true);
    auto constr_v8=chrono_types::make_shared<ChLinkNodeNodeRot>();		
    	constr_v8->Initialize(node_v1, node_v8);     
    	msystem.Add(constr_v8);
    
    std::vector<std::shared_ptr<ChNodeFEAbase> > vertex_nodes{node_v1, node_v2, node_v3, node_v4, node_v5, node_v6, node_v7, node_v8};
    
    //
    // make a vector from  members of primary surfaces and remove them from total node list
    //
    std::vector<std::shared_ptr<ChNodeFEAbase> > used_nodes=nodes_left_surf;
    used_nodes.insert(used_nodes.end(), nodes_back_surf.begin(), nodes_back_surf.end());
    used_nodes.insert(used_nodes.end(), nodes_bottom_surf.begin(), nodes_bottom_surf.end());
    for (size_t i = 0; i < CubEdges.size(); ++i) {
		auto edgeNodes=edge_sets["edge_" + std::to_string(i+1)];
		used_nodes.insert(used_nodes.end(), edgeNodes.begin(), edgeNodes.end());
    }
    RemoveItemsFromVector(MeshNodeList, used_nodes);
    RemoveItemsFromVector(MeshNodeList, vertex_nodes);
    //
    //
    // Select secondary surfaces 
    //
    // 
   
    for (int i=0; i< MeshNodeList.size(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(MeshNodeList[i]);
    	ChVector3d p = node->GetPos();  
    	//
    	// Left surface ----> minusX
    	//    
    	if(abs(p.x()-x_max)<max_dist ){
    		//std::cout<<"node i="<<i<<"  p: "<<p<<std::endl;
    		nodes_right_surf.push_back(node);     		
    	}  
    	
    	//
    	// Rear surface ----> minusZ
    	//    
    	if(abs(p.z()-z_max)<max_dist){
    		//std::cout<<"node i="<<i<<"  p: "<<p<<std::endl;
    		nodes_front_surf.push_back(node);
    	}  
    	
    	//
    	// Bottom surface ----> minusY
    	//     	
    	if(abs(p.y()-y_max)<max_dist){
    		//std::cout<<"node i="<<i<<"  p: "<<p<<std::endl;
    		nodes_top_surf.push_back(node);
    	}   
    	
    }
    
    std::cout<<" nodes_left_surf "<<nodes_left_surf.size()<<std::endl;
    std::cout<<" nodes_right_surf "<<nodes_right_surf.size()<<std::endl;
    std::cout<<" nodes_back_surf "<<nodes_back_surf.size()<<std::endl;
    std::cout<<" nodes_front_surf "<<nodes_front_surf.size()<<std::endl;
    std::cout<<" nodes_bottom_surf "<<nodes_bottom_surf.size()<<std::endl;
    std::cout<<" nodes_top_surf "<<nodes_top_surf.size()<<std::endl;
    //std::cout << "center_node->GetIndex() : "<<center_node->GetIndex()<<std::endl;
    
        
    
	
    ///
    ///
    //
    /// Left surface normal is -x directions
    //
    auto tetsurface_L = chrono_types::make_shared<ChMeshSurfaceGeneral>();
    mmesh->AddMeshSurface(tetsurface_L);
    //
    /// Front surface normal is -z directions
    //
    auto tetsurface_BACK = chrono_types::make_shared<ChMeshSurfaceGeneral>();
    mmesh->AddMeshSurface(tetsurface_BACK);
    //
    /// Bottom surface normal is -y directions
    //
    auto tetsurface_BTTM = chrono_types::make_shared<ChMeshSurfaceGeneral>();
    mmesh->AddMeshSurface(tetsurface_BTTM);    
  
    
   
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Create surfaces from node sets in order to apply node-surface interaction
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    
    
    tetsurface_L->AddFacesFromNodeSet(nodes_left_surf);
    tetsurface_BACK->AddFacesFromNodeSet(nodes_back_surf);
    tetsurface_BTTM->AddFacesFromNodeSet(nodes_bottom_surf);
    
	std::string vtkfilename="tetsurface_L";
	
	VisualizeSurfaceMesh(tetsurface_L, vtkfilename);
	
	vtkfilename="tetsurface_BACK";
	VisualizeSurfaceMesh(tetsurface_BACK, vtkfilename);
	
	vtkfilename="tetsurface_BTTM";
	VisualizeSurfaceMesh(tetsurface_BTTM, vtkfilename);
	
	/*
	// Get all faces from the mesh surface
    const auto& faces = tetsurface_L->GetFaces();
	std::vector<ChVector3<int>> Faces;
	std::vector<ChVector3d> vertices;
	std::unordered_map<ChVector3d, int> NodeMap;
    double total_area=0;
	int knode=0;
    for (const auto& face : faces) {
        if (face) {
			auto Lface=std::dynamic_pointer_cast<MyTriangleFace>(face);
            // Access the vertices of the face
            ChVector3d v1 = Lface->GetNode(0)->GetPos();  // First vertex
            ChVector3d v2 = Lface->GetNode(1)->GetPos();  // Second vertex
            ChVector3d v3 = Lface->GetNode(2)->GetPos();  // Third vertex
			
			
			for (int in=0; in<3; in++){
				ChVector3d target=Lface->GetNode(in)->GetPos();
				if (vector_in_vector(vertices, target)) {
					continue;
				} else {
					vertices.push_back(target);			
					NodeMap[target]=knode;
					knode += 1;
				}
		    }
			
			Faces.push_back(ChVector3<int> {NodeMap[v1], NodeMap[v2], NodeMap[v3]});
			std::cout<<"vertex: "<<v1<<"\t"<<v2<<"\t"<<v3<<std::endl;
            // Calculate the area of the triangular face
            auto edge1 = v2 - v1;
            auto edge2 = v3 - v1;
            double area = 0.5 * (edge1.Cross(edge2)).Length();  // Triangle area formula
            total_area += area;
        }
    }
	
	
	std::cout << "\nUsing iterators:" <<NodeMap.size()<< std::endl;
    // Print out the contents of the unordered_map
    for (const auto& pair : NodeMap) {
        std::cout << "Node: (" << pair.first.x() << ", " << pair.first.y() << ", " << pair.first.z() 
                  << ") -> Value: " << pair.second << std::endl;
    }
	
	//exit(0);
	std::string vtkfilename="vtk_surface";
	writeSurfaceMeshtoVtk(vtkfilename, vertices, Faces);
	
	
	std::cout<<"total_area: "<<total_area<<std::endl;
	*/
	
    std::cout<<"tetsurface_L->GetFaces().size() "<<tetsurface_L->GetFaces().size()<<std::endl;
	std::cout<<"tetsurface_Back->GetFaces().size() "<<tetsurface_BACK->GetFaces().size()<<std::endl;
	std::cout<<"tetsurface_BTTM->GetFaces().size() "<<tetsurface_BTTM->GetFaces().size()<<std::endl;
	
	
	
	
    /* 
    tetsurface_L.reset();	 
    tetsurface_L = chrono_types::make_shared<ChMeshSurfaceGeneral>();
	tetsurface_L->AddFacesFromNodeSet(nodes_left_surf);*/
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Apply constraints
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    // Tie node to fem surface
    //   
     
    std::string NS_name_left="SET-CONCRETE-LS";
    double offsetVal=CubeEdgeLength; 
	int const_num=0;
	//
	 
    for (int i = 0; i < nodes_right_surf.size(); ++i) {    	
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(nodes_right_surf[i]);    	
    	bool is_tied=TieNodeToTriFace( msystem, tetsurface_L, node, offsetVal, max_dist); 
    	if (is_tied)
    		++const_num;      	  		
    }
    
    std::cout<<const_num<<" out of "<< nodes_right_surf.size() << " node is attached to left side of core part (tet surface) "<<std::endl;
	
    
    const_num=0;
    for (int i = 0; i < nodes_front_surf.size(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(nodes_front_surf[i]);       		       
    	bool is_tied=TieNodeToTriFace( msystem, tetsurface_BACK, node, offsetVal, max_dist); 
    	if (is_tied)
    		++const_num;       		
    }
    std::cout<<const_num<<" out of "<<nodes_front_surf.size()<< " node is attached to back side of core part (tet surface) "<<std::endl;   
    
    const_num=0;
    for (int i = 0; i < nodes_top_surf.size(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(nodes_top_surf[i]);     	      
    	bool is_tied=TieNodeToTriFace( msystem, tetsurface_BTTM, node, offsetVal, max_dist); 
    	if (is_tied)
    		++const_num;       		
    }
    std::cout<<const_num<<" out of "<<nodes_top_surf.size()<< " node is attached to top side of core part (tet surface) "<<std::endl;
    
    ///////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Constraint for a inner node
    ///
    //////////////////////////////////////////////////////////////////////////////////////
    auto mtruss = chrono_types::make_shared<ChBody>();
    mtruss->SetFixed(true);  
    mtruss->EnableCollision(false);	
    msystem.Add(mtruss);
    
    double xmean=(x_max+x_min)/2;
    double ymean=(y_max+y_min)/2;
    double zmean=(z_max+z_min)/2;
    double lenrange=CubeEdgeLength/5;
    auto fixed_node=chrono_types::make_shared<ChNodeFEAxyzrot>();
    for (int i=0; i< MeshNodeList.size(); ++i) {
    	auto node=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(MeshNodeList[i]);
    	ChVector3d p = node->GetPos();  
    	//
    	// Left surface ----> minusX
    	//        	
    	if( p.x()>(xmean-lenrange) && p.x()<(xmean+lenrange) ){
    		if( p.y()>(ymean-lenrange) && p.y()<(ymean+lenrange) )
    			if( p.z()>(zmean-lenrange) && p.z()<(zmean+lenrange) ) {   		
    				//node->SetFixed(true); 
    				/*auto constr_inner_node=chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, false, false, false);    	
    				constr_inner_node->Initialize(node, mtruss, false, node->Frame(), node->Frame());     
    				msystem.Add(constr_inner_node);*/  
    				//std::cout<<"Restricted node inside volume: "<<p<<std::endl;   								
    				break;    
    				}		
    	}
    }  
    
    //fixed_node->SetFixed(true);
  
}

void ChRepresentedVolumeElement::Solve() {    

    // Solve the system
    if(isDynamic){
    	msystem.DoStepDynamics(timestep);  // Small time step for static simulation        		
    }else{    	
    	msystem.DoStaticAnalysis(static_analysis);     	   	  
    	//msystem.DoStaticNonlinear(100, true);    
    }
    // Compute macro-stress (custom logic to accumulate stress from all elements)
    ComputeMacroStress();
}

void ChRepresentedVolumeElement::ComputeMacroStress() {
        macro_stress->setZero();  // Reset to zero
		#pragma omp parallel for 
		for (int i = 0; i < mmesh->GetNumElements(); ++i) {
			auto elem=mmesh->GetElement(i);
            //
            // Wood elements
            //
            			
            if( auto connector_ptr = std::dynamic_pointer_cast<ChElementCBLCON>(elem) ) {
				#pragma omp critical
				(*macro_stress) += connector_ptr->ComputeMacroStressContribution();			
			}else if ( auto beam_ptr = std::dynamic_pointer_cast<ChElementCurvilinearBeamBezier>(elem) ){
				#pragma omp critical
				(*macro_stress) += beam_ptr->ComputeMacroStressContribution();	
			}else{
				throw std::runtime_error("There are RVE incompatible elements in the mesh");
			}
			
            //std::cout<<" macro_stress: "<<	*macro_stress<<std::endl;
            //
            // CSL ElEMENTS
            //
            
            // to do
            
            //
            // Connector ElEMENTS
            //
            
            // to do
            
            
            //
            // Bezier Beam ElEMENTS
            //
            
            // to do
            
        }     
        
		*macro_stress = *macro_stress * (pow(CubeEdgeLength, -3));   
         	
		       
}


}
}


