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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono_wood/ChBuilderCurvilinearBeam.h"
#include "chrono_wood/ChLineBezierCBL.h"



namespace chrono {
namespace wood {


// ------------------------------------------------------------------
// ChBuilderCurvilinearBeamIGA
// ------------------------------------------------------------------

// Function to parse the knot vectors from the input stream
std::vector<double> parseKnotVectors(std::istringstream& iss) {
    std::vector<double> knotVector;
    double knot;
    while (iss >> knot) {
        knotVector.push_back(knot);
    }
    return knotVector;
}

//
std::vector<int> parseControlPtsVector(std::istringstream& iss) {
    std::vector<int> pointsVector;
    double pts;
    char comma; // To store the comma character
    while (iss >> pts) {
        pointsVector.push_back(pts-1);
        // Check if the next character is a comma
        if (!(iss >> comma && comma == ',')) {
            // If it's not a comma, put it back to the stream and exit the loop
            iss.clear(); // Clear any error flags that may have been set
            iss.putback(comma);
            continue;
        }
        
       
    }
    return pointsVector;
}


/*
struct SectionData {
    int nwings;
    int ridge1;
    int ridge2;
    int ridge3;
    int ridge4;
    int vert1;
    int vert2;
    int vert3;
    int vert4;
    double x;
    double y;    
    double L1;
    double L2;
    double L3;
    double L4;
    double W1;
    double W2;
    double W3;
    double W4;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
};
*/

struct beam_wing {
    double ridge;
    double vert;
    double height;
    double width;
    double angle;   
};

struct SectionData {
    int nwings;
    double x;
    double y; 
    std::vector<beam_wing> wings;
};

std::istream& operator>>(std::istream& is, beam_wing& bw) {
    // Read each member of the struct from the input stream
    is >> bw.ridge >> bw.vert >> bw.height >> bw.width >> bw.angle;
    return is;
}

std::ostream& operator<<(std::ostream& os, const beam_wing& bw) {
    // Output each member of the struct to the output stream
    os << bw.ridge << " ";
    os << bw.vert << " ";
    os << bw.height << " ";
    os << bw.width << " ";  
    os << bw.angle;  
    return os;
}



void ChBuilderCurvilinearBeamIGA::BuildBeam(std::shared_ptr<ChMesh> mesh,                 // mesh to store the resulting elements
                                 std::shared_ptr<CurvedBeamSection> sect,  // section material for beam elements
                                 const int N,                                  // number of elements in the segment
                                 const ChVector3d A,                           // starting point
                                 const ChVector3d B,                           // ending point
                                 const ChVector3d Ydir,                        // the 'up' Y direction of the beam
                                 const int order,                              // the order of spline (default=3,cubic)
				 spline_type curvetype
) {
    beam_elems.clear();
    beam_nodes.clear();

    // rotation of all nodes
    ChMatrix33<> mrot;
    mrot.SetFromAxisX(B - A, Ydir);

    int p = order;

    // Create the 'complete' knot vector, with multiple at the ends
    ChVectorDynamic<> myknots(N + p + p + 1);	
    ChBasisToolsBSpline::ComputeKnotUniformMultipleEnds(myknots, p, 0.0, 1.0);
		

    // Create the 'complete' stl vector of control points, with uniform distribution
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes;
    for (int i_node = 0; i_node < N + p; ++i_node) {
        double abscyssa = ((double)i_node / (double)(N + p - 1));

        // position of node
        ChVector3d pos = A + (B - A) * abscyssa;

        auto hnode_i = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(pos, mrot));
        mesh->AddNode(hnode_i);
        mynodes.push_back(hnode_i);
        this->beam_nodes.push_back(hnode_i);
    }

    // Create the single elements by picking a subset of the nodes and control points
    for (int i_el = 0; i_el < N; ++i_el) {
        std::vector<double> my_el_knots;
        for (int i_el_knot = 0; i_el_knot < p + p + 1 + 1; ++i_el_knot) {
            my_el_knots.push_back(myknots(i_el + i_el_knot));
        }

        std::vector<std::shared_ptr<ChNodeFEAxyzrot>> my_el_nodes;
        for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
            my_el_nodes.push_back(mynodes[i_el + i_el_node]);
        }
		
		if (curvetype==0){		
			auto belement_i = chrono_types::make_shared<ChElementCurvilinearBeamIGA>();
			belement_i->SetNodesGenericOrder(my_el_nodes, my_el_knots, p);
			belement_i->SetSection(sect);
			mesh->AddElement(belement_i);
			this->beam_elems.push_back(belement_i);
		}else{
			
			// Weights of control points are considered as 1 for all nodes  
			std::vector<double> my_el_weights;
			for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
				my_el_weights.push_back(1.);
			}
			
			auto belement_i = chrono_types::make_shared<ChElementCurvilinearBeamIGA>();
			belement_i->SetNodesGenericOrder(my_el_nodes, my_el_weights, my_el_knots, p);
			belement_i->SetSection(sect);
			mesh->AddElement(belement_i);
			this->beam_elems.push_back(belement_i);
		}
			
    }
}

void ChBuilderCurvilinearBeamIGA::BuildBeam(std::shared_ptr<ChMesh> mesh,                 // mesh to store the resulting elements
                                 std::shared_ptr<CurvedBeamSection> sect,  // section material for beam elements
                                 ChLineBSpline& spline,  // the B-spline to be used as the centerline
                                 const ChVector3d Ydir             // the 'up' Y direction of the beam
) {
    beam_elems.clear();
    beam_nodes.clear();

    int p = spline.GetOrder();

    // compute N of spans (excluding start and end multiple knots with zero lenght span):
    int N = (int)spline.Knots().size() - p - p - 1;  // = n+p+1 -p-p-1 = n-p

    // Create the 'complete' stl vector of control points, with uniform distribution
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes;
    for (int i_node = 0; i_node < spline.Points().size(); ++i_node) {
        double abscyssa = ((double)i_node / (double)(spline.Points().size() - 1));

        // position of node
        ChVector3d pos = spline.Points()[i_node];

        // rotation of node, x aligned to tangent at input spline
        ChMatrix33<> mrot;
        ChVector3d tangent=spline.GetTangent(abscyssa);
        mrot.SetFromAxisX(tangent, Ydir);
		
        auto hnode_i = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(pos, mrot));
        mesh->AddNode(hnode_i);
        mynodes.push_back(hnode_i);
        this->beam_nodes.push_back(hnode_i);
    }

    // Create the single elements by picking a subset of the nodes and control points
    for (int i_el = 0; i_el < N; ++i_el) {
        std::vector<double> my_el_knots;
        for (int i_el_knot = 0; i_el_knot < p + p + 1 + 1; ++i_el_knot) {
            my_el_knots.push_back(spline.Knots()(i_el + i_el_knot));
        }

        std::vector<std::shared_ptr<ChNodeFEAxyzrot>> my_el_nodes;
        for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
            my_el_nodes.push_back(mynodes[i_el + i_el_node]);
        }	
		

        auto belement_i = chrono_types::make_shared<ChElementCurvilinearBeamIGA>();
        belement_i->SetNodesGenericOrder(my_el_nodes, my_el_knots, p);
        belement_i->SetSection(sect);
        mesh->AddElement(belement_i);
        this->beam_elems.push_back(belement_i);
    }
}


/*
void ChBuilderCurvilinearBeamIGA::BuildBeam(std::shared_ptr<ChMesh> mesh,                 // mesh to store the resulting elements
                                 std::shared_ptr<CurvedBeamSection> sect,  // section material for beam elements
                                 ChLineBezierCBL& bezier,  // the B-spline to be used as the centerline
                                 const ChVector3d Ydir             // the 'up' Y direction of the beam
) {
    beam_elems.clear();
    beam_nodes.clear();

    int p = bezier.GetOrder();

    // compute N of spans (excluding start and end multiple knots with zero lenght span):
    int N = (int)bezier.Knots().size() - p - p - 1;  // = n+p+1 -p-p-1 = n-p

    // Create the 'complete' stl vector of control points, with uniform distribution
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes;
    for (int i_node = 0; i_node < Bezier.Points().size(); ++i_node) {
        double abscyssa = ((double)i_node / (double)(Bezier.Points().size() - 1));

        // position of node
        ChVector3d pos = Bezier.Points()[i_node];

        // rotation of node, x aligned to tangent at input Bezier
        ChMatrix33<> mrot;
        ChVector3d tangent;
        Bezier.Derive(tangent, abscyssa);
        mrot.SetFromAxisX(tangent, Ydir);
		
        auto hnode_i = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(pos, mrot));
        mesh->AddNode(hnode_i);
        mynodes.push_back(hnode_i);
        this->beam_nodes.push_back(hnode_i);
    }

    // Create the single elements by picking a subset of the nodes and control points
    for (int i_el = 0; i_el < N; ++i_el) {
        std::vector<double> my_el_knots;
        for (int i_el_knot = 0; i_el_knot < p + p + 1 + 1; ++i_el_knot) {
            my_el_knots.push_back(Bezier.Knots()(i_el + i_el_knot));
        }

        std::vector<std::shared_ptr<ChNodeFEAxyzrot>> my_el_nodes;
        for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
            my_el_nodes.push_back(mynodes[i_el + i_el_node]);
        }	
		

        auto belement_i = chrono_types::make_shared<ChElementCurvilinearBeamIGA>();
        belement_i->SetNodesGenericOrder(my_el_nodes, my_el_knots, p);
        belement_i->SetSection(sect);
        mesh->AddElement(belement_i);
        this->beam_elems.push_back(belement_i);
    }
}
*/



void ChBuilderCurvilinearBeamIGA::BuildBeam(std::shared_ptr<ChMesh> mesh,                 // mesh to store the resulting elements
                                 std::shared_ptr<CurvedBeamSection> sect,  // section material for beam elements
                                 ChLineNurbs& Nurbs,  // the B-spline to be used as the centerline
                                 const ChVector3d Ydir             // the 'up' Y direction of the beam
) {
    beam_elems.clear();
    beam_nodes.clear();

    int p = Nurbs.GetOrder();

    // compute N of spans (excluding start and end multiple knots with zero lenght span):
    int N = (int)Nurbs.Knots().size() - p - p - 1;  // = n+p+1 -p-p-1 = n-p

    // Create the 'complete' stl vector of control points, with uniform distribution
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes;
    for (int i_node = 0; i_node < Nurbs.Points().size(); ++i_node) {
        double abscyssa = ((double)i_node / (double)(Nurbs.Points().size() - 1));	
	
        // position of node
        ChVector3d pos = Nurbs.Points()[i_node];	
        // rotation of node, x aligned to tangent at input spline
        ChMatrix33<> mrot;
        ChVector3d tangent=Nurbs.GetTangent(abscyssa);        
        //   
        tangent={abs(tangent[0]),abs(tangent[1]), abs(tangent[2])};    	
    	mrot.SetFromAxisX(tangent, Ydir);   
    	//mrot=QUNIT;       
	//
        auto hnode_i = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(pos, mrot));
        mesh->AddNode(hnode_i);
        mynodes.push_back(hnode_i);
        this->beam_nodes.push_back(hnode_i);
    }

    // Create the single elements by picking a subset of the nodes and control points
    for (int i_el = 0; i_el < N; ++i_el) {
        std::vector<double> my_el_knots;
        for (int i_el_knot = 0; i_el_knot < p + p + 1 + 1; ++i_el_knot) {
            my_el_knots.push_back(Nurbs.Knots()(i_el + i_el_knot));
        }

        std::vector<std::shared_ptr<ChNodeFEAxyzrot>> my_el_nodes;
        for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
            my_el_nodes.push_back(mynodes[i_el + i_el_node]);
        }
		
	std::vector<double> my_el_weights;
        for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
            my_el_weights.push_back(Nurbs.Weights()(i_el + i_el_node));
        }

        auto belement_i = chrono_types::make_shared<ChElementCurvilinearBeamIGA>();
        belement_i->SetNodesGenericOrder(my_el_nodes, my_el_weights, my_el_knots, p);
        belement_i->SetSection(sect);
        mesh->AddElement(belement_i);
        this->beam_elems.push_back(belement_i);
    }
        
}




void ChBuilderCurvilinearBeamIGA::BuildBeam(std::shared_ptr<ChMesh> mesh,                 // mesh to store the resulting elements
                                 std::shared_ptr<CurvedBeamSection> sect,  // section material for beam elements
                                 const int p, 				   // order of spline curve
                                 std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes,  // Control points list belongs to this patch
                                 std::vector<double> knots,  // the knot vector of a patch of B-spline or Nurbs curve
                                 std::vector<double> weights,  // the weight vector of a patch of a Nurbs curve
                                 const ChVector3d Ydir             // the 'up' Y direction of the beam
) {
    //beam_elems.clear();
    //beam_nodes.clear();  
    
    
    // compute N of spans (excluding start and end multiple knots with zero lenght span):
    int N = (int)knots.size() - p - p - 1;  // = n+p+1 -p-p-1 = n-p
    std::vector<ChVector3d > my_points;
    for (auto node : mynodes){
    	my_points.push_back(node->GetPos());
    }
    
    
    // Convert std::vector<double> to chrono::ChVectorDynamic<double>
    ChVectorDynamic<> my_weights(weights.size());
    for (int i = 0; i < weights.size(); ++i) {
        my_weights(i) = weights[i];
    }
    
    ChLineNurbs my_nurbs(p, my_points);
    my_nurbs.weights=my_weights;
    
    
    // Create the 'complete' stl vector of control points, with uniform distribution    
    for (int i_node = 0; i_node < mynodes.size(); ++i_node) {
        double abscyssa = ((double)i_node / (double)(mynodes.size() - 1));	
	
        
        // rotation of node, x aligned to tangent at input spline
        ChMatrix33<> mrot;
        ChVector3d tangent=my_nurbs.GetTangent(abscyssa);        
        //   
        tangent={abs(tangent[0]),abs(tangent[1]), abs(tangent[2])};    	
    	mrot.SetFromAxisX(tangent, Ydir);   
    	//mrot=QUNIT;       
	//
	mynodes[i_node]->SetRot(mrot);     
    }
    
    
    // Create the single elements by picking a subset of the nodes and control points
    for (int i_el = 0; i_el < N; ++i_el) {
        std::vector<double> my_el_knots;
        for (int i_el_knot = 0; i_el_knot < p + p + 1 + 1; ++i_el_knot) {
            my_el_knots.push_back(knots[i_el + i_el_knot]);
        }

        std::vector<std::shared_ptr<ChNodeFEAxyzrot>> my_el_nodes;
        for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
            my_el_nodes.push_back(mynodes[i_el + i_el_node]);
        }
		
	std::vector<double> my_el_weights;
        for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
            my_el_weights.push_back(weights[i_el + i_el_node]);
        }
	
	
        auto belement_i = chrono_types::make_shared<ChElementCurvilinearBeamIGA>();
        belement_i->SetNodesGenericOrder(my_el_nodes, my_el_weights, my_el_knots, p);
        belement_i->SetSection(sect);
        mesh->AddElement(belement_i);        
        //this->beam_elems.push_back(belement_i);
    }
}




void ChBuilderCurvilinearBeamIGA::read_CBL_info(std::shared_ptr<ChMesh> my_mesh,   std::shared_ptr<CurvedBeamSection> msection, std::string& CBL_data_path, 
				std::string& CBL_GeoName){
	//		
	//
	//
	//
	std::string nodesFilename=CBL_data_path+CBL_GeoName+"-chronoNodes.dat";
    	std::string elemFilename=CBL_data_path+CBL_GeoName+"-chronoElements.dat";   
    	std::string patchFilename=CBL_data_path+CBL_GeoName+"-chronoIGA.dat"; 	
    	//
	std::ifstream nodesFile(nodesFilename);
        std::ifstream elemFile(elemFilename);
        std::ifstream patchFile(patchFilename);
	
    ////////////////////////////////////////////////////////////////////////
    //
    // Read node info from Freecad produced "<projectname>-chronoNodes.dat" file
	//          ( x, y, z coordinates )
	// And create node instances using ChNodeFEAxyzrot class. 
	// 			( node with 6 deegrees of freedom)
	//
	////////////////////////////////////////////////////////////////////////
	std::vector< std::shared_ptr<ChNodeFEAxyzrot> > CBLnodeList;	
	auto dummy_mesh = chrono_types::make_shared<ChMesh>();
    if (nodesFile.is_open()) {
       	ChVector3d pos;
       	double x, y, z;
       	std::string dummy_var;
       	unsigned int idnode=0;
		std::string line;
		
		while (std::getline(nodesFile, line)) {			
			//
			if( line[0]!='/') {				
				std::istringstream sline(line);
				sline>> x >> dummy_var >> y >> dummy_var >> z;				
				//
				auto mnode= chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(x, y, z)));
				mnode->SetIndex(idnode);
				CBLnodeList.push_back(mnode);
                		dummy_mesh->AddNode(mnode);								
                		++idnode;
			}
		}
    }
    else{
    	throw std::invalid_argument("ERROR opening nodes info file: " + std::string(nodesFilename) + "\n");
    	exit(EXIT_FAILURE);
    }
    
    
    for(int i=0; i<dummy_mesh->GetNumNodes(); i++){
    
    	auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(dummy_mesh->GetNode(i));
    	//std::cout<< i <<". node: "<<node->Frame().GetPos().x()<<"\t"<<node->Frame().GetPos().y()<<"\t"<<node->Frame().GetPos().z()<<std::endl;
    
    }
	

	/*
	auto msection = chrono_types::make_shared<ChSectionCBL>();
	msection->Set_material(vect_mat);
	msection->SetDrawThickness(2, 2);
	*/
    ////////////////////////////////////////////////////////////////////////
    //
    // Read patch data
    //         
    // 
    // 			
    //
    ////////////////////////////////////////////////////////////////////////	
	
    // Initialize variables to store extracted information
    int dimension = 0;
    int orderOfBasisFunctions = 0;
    int numControlPointsPerPatch = 0;
    int numElementsPerPatch = 0;
    int numPatches = 0;
    std::vector<std::vector<double>> knotVectors;
    std::vector<std::vector<int>> controlPointsVectors;
    
     if (patchFile.is_open()) {     
    
    //   
    std::string line;
    while (std::getline(patchFile, line)) {
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        if (token == "#") {
            std::string keyword;
            iss >> keyword;
            if (keyword == "Dimension") {
            	std::getline(patchFile, line);
            	std::istringstream iss(line);
                iss >> dimension;
            } else if (keyword == "Order") {
            	std::getline(patchFile, line);
            	std::istringstream iss(line);
                iss >> orderOfBasisFunctions;
            } else if (keyword == "Number") {
                std::string secondWord;
                iss >> secondWord;
                if (secondWord == "of") {
                    std::string thirdWord;
                    iss >> thirdWord;
                    if (thirdWord == "control") {
                    	std::getline(patchFile, line);
            		std::istringstream iss(line);
                        iss >> numControlPointsPerPatch;
                    } else if (thirdWord == "elements") {
                    	std::getline(patchFile, line);
            		std::istringstream iss(line);
                        iss >> numElementsPerPatch;
                    } else if (thirdWord == "Patches") {
                    std::getline(patchFile, line);
            	    std::istringstream iss(line);
                    iss >> numPatches;
                    }
            	}
            }
        } else if (token == "knot") {
            std::getline(patchFile, line);
            std::istringstream iss(line);
            std::vector<double> knotVector = parseKnotVectors(iss);
            knotVectors.push_back(knotVector);
            //
            std::getline(patchFile, line);
            std::istringstream issPts(line);            	
            std::vector<int> controlPoints = parseControlPtsVector(issPts);
            controlPointsVectors.push_back(controlPoints);
        }
    }
    }else{
    
     throw std::invalid_argument("ERROR opening patches info file: " + std::string(patchFilename) + "\n");
    	exit(1);        
    }
    
    
    
    

    // Print the extracted information
    //std::cout << "Dimension: " << dimension << std::endl;
    //std::cout << "Order of basis functions: " << orderOfBasisFunctions << std::endl;
    //std::cout << "Number of control points per patch: " << numControlPointsPerPatch << std::endl;
    //std::cout << "Number of elements per patch: " << numElementsPerPatch << std::endl;
    //std::cout << "Number of patches: " << numPatches << std::endl;
    //std::cout << "Knot vectors:\n";
    for (const auto& knotVector : knotVectors) {
        for (const auto& knot : knotVector) {
            std::cout << knot << " ";
        }
        std::cout << std::endl;        
    }
    //std::cout << "Control Points :\n";
    for (const auto& controlPtsVector : controlPointsVectors) {
        for (const auto& pts : controlPtsVector) {
            std::cout << pts << " ";
        }
        std::cout << std::endl;        
    }

    // Close the input file
    patchFile.close();
    //
    // Nurbs curve based creation of element
    //
    	
    ChBuilderCurvilinearBeamIGA builderR;    
    for (int ip=0; ip<numPatches; ip++){
    	//std::cout<<"Patch no: "<<ip<<std::endl;
    	//std::cout<<"numControlPointsPerPatch: "<<numControlPointsPerPatch<<"\t orderOfBasisFunctions: "<<orderOfBasisFunctions<<std::endl;
    	auto pts=controlPointsVectors[ip];
    	std::vector<ChVector3d> my_points;
    	//std::vector<std::shared_ptr<ChNodeFEAxyzrot>> my_points;
    	/*std::cout<<"pts: "<<pts.size()<<std::endl;
    	for (int ipt=0; ipt<pts.size(); ipt++){
    		std::cout<<pts[ipt]<<"\t";
    	}
    	std::cout<<std::endl;*/
    	int centerP=numControlPointsPerPatch/2;
    	for (int ind=0; ind<pts.size(); ind++){
    	    if (ind==centerP)
    	    	continue;
    	    auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(dummy_mesh->GetNode(pts[ind]));
    	    my_points.push_back(mynode->Frame().GetPos());
    		
    	}
    	
    	/*std::cout<<"my_points: "<<my_points.size()<<std::endl;    
    		
    	for (int ipp=0; ipp<my_points.size(); ipp++){
    		std::cout<<my_points[ipp].x()<<"\t"<<my_points[ipp].y()<<"\t"<<my_points[ipp].z()<<"\n";
    	}
    	*/
    	//exit(10);
    	ChVectorDynamic<> my_weight(numControlPointsPerPatch-1);	
	my_weight.setConstant(1.0);
    	ChLineNurbs my_nurbs(orderOfBasisFunctions,           // order (3 = cubic, etc)
                                      my_points);  // control points, will become the IGA nodes
	
	my_nurbs.weights=my_weight;	
	
	//std::cout<<"order : "<<my_nurbs.GetOrder()<<std::endl;
	//std::cout<<"knot:\n"<<my_nurbs.Knots()<<std::endl;
	//std::cout<<"weights:\n"<<my_nurbs.Weights()<<std::endl;
	
    	builderR.BuildBeam(my_mesh,    // the mesh to put the elements in
                       msection,   // section of the beam
                       my_nurbs,  // NURBS to match (also order will be matched)
                       VECT_Z);    // suggested Y direction of section
    }
     /*  
    //
    // Node based creation of element
    //
    
    ChBuilderCurvilinearBeamIGA builderR;
    for (int ip=0; ip<numPatches; ip++){
    	std::cout<<"Patch no: "<<ip<<std::endl;
    	std::cout<<"numControlPointsPerPatch: "<<numControlPointsPerPatch<<"\t orderOfBasisFunctions: "<<orderOfBasisFunctions<<std::endl;
    	auto pts=controlPointsVectors[ip];
    	std::vector<std::shared_ptr<ChNodeFEAxyzrot>> my_nodes;
    	//std::vector<std::shared_ptr<ChNodeFEAxyzrot>> my_points;
    	std::cout<<"pts: "<<pts.size()<<std::endl;
    	for (int ipt=0; ipt<pts.size(); ipt++){
    		std::cout<<pts[ipt]<<"\t";
    	}
    	std::cout<<std::endl;
    	int centerP=numControlPointsPerPatch/2;
    	for (int ind=0; ind<pts.size(); ind++){
    	    if (ind==centerP)
    	    	continue;
    	    auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(dummy_mesh->GetNode(pts[ind]));
    	    my_nodes.push_back(mynode);
    		
    	}
    	
    	
    	
    	
    	std::vector<double> knots=knotVectors[ip];
    	std::vector<double> my_weight= {1.0, 1.0, 1.0, 1.0};	
		
    	
	
	
	
    	builderR.BuildBeam( my_mesh,                 		// mesh to store the resulting elements
                                 msection,  		// section material for beam elements
                                 orderOfBasisFunctions, 				   		// order of spline curve
                                 my_nodes, // Control points list belongs to this patch
                                 knots,  				// the knot vector of a patch of B-spline or Nurbs curve
                                 my_weight,  				// the weight vector of a patch of a Nurbs curve
                                 VECT_Z);             			// the 'up' Y direction of the beam
    }
    */
    ////////////////////////////////////////////////////////////////////////
    //
    // Read element connectivity info from Freecad produced "<projectname>-chronoElements.dat" file
	//          ( node1, node2, ....,nodeN ) where N=p+1 and p shows degree of the curve
	// And create element instances using ChElementCurvilinearBeamIGA class. 
	// 			( CBL beam element with )
	//
	////////////////////////////////////////////////////////////////////////
    /*if (elemFile.is_open()) {       	
       	int elid;
       	int node1, node2, node3, node4;
       	unsigned int idelem=0;
		std::string line;
    	while (std::getline(elemFile, line)) {
			//
			if( line[0]!='/') {				
				std::istringstream sline(line);
				sline>> node1 >> node2 >> node3 >> node4;				
				auto nodeI = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(node1));
				auto nodeJ = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(node2));
				auto nodeK = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(node3));
				auto nodeL = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(node4));
				auto mel = chrono_types::make_shared<ChElementCurvilinearBeamIGA>();
				//mel->SetNodes(nodeI, nodeJ, nodeK, nodeL);
				mel->SetSection(msection);
				my_mesh->AddElement(mel);
                ++idelem;

			}
		}
    }
    else{
    	throw std::invalid_argument("ERROR opening element info file: " + std::string(elemFilename) + "\n");
    	exit(EXIT_FAILURE);
    }
	
	int elementnum=my_mesh->GetNelements();
	std::cout<<"elementnum "<<elementnum<<std::endl;*/
	
	/*
	for(int i=0; i<my_mesh->GetNelements(); i++){    
	    	auto iel = std::dynamic_pointer_cast<ChElementCBL>(my_mesh->GetElement(i));
	    	auto nI=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(iel->GetNodeN(0));
	    	auto nJ=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(iel->GetNodeN(1));
	    	auto nK=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(iel->GetNodeN(2));
	    	auto nL=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(iel->GetNodeN(3));
	    	std::cout<< i <<". element:  ";
	    	std::cout<<nI->Frame().GetPos().x()<<"\t"<<nI->Frame().GetPos().y()<<"\t"<<nI->Frame().GetPos().z()<<"\t";
	    	std::cout<<nJ->Frame().GetPos().x()<<"\t"<<nJ->Frame().GetPos().y()<<"\t"<<nJ->Frame().GetPos().z()<<"\t";
	    	std::cout<<nK->Frame().GetPos().x()<<"\t"<<nK->Frame().GetPos().y()<<"\t"<<nK->Frame().GetPos().z()<<"\t";
	    	std::cout<<nL->Frame().GetPos().x()<<"\t"<<nL->Frame().GetPos().y()<<"\t"<<nL->Frame().GetPos().z()<<std::endl;
    
    	}
	*/	
    
	
}




void ChBuilderCurvilinearBeamIGA::read_CBL_info_ElBased(std::shared_ptr<ChMesh> my_mesh,   std::shared_ptr<CurvedBeamSection> msection, std::string& CBL_data_path, 
				std::string& CBL_GeoName){
	//	
	//
	//
	//
	std::string nodesFilename=CBL_data_path+CBL_GeoName+"-chronoNodes.dat";
    	std::string elemFilename=CBL_data_path+CBL_GeoName+"-chronoElements.dat";   
    	std::string patchFilename=CBL_data_path+CBL_GeoName+"-chronoIGA.dat"; 	
    	//
	std::ifstream nodesFile(nodesFilename);
        std::ifstream elemFile(elemFilename);
        std::ifstream patchFile(patchFilename);
	
    ////////////////////////////////////////////////////////////////////////
    //
    // Read node info from Freecad produced "<projectname>-chronoNodes.dat" file
	//          ( x, y, z coordinates )
	// And create node instances using ChNodeFEAxyzrot class. 
	// 			( node with 6 deegrees of freedom)
	//
	////////////////////////////////////////////////////////////////////////
	std::vector< std::shared_ptr<ChNodeFEAxyzrot> > CBLnodeList;	
    if (nodesFile.is_open()) {
       	ChVector3d pos;
       	double x, y, z;
       	std::string dummy_var;
       	unsigned int idnode=0;
		std::string line;
		
		while (std::getline(nodesFile, line)) {			
			//
			if( line[0]!='/') {				
				std::istringstream sline(line);
				sline>> x >> dummy_var >> y >> dummy_var >> z;				
				//
				auto mnode= chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(x, y, z),QUNIT));
				mnode->SetIndex(idnode);
				CBLnodeList.push_back(mnode);
                		my_mesh->AddNode(mnode);								
                		++idnode;
			}
		}
    }
    else{
    	throw std::invalid_argument("ERROR opening nodes info file: " + std::string(nodesFilename) + "\n");
    	exit(EXIT_FAILURE);
    }
    
    /*
    for(int i=0; i<my_mesh->GetNumNodes(); i++){
    
    	auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(i));
    	std::cout<< i <<". node: "<<node->Frame().GetPos().x()<<"\t"<<node->Frame().GetPos().y()<<"\t"<<node->Frame().GetPos().z()<<std::endl;
    
    }
	*/

	/*
	auto msection = chrono_types::make_shared<ChSectionCBL>();
	msection->Set_material(vect_mat);
	msection->SetDrawThickness(2, 2);
	*/
    ////////////////////////////////////////////////////////////////////////
    //
    // Read patch data
    //         
    // 
    // 			
    //
    ////////////////////////////////////////////////////////////////////////	
	
    // Initialize variables to store extracted information
    int dimension = 0;
    int orderOfBasisFunctions = 0;
    int numControlPointsPerPatch = 0;
    int numElementsPerPatch = 0;
    int numPatches = 0;
    std::vector<std::vector<double>> knotVectors;
    std::vector<std::vector<int>> controlPointsVectors;
    
     if (patchFile.is_open()) {     
    
    //   
    std::string line;
    while (std::getline(patchFile, line)) {
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        if (token == "#") {
            std::string keyword;
            iss >> keyword;
            if (keyword == "Dimension") {
            	std::getline(patchFile, line);
            	std::istringstream iss(line);
                iss >> dimension;
            } else if (keyword == "Order") {
            	std::getline(patchFile, line);
            	std::istringstream iss(line);
                iss >> orderOfBasisFunctions;
            } else if (keyword == "Number") {
                std::string secondWord;
                iss >> secondWord;
                if (secondWord == "of") {
                    std::string thirdWord;
                    iss >> thirdWord;
                    if (thirdWord == "control") {
                    	std::getline(patchFile, line);
            		std::istringstream iss(line);
                        iss >> numControlPointsPerPatch;
                    } else if (thirdWord == "elements") {
                    	std::getline(patchFile, line);
            		std::istringstream iss(line);
                        iss >> numElementsPerPatch;
                    } else if (thirdWord == "Patches") {
                    std::getline(patchFile, line);
            	    std::istringstream iss(line);
                    iss >> numPatches;
                    }
            	}
            }
        } else if (token == "knot") {
            std::getline(patchFile, line);
            std::istringstream iss(line);
            std::vector<double> knotVector = parseKnotVectors(iss);
            knotVectors.push_back(knotVector);
            //
            std::getline(patchFile, line);
            std::istringstream issPts(line);            	
            std::vector<int> controlPoints = parseControlPtsVector(issPts);
            controlPointsVectors.push_back(controlPoints);
        }
    }
    }else{
    
     throw std::invalid_argument("ERROR opening patches info file: " + std::string(patchFilename) + "\n");
    	exit(1);        
    }
    
    
    
    

    // Print the extracted information
    std::cout << "Dimension: " << dimension << std::endl;
    std::cout << "Order of basis functions: " << orderOfBasisFunctions << std::endl;
    std::cout << "Number of control points per patch: " << numControlPointsPerPatch << std::endl;
    std::cout << "Number of elements per patch: " << numElementsPerPatch << std::endl;
    std::cout << "Number of patches: " << numPatches << std::endl;
    /*std::cout << "Knot vectors:\n";
    for (const auto& knotVector : knotVectors) {
        for (const auto& knot : knotVector) {
            std::cout << knot << " ";
        }
        std::cout << std::endl;        
    }
    std::cout << "Control Points :\n";
    for (const auto& controlPtsVector : controlPointsVectors) {
        for (const auto& pts : controlPtsVector) {
            std::cout << pts << " ";
        }
        std::cout << std::endl;        
    }
    */
    // Close the input file
    patchFile.close();
    
    //
    // Node based creation of element
    //
    
    ChBuilderCurvilinearBeamIGA builderR;
    for (int ip=0; ip<numPatches; ip++){
    	//std::cout<<"Patch no: "<<ip<<std::endl;
    	//std::cout<<"numControlPointsPerPatch: "<<numControlPointsPerPatch<<"\t orderOfBasisFunctions: "<<orderOfBasisFunctions<<std::endl;
    	auto pts=controlPointsVectors[ip];
    	std::vector<std::shared_ptr<ChNodeFEAxyzrot>> my_nodes;
    	//std::vector<std::shared_ptr<ChNodeFEAxyzrot>> my_points;
    	/*std::cout<<"pts: "<<pts.size()<<std::endl;
    	for (int ipt=0; ipt<pts.size(); ipt++){
    		std::cout<<pts[ipt]<<"\t";
    	}
    	std::cout<<std::endl;*/
    	int centerP=numControlPointsPerPatch/2;
    	for (int ind=0; ind<pts.size(); ind++){
    	    if (ind==centerP)
    	    	continue;
    	    auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(pts[ind]));
    	    my_nodes.push_back(mynode);
    		
    	}
    	
    	
    	
    	
    	std::vector<double> knots=knotVectors[ip];
    	std::vector<double> my_weight(numControlPointsPerPatch-1);
    	std::fill(my_weight.begin(), my_weight.end(), 1.0);		
		
    	
	
	
	
    	builderR.BuildBeam( my_mesh,                 		// mesh to store the resulting elements
                                 msection,  		// section material for beam elements
                                 orderOfBasisFunctions, 				   		// order of spline curve
                                 my_nodes, // Control points list belongs to this patch
                                 knots,  				// the knot vector of a patch of B-spline or Nurbs curve
                                 my_weight,  				// the weight vector of a patch of a Nurbs curve
                                 VECT_Z);             			// the 'up' Y direction of the beam
    }
    
    ////////////////////////////////////////////////////////////////////////
    //
    // Read element connectivity info from Freecad produced "<projectname>-chronoElements.dat" file
	//          ( node1, node2, ....,nodeN ) where N=p+1 and p shows degree of the curve
	// And create element instances using ChElementCurvilinearBeamIGA class. 
	// 			( CBL beam element with )
	//
	////////////////////////////////////////////////////////////////////////
    /*if (elemFile.is_open()) {       	
       	int elid;
       	int node1, node2, node3, node4;
       	unsigned int idelem=0;
		std::string line;
    	while (std::getline(elemFile, line)) {
			//
			if( line[0]!='/') {				
				std::istringstream sline(line);
				sline>> node1 >> node2 >> node3 >> node4;				
				auto nodeI = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(node1));
				auto nodeJ = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(node2));
				auto nodeK = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(node3));
				auto nodeL = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(node4));
				auto mel = chrono_types::make_shared<ChElementCurvilinearBeamIGA>();
				//mel->SetNodes(nodeI, nodeJ, nodeK, nodeL);
				mel->SetSection(msection);
				my_mesh->AddElement(mel);
                ++idelem;

			}
		}
    }
    else{
    	throw std::invalid_argument("ERROR opening element info file: " + std::string(elemFilename) + "\n");
    	exit(EXIT_FAILURE);
    }
	
	int elementnum=my_mesh->GetNelements();
	std::cout<<"elementnum "<<elementnum<<std::endl;*/
	
	/*
	for(int i=0; i<my_mesh->GetNelements(); i++){    
	    	auto iel = std::dynamic_pointer_cast<ChElementCBL>(my_mesh->GetElement(i));
	    	auto nI=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(iel->GetNodeN(0));
	    	auto nJ=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(iel->GetNodeN(1));
	    	auto nK=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(iel->GetNodeN(2));
	    	auto nL=std::dynamic_pointer_cast<ChNodeFEAxyzrot>(iel->GetNodeN(3));
	    	std::cout<< i <<". element:  ";
	    	std::cout<<nI->Frame().GetPos().x()<<"\t"<<nI->Frame().GetPos().y()<<"\t"<<nI->Frame().GetPos().z()<<"\t";
	    	std::cout<<nJ->Frame().GetPos().x()<<"\t"<<nJ->Frame().GetPos().y()<<"\t"<<nJ->Frame().GetPos().z()<<"\t";
	    	std::cout<<nK->Frame().GetPos().x()<<"\t"<<nK->Frame().GetPos().y()<<"\t"<<nK->Frame().GetPos().z()<<"\t";
	    	std::cout<<nL->Frame().GetPos().x()<<"\t"<<nL->Frame().GetPos().y()<<"\t"<<nL->Frame().GetPos().z()<<std::endl;
    
    	}
	*/	
    
	
}



void ChBuilderCurvilinearBeamBezier::BuildBeam(std::shared_ptr<ChMesh> mesh,                 // mesh to store the resulting elements
                                 std::shared_ptr<CurvedBeamSection> sect,  // section material for beam elements
                                 ChLineBezierCBL& bezier,  // the B-spline to be used as the centerline
                                 const ChVector3d Ydir             // the 'up' Y direction of the beam
) {
    beam_elems.clear();
    beam_nodes.clear();

    int p = bezier.GetOrder();

    // compute N of spans (excluding start and end multiple knots with zero lenght span):
    int N = (int)bezier.Knots().size() - p - p - 1;  // = n+p+1 -p-p-1 = n-p

    // Create the 'complete' stl vector of control points, with uniform distribution
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes;
    for (int i_node = 0; i_node < bezier.Points().size(); ++i_node) {
        double abscyssa = ((double)i_node / (double)(bezier.Points().size() - 1));

        // position of node
        ChVector3d pos = bezier.Points()[i_node];        
        // rotation of node, x aligned to tangent at input Bezier
        ChMatrix33<> mrot;
        ChVector3d tangent;
        bezier.Derive(tangent, abscyssa);
        mrot.SetFromAxisX(tangent, Ydir);    
        mrot=QUNIT;    
        auto hnode_i = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(pos, mrot));
        mesh->AddNode(hnode_i);
        mynodes.push_back(hnode_i);
        this->beam_nodes.push_back(hnode_i);
    }	
    // Create the single elements by picking a subset of the nodes and control points
    for (int i_el = 0; i_el < N; ++i_el) {
        std::vector<double> my_el_knots;
        for (int i_el_knot = 0; i_el_knot < p + p + 1 + 1; ++i_el_knot) {
            my_el_knots.push_back(bezier.Knots()(i_el + i_el_knot));
        }

        std::vector<std::shared_ptr<ChNodeFEAxyzrot>> my_el_nodes;
        for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
            my_el_nodes.push_back(mynodes[i_el + i_el_node]);
        }	
		

        auto belement_i = chrono_types::make_shared<ChElementCurvilinearBeamBezier>();
        belement_i->SetNodesGenericOrder(my_el_nodes, my_el_knots, p);
        belement_i->SetSection(sect);
        mesh->AddElement(belement_i);
        this->beam_elems.push_back(belement_i);
    }
    
}




void ChBuilderCurvilinearBeamBezier::BuildBeam(std::shared_ptr<ChMesh> mesh,                 // mesh to store the resulting elements
                                 std::shared_ptr<CurvedBeamSection> sect,  // section material for beam elements
                                 const int p, 				   // order of spline curve
                                 std::vector<ChVector3d> my_points,  // Control points list belongs to this patch
                                 ChVectorDynamic<> knots,  // the knot vector of a patch of B-spline or Nurbs curve                                
                                 const ChVector3d Ydir             // the 'up' Y direction of the beam
) {
    beam_elems.clear();
    beam_nodes.clear();  
    
    
    // compute N of spans (excluding start and end multiple knots with zero lenght span):
    int N = (int)knots.size() - p - p - 1;  // = n+p+1 -p-p-1 = n-p
	
    ChLineBezierCBL my_bezier(p, my_points);
    
    // Create the 'complete' stl vector of control points, with uniform distribution
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes;
    for (int i_node = 0; i_node < my_points.size(); ++i_node) {
        double abscyssa = ((double)i_node / (double)(my_points.size() - 1));

        // position of node
        ChVector3d pos = my_points[i_node];

        // rotation of node, x aligned to tangent at input Bezier
        ChMatrix33<> mrot;
        ChVector3d tangent;
        my_bezier.Derive(tangent, abscyssa);
        mrot.SetFromAxisX(tangent, Ydir);
	mrot=QUNIT;	
        auto hnode_i = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(pos, mrot));
        mesh->AddNode(hnode_i);
        mynodes.push_back(hnode_i);
        this->beam_nodes.push_back(hnode_i);
    }
    
    
    // Create the single elements by picking a subset of the nodes and control points
    for (int i_el = 0; i_el < N; ++i_el) {
        std::vector<double> my_el_knots;
        for (int i_el_knot = 0; i_el_knot < p + p + 1 + 1; ++i_el_knot) {
            my_el_knots.push_back(knots[i_el + i_el_knot]);
        }

        std::vector<std::shared_ptr<ChNodeFEAxyzrot>> my_el_nodes;
        for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
            my_el_nodes.push_back(mynodes[i_el*p + i_el_node]);
        }		
	
	
        auto belement_i = chrono_types::make_shared<ChElementCurvilinearBeamBezier>();
        belement_i->SetNodesGenericOrder(my_el_nodes, my_el_knots, p);
        belement_i->SetSection(sect);
        mesh->AddElement(belement_i);        
        this->beam_elems.push_back(belement_i);
    }
}




void ChBuilderCurvilinearBeamBezier::BuildBeam(std::shared_ptr<ChMesh> mesh,                 // mesh to store the resulting elements
                                 std::shared_ptr<CurvedBeamSection> sect,  // section material for beam elements
                                 const int p, 				   // order of spline curve
                                 std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes,  // Control points list belongs to this patch
                                 std::vector<double> knots,  // the knot vector of a patch of B-spline or Nurbs curve                                
                                 const ChVector3d Ydir             // the 'up' Y direction of the beam
) {
    beam_elems.clear();
    beam_nodes.clear();  
    
    
    // compute N of spans (excluding start and end multiple knots with zero lenght span):
    int N = (int)knots.size() - p - p - 1;  // = n+p+1 -p-p-1 = n-p
    std::vector<ChVector3d > my_points;
    for (auto node : mynodes){
    	my_points.push_back(node->GetPos());
    	this->beam_nodes.push_back(node);
    }
    
    //create bezier curve to find local sytem of reference
    ChLineBezierCBL my_bezier(p, my_points);
    
    
    // Create the 'complete' stl vector of control points, with uniform distribution    
    for (int i_node = 0; i_node < mynodes.size(); ++i_node) {
        double abscyssa = ((double)i_node / (double)(mynodes.size() - 1));	
	
        
        // rotation of node, x aligned to tangent at input spline
        ChMatrix33<> mrot;
        ChVector3d tangent;
        my_bezier.Derive(tangent, abscyssa);        
        //   
        tangent={abs(tangent[0]),abs(tangent[1]), abs(tangent[2])};    	
    	mrot.SetFromAxisX(tangent, Ydir);   
    	mrot=QUNIT;       
	//
	mynodes[i_node]->SetRot(mrot);     
    }
    
    
    // Create the single elements by picking a subset of the nodes and control points
    for (int i_el = 0; i_el < N; ++i_el) {
        std::vector<double> my_el_knots;
        for (int i_el_knot = 0; i_el_knot < p + p + 1 + 1; ++i_el_knot) {
            my_el_knots.push_back(knots[i_el + i_el_knot]);
        }

        std::vector<std::shared_ptr<ChNodeFEAxyzrot>> my_el_nodes;        
        for (int i_el_node = 0; i_el_node < p + 1; ++i_el_node) {
            my_el_nodes.push_back(mynodes[i_el*p + i_el_node]);
        }		
	
	
        auto belement_i = chrono_types::make_shared<ChElementCurvilinearBeamBezier>();
        belement_i->SetNodesGenericOrder(my_el_nodes, my_el_knots, p);
        belement_i->SetSection(sect);
        mesh->AddElement(belement_i);        
        this->beam_elems.push_back(belement_i);
    }
}




void ChBuilderCurvilinearBeamBezier::read_CBL_info_ElBased(std::shared_ptr<ChMesh> my_mesh,   std::string& CBL_data_path, 
				std::string& CBL_GeoName, double YoungModulus,	double ShearModulus, double density){
	//	
	//
	//
	//
	std::string nodesFilename=CBL_data_path+CBL_GeoName+"-chronoNodes.dat";
    	std::string propertyFilename=CBL_data_path+CBL_GeoName+"-vertex.mesh";   
    	std::string patchFilename=CBL_data_path+CBL_GeoName+"-chronoIGA.dat"; 	
    	//
	std::ifstream nodesFile(nodesFilename);
        std::ifstream SectionPropertyFile(propertyFilename);
        std::ifstream patchFile(patchFilename);
	
    ////////////////////////////////////////////////////////////////////////
    //
    // Read node info from Freecad produced "<projectname>-chronoNodes.dat" file
	//          ( x, y, z coordinates )
	// And create node instances using ChNodeFEAxyzrot class. 
	// 			( node with 6 deegrees of freedom)
	//
	////////////////////////////////////////////////////////////////////////
	std::vector< std::shared_ptr<ChNodeFEAxyzrot> > CBLnodeList;	
    if (nodesFile.is_open()) {
       	ChVector3d pos;
       	double x, y, z;
       	std::string dummy_var;
       	unsigned int idnode=0;
		std::string line;
		
		while (std::getline(nodesFile, line)) {			
			//
			if( line[0]!='/') {				
				std::istringstream sline(line);				
				sline>> x >> dummy_var >> y >> dummy_var >> z;	
							
				//
				auto mnode= chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(x, y, z),QUNIT));
				mnode->SetIndex(idnode);
				CBLnodeList.push_back(mnode);
                		my_mesh->AddNode(mnode);								
                		++idnode;
			}
		}
    }
    else{
    	throw std::invalid_argument("ERROR opening nodes info file: " + std::string(nodesFilename) + "\n");
    	exit(EXIT_FAILURE);
    }
    
    /*
    for(int i=0; i<my_mesh->GetNumNodes(); i++){
    
    	auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(i));
    	std::cout<< i <<". node: "<<node->Frame().GetPos().x()<<"\t"<<node->Frame().GetPos().y()<<"\t"<<node->Frame().GetPos().z()<<std::endl;
    
    }
	*/

	/*
	auto msection = chrono_types::make_shared<ChSectionCBL>();
	msection->Set_material(vect_mat);
	msection->SetDrawThickness(2, 2);
	*/
    ////////////////////////////////////////////////////////////////////////
    //
    // Read patch data
    //         
    // 
    // 			
    //
    ////////////////////////////////////////////////////////////////////////	
	
    // Initialize variables to store extracted information
    int dimension = 0;
    int orderOfBasisFunctions = 0;
    int numControlPointsPerPatch = 0;
    int numElementsPerPatch = 0;
    int numPatches = 0;
    std::vector<std::vector<double>> knotVectors;
    std::vector<std::vector<int>> controlPointsVectors;
    
     if (patchFile.is_open()) {     
    
    //   
    std::string line;
    while (std::getline(patchFile, line)) {
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        if (token == "#") {
            std::string keyword;
            iss >> keyword;
            if (keyword == "Dimension") {
            	std::getline(patchFile, line);
            	std::istringstream iss(line);
                iss >> dimension;
            } else if (keyword == "Order") {
            	std::getline(patchFile, line);
            	std::istringstream iss(line);
                iss >> orderOfBasisFunctions;
            } else if (keyword == "Number") {
                std::string secondWord;
                iss >> secondWord;
                if (secondWord == "of") {
                    std::string thirdWord;
                    iss >> thirdWord;
                    if (thirdWord == "control") {
                    	std::getline(patchFile, line);
            		std::istringstream iss(line);
                        iss >> numControlPointsPerPatch;
                    } else if (thirdWord == "elements") {
                    	std::getline(patchFile, line);
            		std::istringstream iss(line);
                        iss >> numElementsPerPatch;
                    } else if (thirdWord == "Patches") {
                    std::getline(patchFile, line);
            	    std::istringstream iss(line);
                    iss >> numPatches;
                    }
            	}
            }
        } else if (token == "knot") {
            std::getline(patchFile, line);
            //std::cout<<"line: "<<line<<std::endl;
            std::istringstream iss(line);
            std::vector<double> knotVector = parseKnotVectors(iss);
            knotVectors.push_back(knotVector);
            //
            std::getline(patchFile, line);
            //std::cout<<"line: "<<line<<std::endl;
            std::istringstream issPts(line);            	
            std::vector<int> controlPoints = parseControlPtsVector(issPts);
            controlPointsVectors.push_back(controlPoints);
        }
    }
    }else{
    
     throw std::invalid_argument("ERROR opening patches info file: " + std::string(patchFilename) + "\n");
    	exit(1);        
    }
    
    
    
    

    // Print the extracted information
    std::cout << "Dimension: " << dimension << std::endl;
    std::cout << "Order of basis functions: " << orderOfBasisFunctions << std::endl;
    std::cout << "Number of control points per patch: " << numControlPointsPerPatch << std::endl;
    std::cout << "Number of elements per patch: " << numElementsPerPatch << std::endl;
    std::cout << "Number of patches: " << numPatches << std::endl;
    /*std::cout << "Knot vectors:\n";
    for (const auto& knotVector : knotVectors) {
        for (const auto& knot : knotVector) {
            std::cout << knot << " ";
        }
        std::cout << std::endl;        
    }
    std::cout << "Control Points :\n";
    for (const auto& controlPtsVector : controlPointsVectors) {
        for (const auto& pts : controlPtsVector) {
            std::cout << pts << " ";
        }
        std::cout << std::endl;        
    }
     */
    // Close the input file
    patchFile.close();
    
    ////////////////////////////////////////////////////////////////////////
    //
    // Read section data
    //         
    // 
    // 			
    //
    ////////////////////////////////////////////////////////////////////////	
    
    
    std::vector<SectionData> sectionProperties;    
    beam_wing bwing;
    //
    if (SectionPropertyFile.is_open()) {  
		std::string line; 
	    	while (std::getline(SectionPropertyFile, line)) {
	    		SectionData temp;
			std::istringstream iss(line);
			if (!(iss >> temp.x >> temp.y >> temp.nwings)) {            
		    		continue;
			}else{
				for (int i=0; i<temp.nwings; ++i){
					iss >> bwing;
					temp.wings.push_back(bwing);
				}
			}
		sectionProperties.push_back(temp);
    }

    
    }
    
    
    std::vector<std::shared_ptr<ChBeamSectionCurvedEasyMultiWings>> Beam_Sections;
    for(auto props : sectionProperties){   
    	std::vector<rectangle_wing> sec_props;
    	for ( auto w : props.wings){
    		rectangle_wing wing;
    		wing.height=w.height;
    		wing.width=w.width;
    		wing.angle=w.angle;
    		//msection->SetProps(wing);
    		sec_props.push_back(wing);
    	}
    	
    	auto newsection = chrono_types::make_shared<ChBeamSectionCurvedEasyMultiWings>(     //curvature, orderOfBasisFunctions,		
		sec_props,			// section props
		YoungModulus,			// Young modulus
		ShearModulus,			// shear modulus
		density			        // density
		);
		
	Beam_Sections.push_back(newsection);
    
     }
    
    /*std::cout << "sectionProperties :\n";
    for (auto props : sectionProperties) {       
        //std::cout << props.x << " "<<props.y<<" "<<props.nwings << " "<<props.L1   << " "<<props.L2<< " "<<props.L3<< " "<<props.W1  << " "<<props.W2<< " "<<props.W3;
        std::cout << props.x << " "<<props.y<<" "<<props.nwings << "\t";  
        for (int i=0; i<props.nwings; ++i){		
		std::cout<< props.wings[i] <<"\t";
	}     
        std::cout << std::endl;        
    }
    */
    
    //
    // Node based creation of element
    //
    int numberOfSection= std::size(sectionProperties);
    //std::cout<<"numberOfSection: "<<numberOfSection<<"\n";
    
    ChBuilderCurvilinearBeamBezier builderR;
    for (int ip=0; ip<numPatches; ip++){
    	//std::cout<<"Patch no: "<<ip<<std::endl;
    	//std::cout<<"numControlPointsPerPatch: "<<numControlPointsPerPatch<<"\t orderOfBasisFunctions: "<<orderOfBasisFunctions<<std::endl;
    	auto pts=controlPointsVectors[ip];
    	std::vector<std::shared_ptr<ChNodeFEAxyzrot>> my_nodes;    	   	
    	for (int ind=0; ind<pts.size(); ind++){    	    
    	    auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(pts[ind]));
    	    my_nodes.push_back(mynode);    		
    	}
    	
    	
    	
    	
    	std::vector<double> knots=knotVectors[ip];
    	//std::vector<double> my_weight(numControlPointsPerPatch);
    	//std::fill(my_weight.begin(), my_weight.end(), 1.0);	

    	//auto msection = chrono_types::make_shared<CurvedBeamSection>(section_inertia, section_elasticity);    	
    	int secID=ip % numberOfSection;
    	//    	
    	auto props = sectionProperties[secID];
    	//msection->SetNwings(props.nwings);
    	std::vector<rectangle_wing> sec_props;
    	for ( auto w : props.wings){
    		rectangle_wing wing;
    		wing.height=w.height;
    		wing.width=w.width;
    		wing.angle=w.angle;
    		//msection->SetProps(wing);
    		sec_props.push_back(wing);
    	}    	
    	
    	auto newsection = Beam_Sections[secID];
    	
    	/*    	
    	std::vector<std::vector<ChVector3d > > polyline_points;
    	for (unsigned int iwing = 0; iwing < newsection->GetNwings(); iwing++){    		
    		
    		auto props=newsection->GetProps();
    		double hw = props[iwing].height;
    		double Bw = props[iwing].width;    
    		double alpha = props[iwing].angle;     		
    		
    		double c=cos(alpha);
    		double s=sin(alpha);
    		//std::cout<<"iwing: "<<iwing<<" hw: "<<hw<<" Bw: "<<Bw<<" alpha: "<<alpha<<"\t";
    		std::vector<ChVector3d >  rectangul={{0, -s*Bw/2, c*Bw/2}, {0, hw*c-s*Bw/2, -hw*s-c*Bw/2}, {0, hw*c+s*Bw/2, -hw*s+c*Bw/2}, {0, s*Bw/2, c*Bw/2}};
    		polyline_points.push_back(rectangul);    		
    		
    		
    	}
    	*/
    	//std::cout<<std::endl;
    	
    	//auto msection_drawshape = chrono_types::make_shared<ChBeamSectionShapePolyline>(polyline_points);
    	//newsection->SetDrawShape(msection_drawshape);
    	
    	//msection->SetWidth_y(beam_wy);
    	//msection->SetWidth_z(beam_wz);
    	//auto msection_drawshape = chrono_types::make_shared<ChBeamSectionShapeRectangular>(0.2, 0.2);
    	//newsection->SetDrawShape(msection_drawshape);
	
	
	
    	builderR.BuildBeam( my_mesh,                 	// mesh to store the resulting elements
                            newsection,  			// section material for beam elements
                            orderOfBasisFunctions, 	// order of spline curve
                            my_nodes, 			// Control points list belongs to this patch    
                            knots,                      // knot vector of this patch     
                            VECT_Z);             	// the 'up' Y direction of the beam
    }
    
       
        
	
}




}  // end namespace wood
}  // end namespace chrono
