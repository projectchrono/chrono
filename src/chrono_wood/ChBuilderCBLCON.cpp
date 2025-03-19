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
//
// =============================================================================
// Authors: Erol Lale
// =============================================================================
// Builder class for CBLCON 
//
//  Builder reads node coordinates, element connectivity, facet info from the file created by preprocessor
//
// 
// =============================================================================
#include "chrono/physics/ChSystem.h"
//#include "chrono/fea/ChBuilderBeam.h"
#include "chrono_wood/ChBeamSectionCBLCON.h"
#include "chrono_wood/ChBuilderCBLCON.h"

namespace chrono {
namespace wood {

// ------------------------------------------------------------------
// ChBuilderCBLCON
// ------------------------------------------------------------------
struct ConnectorData {
    int inode;
    int jnode;
    int conType;
    double centerx;
    double centery;
    double centerz;
    double mx;
    double my;
    double mz;
    double lx;
    double ly;
    double lz;
    double width;
    double height;
    double randomField=0;
	double knot=0;
	double preCrack=0;
};


ChVector3d projectVectorOntoPlane(ChVector3d& vectorToProject, ChVector3d& planeNormal) {
    // Step 2: Calculate the dot product of the vector and the normalized normal vector
    ChVector3d normalizedNormal=planeNormal.GetNormalized();
    double dotProduct = vectorToProject[0]*normalizedNormal[0] + vectorToProject[1]*normalizedNormal[1] + vectorToProject[2]*normalizedNormal[2] ;    
    return (vectorToProject-dotProduct * normalizedNormal).GetNormalized();
}


void ChBuilderCBLCON::BuildBeam(std::shared_ptr<ChMesh> mesh,                 // mesh to store the resulting elements
                                   std::shared_ptr<ChBeamSectionCBLCON> sect,     // section material for beam elements
                                   const int N,                                  // number of elements in the segment
                                   const ChVector3d A,                           // starting point
                                   const ChVector3d B,                           // ending point
                                   const ChVector3d Ydir                         // the 'up' Y direction of the beam
) {
    beam_elems.clear();
    beam_nodes.clear();

    ChMatrix33<> mrot;
    mrot.SetFromAxisX (B - A, Ydir);

    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(A, mrot));
    mesh->AddNode(nodeA);
    beam_nodes.push_back(nodeA);

    for (int i = 1; i <= N; ++i) {
        double eta = (double)i / (double)N;
        ChVector3d pos = A + (B - A) * eta;

        auto nodeB = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(pos, mrot));
        mesh->AddNode(nodeB);
        beam_nodes.push_back(nodeB);

        auto element = chrono_types::make_shared<ChElementCBLCON>();
        mesh->AddElement(element);
        beam_elems.push_back(element);

        element->SetNodes(beam_nodes[i - 1], beam_nodes[i]);

        element->SetSection(sect);
    }
}

void ChBuilderCBLCON::BuildBeam(std::shared_ptr<ChMesh> mesh,                 // mesh to store the resulting elements
                                   std::shared_ptr<ChBeamSectionCBLCON> sect,     // section material for beam elements
                                   const int N,                                  // number of elements in the segment
                                   std::shared_ptr<ChNodeFEAxyzrot> nodeA,       // starting point
                                   std::shared_ptr<ChNodeFEAxyzrot> nodeB,       // ending point
                                   const ChVector3d Ydir                         // the 'up' Y direction of the beam
) {
    beam_elems.clear();
    beam_nodes.clear();

    ChMatrix33<> mrot;
    mrot.SetFromAxisX (nodeB->Frame().GetPos() - nodeA->Frame().GetPos(), Ydir);

    beam_nodes.push_back(nodeA);

    for (int i = 1; i <= N; ++i) {
        double eta = (double)i / (double)N;
        ChVector3d pos = nodeA->Frame().GetPos() + (nodeB->Frame().GetPos() - nodeA->Frame().GetPos()) * eta;

        std::shared_ptr<ChNodeFEAxyzrot> nodeBi;
        if (i < N) {
            nodeBi = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(pos, mrot));
            mesh->AddNode(nodeBi);
        } else
            nodeBi = nodeB;  // last node: use the one passed as parameter.

        beam_nodes.push_back(nodeBi);

        auto element = chrono_types::make_shared<ChElementCBLCON>();
        mesh->AddElement(element);
        beam_elems.push_back(element);

        element->SetNodes(beam_nodes[i - 1], beam_nodes[i]);

        ChQuaternion<> elrot = mrot.GetQuaternion();
        element->SetNodeAreferenceRot(elrot.GetConjugate() * element->GetNodeA()->Frame().GetRot());
        element->SetNodeBreferenceRot(elrot.GetConjugate() * element->GetNodeB()->Frame().GetRot());

        element->SetSection(sect);
    }
}

void ChBuilderCBLCON::BuildBeam(std::shared_ptr<ChMesh> mesh,                 // mesh to store the resulting elements
                                   std::shared_ptr<ChBeamSectionCBLCON> sect,     // section material for beam elements
                                   const int N,                                  // number of elements in the segment
                                   std::shared_ptr<ChNodeFEAxyzrot> nodeA,       // starting point
                                   const ChVector3d B,                           // ending point
                                   const ChVector3d Ydir                         // the 'up' Y direction of the beam
) {
    beam_elems.clear();
    beam_nodes.clear();

    ChMatrix33<> mrot;
    mrot.SetFromAxisX (B - nodeA->Frame().GetPos(), Ydir);

    beam_nodes.push_back(nodeA);

    for (int i = 1; i <= N; ++i) {
        double eta = (double)i / (double)N;
        ChVector3d pos = nodeA->Frame().GetPos() + (B - nodeA->Frame().GetPos()) * eta;

        auto nodeBi = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(pos, mrot));
        mesh->AddNode(nodeBi);
        beam_nodes.push_back(nodeBi);

        auto element = chrono_types::make_shared<ChElementCBLCON>();
        mesh->AddElement(element);
        beam_elems.push_back(element);

        element->SetNodes(beam_nodes[i - 1], beam_nodes[i]);

        ChQuaternion<> elrot = mrot.GetQuaternion();
        element->SetNodeAreferenceRot(elrot.GetConjugate() * element->GetNodeA()->Frame().GetRot());
        element->SetNodeBreferenceRot(elrot.GetConjugate() * element->GetNodeB()->Frame().GetRot());
        // GetLog() << "Element n." << i << " with rotations: \n";
        // GetLog() << "   Qa=" << element->GetNodeAreferenceRot() << "\n";
        // GetLog() << "   Qb=" << element->GetNodeBreferenceRot() << "\n\n";
        element->SetSection(sect);
    }
}


void ChBuilderCBLCON::read_CBLCON_info(std::shared_ptr<ChMesh> my_mesh,  std::shared_ptr<ChWoodMaterialVECT> vect_mat, std::string& CBLCON_data_path, 
				std::string& CBLCON_GeoName){
	//
	std::vector<std::vector<double>> Mfacet;
	//std::vector<vertdata> MvertIDs;
	//
	//
	//
	std::string nodesFilename=CBLCON_data_path+CBLCON_GeoName+"-chronoNodes.dat";
	std::string meshFilename=CBLCON_data_path+CBLCON_GeoName+"-mesh.txt"; 
	std::cout<< "meshFilename: " <<meshFilename<<std::endl;
		
    	//
    	std::ifstream nodesFile(nodesFilename);
	std::ifstream con_MeshFile(meshFilename);        
    	////////////////////////////////////////////////////////////////////////
    	//
    	//
    	// node info should be read before for Bezier elements and store into mesh object
	// But if mesh object doesnt contain node info then
	// Read node info from Freecad produced "<projectname>-data-nodes.dat" file
	//          ( x, y, z coordinates )
	// And create node instances using ChNodeFEAxyzrot class. 
	// 			( node with 6 deegrees of freedom)
	//
	//
	////////////////////////////////////////////////////////////////////////
	/*std::vector< std::shared_ptr<ChNodeFEAxyzrot> > CBLnodeList;	
	if (!(my_mesh->GetNodes().size()))
	    if (nodesFile.is_open()) {
	       	chrono::ChVector3d pos;
	       	double x, y, z;
	       	char com1, com2;
	       	unsigned int idnode=0;
			std::string line;
			
			while (std::getline(nodesFile, line)) {			
				//
				std::istringstream sline(line);
				if( !(sline>> x >> com1 >> y >> com2 >> z))
					continue; 				
					
				//
				auto mnode= chrono_types::make_shared<ChNodeFEAxyzrot>(chrono::ChFrame<>(chrono::ChVector3d(x, y, z)));
				mnode->SetIndex(idnode);
				CBLnodeList.push_back(mnode);
		        	my_mesh->AddNode(mnode);					
				
			}
	    }
	    else{
	    	throw std::invalid_argument("ERROR opening nodes info file: " + std::string(nodesFilename) + "\n");
	    	exit(EXIT_FAILURE);
	    }
	
	*/
	
    	////////////////////////////////////////////////////////////////////////
    	//
    	// Read Connector Data Generated with FreeCAD Mesh Generation Tool
    	// And create section for connector and element instances using ChElementCBLCON class 
    	//
	// [inode jnode centerx centery centerz dx1 dy1 dz1 dx2 dy2 dz2 width height random_field connector_flag]
	// 
	////////////////////////////////////////////////////////////////////////
    	std::vector<ConnectorData> connector_data;
    	ConnectorData temp;
    	double dtemp;
    	//
    	if (con_MeshFile.is_open()) {  
		std::string line; 
	    	while (std::getline(con_MeshFile, line)) {
			std::istringstream iss(line);
			if (!(iss >> temp.inode >> temp.jnode >> temp.centerx >> temp.centery >> temp.centerz >> dtemp >> dtemp >> dtemp >> dtemp  >> dtemp >> dtemp
			>> temp.mx >> temp.my >> temp.mz >> temp.lx >> temp.ly >> temp.lz >> temp.width >> temp.height >> temp.randomField >> temp.conType >> temp.knot >> temp.preCrack)) {            
		    		continue;
			}
		connector_data.push_back(temp);
    	}

	    
	    /*
	    for (const auto& connector : connector_data) {
		std::cout << connector.inode << " " << connector.jnode << " " << connector.centerx << " " << connector.centery << " " << connector.centerz << " "
		          << connector.mx << " " << connector.my << " " << connector.mz << " " << connector.lx << " " << connector.ly << " " << connector.lz << " "
		          << connector.width << " " << connector.height << " " << connector.randomField << " "<< connector.conType << connector.knot << " "<< connector.preCrack << std::endl;
	    }
    	*/	
    	}
    	else{
    		throw std::invalid_argument("ERROR opening connector info file: " + std::string(meshFilename) + "\n");
    		exit(EXIT_FAILURE);
    	}
	
	int elementnum=my_mesh->GetNumElements();
	std::cout<<"Connector elementnum "<<elementnum<<std::endl;
	
	
	////////////////////////////////////////////////////////////////////////
    	//
    	// Assign element and section properties to each element, such as;
	//       Area of facet, sytem of referans, center point
	//			vertices coordinates
	//
	////////////////////////////////////////////////////////////////////////  
	chrono::ChVector3d facetC;
	chrono::ChMatrix33<double> nmL;	
	
	for (auto connector : connector_data) {
		//
		// Get connector nodes which previously stored as pointers in mesh object	
		//
		if(connector.preCrack)
			continue;   //if precrack skip inserting connector into mesh
		//
		auto nodeA = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(connector.inode-1));
		auto nodeB = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(connector.jnode-1));
		//
		double area=connector.width*connector.height;
		//
		facetC[0]=connector.centerx;
		facetC[1]=connector.centery;
		facetC[2]=connector.centerz;	
		//
		ChVector3d nvec = nodeB->GetX0().GetPos() - nodeA->GetX0().GetPos();
		nvec.Normalize();
		
	    	ChVector3d myele = (nodeA->Frame().GetRotMat().GetAxisY() + 
				    nodeB->Frame().GetRotMat().GetAxisY()).GetNormalized();
		//ChVector3d myele(-nvec[1],nvec[0],nvec[2]);
	   	ChMatrix33<> nmL;
	   	ChMatrix33<> Aabs;
	   	Aabs.SetFromAxisX (nvec, myele);	   	
	    	//abs_rot = Aabs.GetQuaternion();	
		//
				
		nmL(0,0)=nvec.x();
		nmL(0,1)=nvec.y();
		nmL(0,2)=nvec.z();
		//
		//ChVector3d mvec={connector.mx, connector.my, connector.mz};
		ChVector3d mvec=facetC-(nodeA->GetX0().GetPos()+nodeB->GetX0().GetPos())/2.;
		//std::cout<< "facetC: "<<facetC<<std::endl;
		//std::cout<<"mvec-1: "<<mvec<<std::endl;		
		mvec= projectVectorOntoPlane(mvec, nvec);		
		mvec.Normalize();		
		//std::cout<<"mvec-2: "<<mvec<<std::endl;	
		nmL(1,0)=mvec.x();
		nmL(1,1)=mvec.y();
		nmL(1,2)=mvec.z();
		//
		//ChVector3d lvec={connector.lx, connector.ly, connector.lz};
		ChVector3d lvec=nvec.Cross(mvec);
		lvec.Normalize();
		nmL(2,0)=lvec.x();
		nmL(2,1)=lvec.y();
		nmL(2,2)=lvec.z();
		
		//std::cout<<"inode: "<<nodeA->GetIndex()<<" jnode: "<<nodeB->GetIndex()<<std::endl;
		//std::cout<<"NodeA: "<<nodeA->GetPos().x()<<"\t"<<nodeA->GetPos().y()<<"\t"<<nodeA->GetPos().z()<<"\n";
		//std::cout<<"NodeB: "<<nodeB->GetPos().x()<<"\t"<<nodeB->GetPos().y()<<"\t"<<nodeB->GetPos().z()<<"\n";
		//std::cout<<"connector.width: "<< connector.width<< " connector.height: "<< connector.height <<std::endl;
		//std::cout<<"Nvec: "<< nvec<<std::endl;	
		//std::cout<<"Aabs: "<<Aabs.transpose()<<std::endl;		
		//std::cout<<"nmL: "<<nmL<<std::endl;
		//std::cout<<"section type: "<< connector.conType <<std::endl;		
		//std::cout<<"-----------------------------------------------------------------------------\n";
		
		
		//
		int typeFlag=connector.conType;
		//
		auto msection = chrono_types::make_shared<ChBeamSectionCBLCON>();
		msection->Set_material(vect_mat);
		msection->SetWidth(connector.width);
		msection->SetHeight(connector.height);
		msection->Set_area(area);
		msection->Set_center(facetC);
		if (connector.conType==4 & mvec.Length()!=0){
			msection->Set_facetFrame(nmL);
		}else{
			msection->Set_facetFrame(Aabs.transpose());
		}
		
		auto msection_drawshape = chrono_types::make_shared<ChBeamSectionShapeRectangular>(connector.width, connector.height);
    		msection->SetDrawShape(msection_drawshape);
		//msection->SetDrawThickness(0.1, 0.25);
		switch(typeFlag) {
        		case 1: 
        			msection->SetSectionType(ChBeamSectionCBLCON::ConSectionType::transverse_bot);  
        			break;
        		case 2:
        			msection->SetSectionType(ChBeamSectionCBLCON::ConSectionType::transverse_generic); 
        			break;
        		case 3:
        			msection->SetSectionType(ChBeamSectionCBLCON::ConSectionType::transverse_top); 
        			break;
        		case 4: 
        			msection->SetSectionType(ChBeamSectionCBLCON::ConSectionType::longitudinal);        			
        			break;
        	}
		//		
		auto mel = chrono_types::make_shared<ChElementCBLCON>();
		mel->SetNodes(nodeA, nodeB);
		mel->SetSection(msection);		
		my_mesh->AddElement(mel);			
		
	}
	//exit(0);
	
}



void ChBuilderCBLCON::read_CBLCON_info(std::shared_ptr<ChMesh> my_mesh,  std::vector<std::shared_ptr<ChWoodMaterialVECT>> vect_mat, std::string& CBLCON_data_path, 
				std::string& CBLCON_GeoName){
	
	
	
	//
	std::vector<std::vector<double>> Mfacet;
	//std::vector<vertdata> MvertIDs;
	//
	//
	//
	std::string nodesFilename=CBLCON_data_path+CBLCON_GeoName+"-chronoNodes.dat";
	std::string meshFilename=CBLCON_data_path+CBLCON_GeoName+"-mesh.txt"; 
	std::cout<< "meshFilename: " <<meshFilename<<std::endl;
		
    	//
    	std::ifstream nodesFile(nodesFilename);
	std::ifstream con_MeshFile(meshFilename);        
    	////////////////////////////////////////////////////////////////////////
    	//
    	//
    	// node info should be read before for Bezier elements and store into mesh object
	// But if mesh object doesnt contain node info then
	// Read node info from Freecad produced "<projectname>-data-nodes.dat" file
	//          ( x, y, z coordinates )
	// And create node instances using ChNodeFEAxyzrot class. 
	// 			( node with 6 deegrees of freedom)
	//
	//
	////////////////////////////////////////////////////////////////////////
	
	
    	////////////////////////////////////////////////////////////////////////
    	//
    	// Read Connector Data Generated with FreeCAD Mesh Generation Tool
    	// And create section for connector and element instances using ChElementCBLCON class 
    	//
	// [inode jnode centerx centery centerz dx1 dy1 dz1 dx2 dy2 dz2 width height random_field connector_flag]
	// 
	////////////////////////////////////////////////////////////////////////
    	std::vector<ConnectorData> connector_data;
    	ConnectorData temp;
    	double dtemp;
    	//
    	if (con_MeshFile.is_open()) {  
		std::string line; 
	    	while (std::getline(con_MeshFile, line)) {
			std::istringstream iss(line);
			if (!(iss >> temp.inode >> temp.jnode >> temp.centerx >> temp.centery >> temp.centerz >> dtemp >> dtemp >> dtemp >> dtemp  >> dtemp >> dtemp
			>> temp.mx >> temp.my >> temp.mz >> temp.lx >> temp.ly >> temp.lz >> temp.width >> temp.height >> temp.randomField >> temp.conType >> temp.knot >> temp.preCrack)) {            
		    		continue;
			}
		connector_data.push_back(temp);
    	}

	    
	    /*
	    for (const auto& connector : connector_data) {
		std::cout << connector.inode << " " << connector.jnode << " " << connector.centerx << " " << connector.centery << " " << connector.centerz << " "
		          << connector.mx << " " << connector.my << " " << connector.mz << " " << connector.lx << " " << connector.ly << " " << connector.lz << " "
		          << connector.width << " " << connector.height << " " << connector.randomField<< " " << connector.conType<< " "<< temp.knot << " " << temp.preCrack << std::endl;
	    }*/
    		
    	}
    	else{
    		throw std::invalid_argument("ERROR opening connector info file: " + std::string(meshFilename) + "\n");
    		exit(EXIT_FAILURE);
    	}
	
	int elementnum=my_mesh->GetNumElements();
	std::cout<<"Connector elementnum "<<elementnum<<std::endl;
	
	
	////////////////////////////////////////////////////////////////////////
    	//
    	// Assign element and section properties to each element, such as;
	//       Area of facet, sytem of referans, center point
	//			vertices coordinates
	//
	////////////////////////////////////////////////////////////////////////  
	chrono::ChVector3d facetC;
	chrono::ChMatrix33<double> nmL;		
	for (auto connector : connector_data) {		
		//
		// Get connector nodes which previously stored as pointers in mesh object	
		//
		if(connector.preCrack)
			continue;   //if precrack skip inserting connector into mesh
		//
		auto nodeA = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(connector.inode-1));
		auto nodeB = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(my_mesh->GetNode(connector.jnode-1));
		//
		double area=connector.width*connector.height;
		//
		facetC[0]=connector.centerx;
		facetC[1]=connector.centery;
		facetC[2]=connector.centerz;	
		//
		ChVector3d nvec = nodeB->GetX0().GetPos() - nodeA->GetX0().GetPos();
		nvec.Normalize();
		//std::cout<<"nvec: "<<nvec<<std::endl;
	    	ChVector3d myele = (nodeA->Frame().GetRotMat().GetAxisY() + 
				    nodeB->Frame().GetRotMat().GetAxisY()).GetNormalized();
		//ChVector3d myele(-nvec[1],nvec[0],nvec[2]);
		//std::cout<<"myele: "<<myele<<std::endl;
	   	ChMatrix33<> nmL;
	   	ChMatrix33<> Aabs;
	   	Aabs.SetFromAxisX (nvec, myele);	   	
	    	//abs_rot = Aabs.GetQuaternion();	
		//
		//std::cout<<"Aabs: "<<Aabs<<std::endl;
		//std::cout<<"nvec: "<<nvec<<std::endl;		
		nmL(0,0)=nvec.x();
		nmL(0,1)=nvec.y();
		nmL(0,2)=nvec.z();
		//
		//ChVector3d mvec={connector.mx, connector.my, connector.mz};
		ChVector3d mvec=facetC-(nodeA->GetX0().GetPos()+nodeB->GetX0().GetPos())/2.;
		//std::cout<< "facetC: "<<facetC<<std::endl;
		//std::cout<<"mvec-1: "<<mvec<<std::endl;		
		mvec= projectVectorOntoPlane(mvec, nvec);		
		mvec.Normalize();		
		//std::cout<<"mvec: "<<mvec<<std::endl;	
		nmL(1,0)=mvec.x();
		nmL(1,1)=mvec.y();
		nmL(1,2)=mvec.z();
		//
		//ChVector3d lvec={connector.lx, connector.ly, connector.lz};
		ChVector3d lvec=nvec.Cross(mvec);
		lvec.Normalize();
		nmL(2,0)=lvec.x();
		nmL(2,1)=lvec.y();
		nmL(2,2)=lvec.z();
		//std::cout<<"lvec: "<<lvec<<std::endl;
		//std::cout<<"inode: "<<nodeA->GetIndex()<<" jnode: "<<nodeB->GetIndex()<<std::endl;
		//std::cout<<"NodeA: "<<nodeA->GetPos().x()<<"\t"<<nodeA->GetPos().y()<<"\t"<<nodeA->GetPos().z()<<"\n";
		//std::cout<<"NodeB: "<<nodeB->GetPos().x()<<"\t"<<nodeB->GetPos().y()<<"\t"<<nodeB->GetPos().z()<<"\n";
		//std::cout<<"connector.width: "<< connector.width<< " connector.height: "<< connector.height <<std::endl;
		//std::cout<<"Nvec: "<< nvec<<std::endl;	
		//std::cout<<"Aabs: "<<Aabs<<std::endl;		
		//std::cout<<"nmL: "<<nmL<<std::endl;
		//std::cout<<"section type: "<< connector.conType <<std::endl;		
		//std::cout<<"-----------------------------------------------------------------------------\n";
		
		
		//
		int typeFlag=connector.conType;
		//
		auto msection = chrono_types::make_shared<ChBeamSectionCBLCON>();		
		msection->SetWidth(connector.width);
		msection->SetHeight(connector.height);
		msection->Set_area(area);
		msection->Set_center(facetC);
		msection->SetRandomField(connector.randomField);
		if (connector.conType==4 & mvec.Length()!=0){
			//std::cout<<"Longuitidonal connector\n";
			msection->Set_facetFrame(nmL);
		}else{	// TODO JBC: WHAT IS THAT ? THAT MAKES NO SENSE, should it be the transpose?
            //std::cout<<"transver connector\n";		
			nmL(0,0)= Aabs(0, 0);  nmL(0,1)=Aabs(1, 0); nmL(0,2)=Aabs(2, 0);
			nmL(1,0)= Aabs(0, 2);  nmL(1,1)=Aabs(1, 2); nmL(1,2)=Aabs(2, 2);
			nmL(2,0)= Aabs(0, 1);  nmL(2,1)=Aabs(1, 1); nmL(2,2)=Aabs(2, 1);
			//std::cout<<"nmL-new: "<<nmL<<std::endl;
			msection->Set_facetFrame(nmL);			
		}
		//std::cout<<"nmL-new: "<<nmL<<std::endl;
		//std::cout<<"////////////////////////////////////////////////"<<std::endl;
		
		auto msection_drawshape = chrono_types::make_shared<ChBeamSectionShapeRectangular>(connector.width, connector.height);
    	msection->SetDrawShape(msection_drawshape);		
		//msection->SetDrawThickness(0.1, 0.25);
		switch(typeFlag) {
        		case 1: 
        			msection->SetSectionType(ChBeamSectionCBLCON::ConSectionType::transverse_bot);
        			if (vect_mat.size()==4){  
        				msection->Set_material(vect_mat[typeFlag-1]); 
        			}else{
        				msection->Set_material(vect_mat[0]);
        			}
        			       			
        			break;
        		case 2:
        			msection->SetSectionType(ChBeamSectionCBLCON::ConSectionType::transverse_generic); 
        			if (vect_mat.size()==4){  
        				msection->Set_material(vect_mat[typeFlag-1]); 
        			}else{
        				msection->Set_material(vect_mat[0]);
        			}
        			break;
        		case 3:
        			msection->SetSectionType(ChBeamSectionCBLCON::ConSectionType::transverse_top); 
        			if (vect_mat.size()==4){  
        				msection->Set_material(vect_mat[typeFlag-1]); 
        			}else{
        				msection->Set_material(vect_mat[0]);
        			}
        			break;
        		case 4: 
        			msection->SetSectionType(ChBeamSectionCBLCON::ConSectionType::longitudinal);     
        			if (vect_mat.size()==4){  
        				msection->Set_material(vect_mat[typeFlag-1]); 
        			}else{
        				msection->Set_material(vect_mat[1]);
        			}   			
        			break;
        	}
		//		
		auto mel = chrono_types::make_shared<ChElementCBLCON>();
		mel->SetNodes(nodeA, nodeB);
		mel->SetSection(msection);		
		my_mesh->AddElement(mel);			
		
	}
	
	
}


}  // end namespace wood
}  // end namespace chrono
