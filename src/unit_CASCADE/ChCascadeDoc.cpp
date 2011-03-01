///////////////////////////////////////////////////
//
//   ChCascadeDoc.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

    

#include "ChCascadeDoc.h"

//#include "ChIrrCascadeMeshTools.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_HShape.hxx>
#include <handle_TopoDS_HShape.hxx>
#include <handle_TopoDS_TShape.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_STEPModelType.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <BRepMesh.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp_Explorer.hxx>
#include <Bnd_Box.hxx>
#include <gp_Pnt.hxx>
#include <Prs3d_ShapeTool.hxx>
#include <BRepAdaptor_HSurface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TColgp_HArray1OfVec.hxx> 
#include <TColStd_HArray1OfInteger.hxx>
#include <Poly_Connect.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <TColgp_Array1OfDir.hxx>
#include <CSLib_DerivativeStatus.hxx>
#include <CSLib_NormalStatus.hxx>
#include <CSLib.hxx>
#include <TColgp_Array1OfPnt2d.hxx>
#include <TDocStd_Document.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <TDataStd_Name.hxx>
#include <Interface_Static.hxx>
#include <TDF_Label.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_Tool.hxx>
#include <TDataStd_Shape.hxx>
#include <TDataStd_Position.hxx>
#include <TObj_TObject.hxx>
#include <TObj_TReference.hxx>
#include <TNaming_NamedShape.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>


using namespace chrono;
using namespace cascade;



ChCascadeDoc::ChCascadeDoc()
{
	doc = new Handle(TDocStd_Document);
	(*doc) = new TDocStd_Document("MyStepDoc");
}

ChCascadeDoc::~ChCascadeDoc()
{
	if (doc) delete doc; doc = 0;
}






class callback_CascadeDoc_dump : public ChCascadeDoc::callback_CascadeDoc
{
public:
	virtual bool ForShape(TopoDS_Shape& mshape, TopLoc_Location& mloc, char* mname, int mlevel, TDF_Label& mlabel)
	{
		for (int i=0; i<mlevel; i++) GetLog()<< "  ";
		GetLog() << "-Name :" << mname;
		
		if (mlevel==0) GetLog() << " (root)";
		GetLog() << "\n";

		for (int i=0; i<mlevel; i++) GetLog()<< "  ";
		gp_XYZ mtr = mloc.Transformation().TranslationPart();
		GetLog() << "      pos at: "<< mtr.X() <<" "<< mtr.Y() << " "<<mtr.Z() <<"\n";

		return true;
	}
};

class callback_CascadeDoc_find_name : public ChCascadeDoc::callback_CascadeDoc
{
public:
	char search_name[200];

	bool res_found;
	TopoDS_Shape res_shape;
	TopLoc_Location res_loc;
	int res_level;
	TDF_Label res_label;
	
	callback_CascadeDoc_find_name(char* mname) : res_found(false), res_level(0)  
	{
		strcpy(search_name, mname);
	}

	virtual bool ForShape(TopoDS_Shape& mshape, TopLoc_Location& mloc, char* mname, int mlevel, TDF_Label& mlabel)
	{
		if (strcmp(mname, search_name)) 
			return true; // proceed, not found
		// ..else equal string: found
		res_found = true;
		res_shape = mshape;
		res_loc = mloc;
		res_level = mlevel;
		res_label = mlabel;

		return false;
	}
};



static bool recurse_CascadeDoc(TDF_Label label, Handle_XCAFDoc_ShapeTool& shapeTool, int level, ChCascadeDoc::callback_CascadeDoc& mcallback)
{
	TDF_LabelSequence child_labels;
	Standard_Boolean is_assembly;

	is_assembly = shapeTool->GetComponents(label, child_labels, 0);

	char mstring[200]="no name";
	Standard_PCHaracter mchastr = mstring;

	// access name
	Handle_TDataStd_Name N;
	if (label.FindAttribute( TDataStd_Name::GetID(), N ) )
	{
		N->Get().ToUTF8CString(mchastr);
	}	
	

	TDF_Label reflabel;
	Standard_Boolean isref = shapeTool->GetReferredShape(label, reflabel);

	if (isref) // ..maybe it references some shape: the name is stored there...
	{
		Handle_TDataStd_Name N;
		if ( reflabel.FindAttribute( TDataStd_Name::GetID(), N ) )
		{
			N->Get().ToUTF8CString(mchastr);
		}
	}


	// access shape and position
	TopoDS_Shape rShape;
	rShape = shapeTool->GetShape( label);
	if (!rShape.IsNull()) 
	{
		TopLoc_Location mloc =  shapeTool->GetLocation(label);
		
		// === call the callback! ===
		if (!mcallback.ForShape(rShape, mloc, mstring, level, label))
			return false;
	}
	
	// Recurse all children !!!
	if (is_assembly)
	{
		for ( Standard_Integer j = 1; j <= child_labels.Length(); j++ )
		{
			TDF_Label clabel = child_labels.Value( j );
			if (!recurse_CascadeDoc(clabel, shapeTool, (level+1), mcallback))
				return false;
		}
	}

	// If it is a reference, Recurse all children of reference
	if (isref) 
	{
		TDF_LabelSequence refchild_labels;
		Standard_Boolean refis_assembly;
		refis_assembly = shapeTool->GetComponents(reflabel, refchild_labels, 0);
		for ( Standard_Integer j = 1; j <= refchild_labels.Length(); j++ )
		{
			TDF_Label clabel = refchild_labels.Value( j );
			if (!recurse_CascadeDoc(clabel, shapeTool, (level+1), mcallback))
				return false;
		}
	}

	return true;
}



void ChCascadeDoc::ScanCascadeShapes(callback_CascadeDoc& mcallback)
{
	Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool((*doc)->Main());
	TDF_LabelSequence root_labels;
	shapeTool->GetFreeShapes( root_labels );
	for ( Standard_Integer i = 1; i <= root_labels.Length(); i++ )
	{
		TDF_Label label = root_labels.Value( i );
		int level = 0;
		recurse_CascadeDoc(label, shapeTool, level, mcallback);
	}
}



bool ChCascadeDoc::Load_STEP(const char* filename)
{
	STEPCAFControl_Reader cafreader;

	if (!Interface_Static::SetCVal ("xstep.cascade.unit","M"))
		GetLog() << "\n\n ERROR SETTING 'M' UNITS!!!..   \n\n\n";

	IFSelect_ReturnStatus aStatus = cafreader.ReadFile(filename);

	if (aStatus == IFSelect_RetDone)
	{ 
		Standard_Boolean aRes = cafreader.Transfer((*doc));
		return true;
	}
	return false;
}


void ChCascadeDoc::Dump(ChStreamOutAscii& mstream)
{
	callback_CascadeDoc_dump adumper;
	this->ScanCascadeShapes(adumper);
}


bool ChCascadeDoc::GetRootShape(TopoDS_Shape& mshape, const int num)
{
	Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool((*doc)->Main());
	TDF_LabelSequence root_labels;
	shapeTool->GetFreeShapes( root_labels );
	if (num > root_labels.Length()) 
		return false;
	mshape = shapeTool->GetShape( root_labels.Value(num) );
	return true;
}


bool ChCascadeDoc::GetVolumeProperties(const TopoDS_Shape& mshape,	///< pass the shape here
						const double density,				///< pass the density here 
						ChVector<>& center_position,		///< get the position center, respect to shape pos.
						ChVector<>& inertiaXX,				///< get the inertia diagonal terms
						ChVector<>& inertiaXY,				///< get the inertia extradiagonal terms
						double& volume,						///< get the volume
						double& mass						///< get the mass
						)
{
	if (mshape.IsNull()) 
		return false;

	GProp_GProps mprops;
	GProp_GProps vprops;
	BRepGProp::VolumeProperties(mshape,mprops);
	BRepGProp::VolumeProperties(mshape,vprops);

	mprops.Add(mprops, density);

	mass = mprops.Mass();
	volume = vprops.Mass();
	gp_Pnt G = mprops.CentreOfMass ();
	gp_Mat I = mprops.MatrixOfInertia();

	center_position.x = G.X();
	center_position.y = G.Y();
	center_position.z = G.Z();

	inertiaXX.x = I(1,1);
	inertiaXX.y = I(2,2);
	inertiaXX.z = I(3,3);
	inertiaXY.x = I(1,2);
	inertiaXY.y = I(1,3);
	inertiaXY.z = I(2,3);

	return true;
}












/////////////////////
