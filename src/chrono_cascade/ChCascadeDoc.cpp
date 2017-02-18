//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChCascadeDoc.cpp
//
// ------------------------------------------------
//             http://www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono_cascade/ChCascadeDoc.h"

#include "chrono/core/ChMatrixDynamic.h"

//#include "chrono_cascade/ChIrrCascadeMeshTools.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_HShape.hxx>
#include <Handle_TopoDS_HShape.hxx>
#include <Handle_TopoDS_TShape.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_StepModelType.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <BRepMesh.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRep_Builder.hxx>
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
#include <TObj_TObject.hxx>
#include <TObj_TReference.hxx>
#include <TNaming_NamedShape.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>

using namespace chrono;
using namespace cascade;

ChCascadeDoc::ChCascadeDoc() {
    doc = new Handle(TDocStd_Document);
    (*doc) = new TDocStd_Document("MyStepDoc");
}

ChCascadeDoc::~ChCascadeDoc() {
    if (doc)
        delete doc;
    doc = 0;
}

static bool recurse_CascadeDoc(TDF_Label label,
                               Handle_XCAFDoc_ShapeTool& shapeTool,
                               TopLoc_Location& parentloc,
                               int level,
                               ChCascadeDoc::callback_CascadeDoc& mcallback) {
    TDF_LabelSequence child_labels;
    Standard_Boolean is_assembly;

    is_assembly = shapeTool->GetComponents(label, child_labels, 0);

    char mstring[200] = "no name";
    Standard_PCharacter mchastr = mstring;

    // access name
    Handle_TDataStd_Name N;
    if (label.FindAttribute(TDataStd_Name::GetID(), N)) {
        N->Get().ToUTF8CString(mchastr);
    }

    TDF_Label reflabel;
    Standard_Boolean isref = shapeTool->GetReferredShape(label, reflabel);

    if (isref)  // ..maybe it references some shape: the name is stored there...
    {
        Handle_TDataStd_Name N;
        if (reflabel.FindAttribute(TDataStd_Name::GetID(), N)) {
            N->Get().ToUTF8CString(mchastr);
        }
    }

    TopLoc_Location mloc = shapeTool->GetLocation(label);
    TopLoc_Location absloc = parentloc.Multiplied(mloc);

    // access shape and position
    TopoDS_Shape rShape;
    rShape = shapeTool->GetShape(label);
    if (!rShape.IsNull()) {
        // === call the callback! ===
        if (!mcallback.ForShape(rShape, absloc, mstring, level, label))
            return false;  // (skip children recursion if returned false)
    }

    // Recurse all children !!!
    if (is_assembly) {
        for (Standard_Integer j = 1; j <= child_labels.Length(); j++) {
            TDF_Label clabel = child_labels.Value(j);
            recurse_CascadeDoc(clabel, shapeTool, absloc, (level + 1), mcallback);
        }
    }

    // If it is a reference, Recurse all children of reference
    if (isref) {
        TDF_LabelSequence refchild_labels;
        Standard_Boolean refis_assembly;
        refis_assembly = shapeTool->GetComponents(reflabel, refchild_labels, 0);
        for (Standard_Integer j = 1; j <= refchild_labels.Length(); j++) {
            TDF_Label clabel = refchild_labels.Value(j);
            recurse_CascadeDoc(clabel, shapeTool, absloc, (level + 1), mcallback);
        }
    }

    return true;
}

void ChCascadeDoc::ScanCascadeShapes(callback_CascadeDoc& mcallback) {
    TopLoc_Location rootloc;
    rootloc.Identity();

    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool((*doc)->Main());
    TDF_LabelSequence root_labels;
    shapeTool->GetFreeShapes(root_labels);
    for (Standard_Integer i = 1; i <= root_labels.Length(); i++) {
        TDF_Label label = root_labels.Value(i);
        int level = 0;
        recurse_CascadeDoc(label, shapeTool, rootloc, level, mcallback);
    }
}

bool ChCascadeDoc::Load_STEP(const char* filename) {
    STEPCAFControl_Reader cafreader;

    if (!Interface_Static::SetCVal("xstep.cascade.unit", "M"))
        GetLog() << "\n\n ERROR SETTING 'M' UNITS!!!..   \n\n\n";

    IFSelect_ReturnStatus aStatus = cafreader.ReadFile(filename);

    if (aStatus == IFSelect_RetDone) {
        Standard_Boolean aRes = cafreader.Transfer((*doc));
        return true;
    }
    return false;
}

class callback_CascadeDoc_dump : public ChCascadeDoc::callback_CascadeDoc {
  public:
    virtual bool ForShape(TopoDS_Shape& mshape, TopLoc_Location& mloc, char* mname, int mlevel, TDF_Label& mlabel) {
        for (int i = 0; i < mlevel; i++)
            GetLog() << "  ";
        GetLog() << "-Name :" << mname;

        if (mlevel == 0)
            GetLog() << " (root)";
        GetLog() << "\n";

        for (int i = 0; i < mlevel; i++)
            GetLog() << "  ";
        gp_XYZ mtr = mloc.Transformation().TranslationPart();
        GetLog() << "      pos at: " << mtr.X() << " " << mtr.Y() << " " << mtr.Z() << " (absolute) \n";
        for (int i = 0; i < mlevel; i++)
            GetLog() << "  ";
        gp_XYZ mtr2 = mshape.Location().Transformation().TranslationPart();
        GetLog() << "      pos at: " << mtr2.X() << " " << mtr2.Y() << " " << mtr2.Z() << " (.Location)\n";

        return true;
    }
};

void ChCascadeDoc::Dump(ChStreamOutAscii& mstream) {
    callback_CascadeDoc_dump adumper;
    this->ScanCascadeShapes(adumper);
}

int wildcard_compare(const char* wildcard, const char* string) {
    const char* cp = 0, * mp = 0;

    while ((*string) && (*wildcard != '*')) {
        if ((*wildcard != *string) && (*wildcard != '?')) {
            return 0;
        }
        wildcard++;
        string++;
    }

    while (*string) {
        if (*wildcard == '*') {
            if (!*++wildcard) {
                return 1;
            }
            mp = wildcard;
            cp = string + 1;
        } else {
            if ((*wildcard == *string) || (*wildcard == '?')) {
                wildcard++;
                string++;
            } else {
                wildcard = mp;
                string = cp++;
            }
        }
    }

    while (*wildcard == '*') {
        wildcard++;
    }
    return !*wildcard;
}

class callback_CascadeDoc_getnamed : public ChCascadeDoc::callback_CascadeDoc {
  public:
    char search_string[200];

    std::vector<std::string> level_names;
    std::vector<int> level_copy;

    bool set_location_to_root;

    bool res_found;
    TopoDS_Shape res_shape;
    TopoDS_Compound res_comp;
    BRep_Builder aBuilder;

    TopLoc_Location res_loc;
    int res_level;
    TDF_Label res_label;

    callback_CascadeDoc_getnamed(const char* name_with_path)
        : res_found(false), set_location_to_root(true), res_level(0) {
        aBuilder.MakeCompound(res_comp);

        strcpy(search_string, name_with_path);

        char* mpath = search_string;

        while (*mpath) {
            int copyid = -2;  // default -2 means use no copy number #
            char abuffer[200];
            char* ch = abuffer;
            while (*mpath && (*mpath != *"/") && (*mpath != *"#")) {
                *ch = *mpath;
                ++ch;
                ++mpath;
            }
            *ch = 0;

            if (*mpath == *"#") {
                mpath++;
                char numbuffer[200];
                char* nh = numbuffer;
                while (*mpath && (*mpath != *"/")) {
                    *nh = *mpath;
                    ++nh;
                    ++mpath;
                }
                copyid = atoi(numbuffer);
            }

            if (*mpath == *"/")
                mpath++;

            std::string levelname(abuffer);
            level_names.push_back(levelname);
            level_copy.push_back(copyid);
        }
    }

    virtual bool ForShape(TopoDS_Shape& mshape, TopLoc_Location& mloc, char* mname, int mlevel, TDF_Label& mlabel) {
        if (this->level_names.size() > mlevel) {
            if (wildcard_compare(level_names[mlevel].c_str(), mname)) {
                if (level_copy[mlevel] != -2) {
                    level_copy[mlevel] = level_copy[mlevel] - 1;
                    if (level_copy[mlevel] < -1)
                        level_copy[mlevel] = -1;
                }

                if ((level_copy[mlevel] == 0) || (level_copy[mlevel] == -2)) {
                    if (mlevel == this->level_names.size() - 1) {
                        // Found!!!

                        if (this->set_location_to_root)
                            mshape.Location(mloc);

                        // For single istance: only 1st found shape is considered
                        if (!res_found) {
                            res_found = true;
                            res_shape = mshape;
                            res_loc = mloc;
                            res_level = mlevel;
                            res_label = mlabel;
                        }

                        // Add to the shape compound, for multiple results
                        aBuilder.Add(res_comp, mshape);
                    }
                    return true;  // proceed!
                }
                return false;  // break, matching name but not #ID, not needed to go deeper in levels
            }
            return false;  // break, not matching, not needed to go deeper in levels
        } else
            return false;  // break, not needed to go deeper in levels than in string wildcard
    }
};

bool ChCascadeDoc::GetNamedShape(TopoDS_Shape& mshape, char* name, bool set_location_to_root, bool get_multiple) {
    callback_CascadeDoc_getnamed aselector(name);

    this->ScanCascadeShapes(aselector);

    if (aselector.res_found) {
        if (get_multiple)
            mshape = aselector.res_comp;
        else
            mshape = aselector.res_shape;
        return true;
    } else {
        mshape.Nullify();
        return false;
    }
}

bool ChCascadeDoc::GetRootShape(TopoDS_Shape& mshape, const int num) {
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool((*doc)->Main());
    TDF_LabelSequence root_labels;
    shapeTool->GetFreeShapes(root_labels);
    if (num > root_labels.Length())
        return false;
    mshape = shapeTool->GetShape(root_labels.Value(num));
    return true;
}

bool ChCascadeDoc::GetVolumeProperties(const TopoDS_Shape& mshape,   ///< pass the shape here
                                       const double density,         ///< pass the density here
                                       ChVector<>& center_position,  ///< get the position center, respect to shape pos.
                                       ChVector<>& inertiaXX,        ///< get the inertia diagonal terms
                                       ChVector<>& inertiaXY,        ///< get the inertia extradiagonal terms
                                       double& volume,               ///< get the volume
                                       double& mass                  ///< get the mass
                                       ) {
    if (mshape.IsNull())
        return false;

    GProp_GProps mprops;
    GProp_GProps vprops;
    BRepGProp::VolumeProperties(mshape, mprops);
    BRepGProp::VolumeProperties(mshape, vprops);

    mprops.Add(mprops, density);

    mass = mprops.Mass();
    volume = vprops.Mass();
    gp_Pnt G = mprops.CentreOfMass();
    gp_Mat I = mprops.MatrixOfInertia();

    center_position.x() = G.X();
    center_position.y() = G.Y();
    center_position.z() = G.Z();

    inertiaXX.x() = I(1, 1);
    inertiaXX.y() = I(2, 2);
    inertiaXX.z() = I(3, 3);
    inertiaXY.x() = I(1, 2);
    inertiaXY.y() = I(1, 3);
    inertiaXY.z() = I(2, 3);

    return true;
}

void ChCascadeDoc::FromCascadeToChrono(const TopLoc_Location& from_coord, ChFrame<>& to_coord) {
    gp_XYZ mtr = from_coord.Transformation().TranslationPart();
    to_coord.SetPos(ChVector<>(mtr.X(), mtr.Y(), mtr.Z()));

    gp_Mat mro = from_coord.Transformation().VectorialPart();
    ChMatrix33<> to_mat;

    to_mat(0, 0) = mro(1, 1);
    to_mat(0, 1) = mro(1, 2);
    to_mat(0, 2) = mro(1, 3);

    to_mat(1, 0) = mro(2, 1);
    to_mat(1, 1) = mro(2, 2);
    to_mat(1, 2) = mro(2, 3);

    to_mat(2, 0) = mro(3, 1);
    to_mat(2, 1) = mro(3, 2);
    to_mat(2, 2) = mro(3, 3);

    to_coord.SetRot(to_mat);
}

void ChCascadeDoc::FromChronoToCascade(const ChFrame<>& from_coord, TopLoc_Location& to_coord) {
    const ChVector<>& mpos = from_coord.GetPos();
    gp_Vec mtr(mpos.x(), mpos.y(), mpos.z());

    const ChMatrix33<>& from_mat = from_coord.GetA();

    ((gp_Trsf)(to_coord.Transformation()))
        .SetValues(from_mat(0, 0), from_mat(0, 1), from_mat(0, 2), mpos.x(), from_mat(1, 0), from_mat(1, 1),
                   from_mat(1, 2), mpos.y(), from_mat(2, 0), from_mat(2, 1), from_mat(2, 2), mpos.z()); //0, 0);
}



/// Create a ChBodyAuxRef with assets for the given TopoDS_Shape
std::shared_ptr<ChBodyAuxRef> ChCascadeDoc::CreateBodyFromShape(
                const TopoDS_Shape& mshape,   ///< pass the shape here
                const double density          ///< pass the density here
                )
{
    std::shared_ptr<ChBodyAuxRef> mbody(new ChBodyAuxRef);
    
    chrono::ChFrame<> frame_ref_to_abs;

    TopLoc_Location loc_shape_to_abs = mshape.Location();
    chrono::cascade::ChCascadeDoc::FromCascadeToChrono(loc_shape_to_abs, frame_ref_to_abs);   

    TopoDS_Shape objshape = mshape;
    objshape.Location(TopLoc_Location());  // Reset shape location to local ref csys (identity).

    chrono::ChVector<> mcog;
    chrono::ChVector<> minertiaXX;
    chrono::ChVector<> minertiaXY;
    double mvol;
    double mmass;
    chrono::cascade::ChCascadeDoc::GetVolumeProperties(objshape, density, mcog, minertiaXX, minertiaXY, mvol, mmass);

    mbody->SetFrame_REF_to_abs(frame_ref_to_abs);

    //mbody->SetFrame_COG_to_REF(frame_ref_to_abs.Invert() * mcog );

    chrono::ChFrame<>* frame_cog_to_ref = (chrono::ChFrame<>*)mbody.get();
    frame_cog_to_ref->SetPos(mcog);
    frame_cog_to_ref->SetRot(chrono::QUNIT);

    return mbody;
}


/////////////////////
