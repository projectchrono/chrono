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

#include "chrono_cascade/ChCascadeDoc.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono_cascade/ChCascadeMeshTools.h"

#include <TopoDS_Shape.hxx>
#include <BRep_Builder.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <Interface_Static.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <TDataStd_Name.hxx>

using namespace chrono;
using namespace cascade;

// -----------------------------------------------------------------------------
// Utils
// -----------------------------------------------------------------------------
static bool RecurseCascadeDoc(TDF_Label label,
                              opencascade::handle<XCAFDoc_ShapeTool>& shapeTool,
                              TopLoc_Location& parentloc,
                              int level,
                              ChCascadeDoc::ScanShapesCallback& callback) {
    TDF_LabelSequence child_labels;
    Standard_Boolean is_assembly;

    is_assembly = shapeTool->GetComponents(label, child_labels, 0);

    char mstring[200] = "no name";
    Standard_PCharacter mchastr = mstring;

    // access name
    opencascade::handle<TDataStd_Name> N;
    if (label.FindAttribute(TDataStd_Name::GetID(), N)) {
        N->Get().ToUTF8CString(mchastr);
    }

    TDF_Label reflabel;
    Standard_Boolean isref = shapeTool->GetReferredShape(label, reflabel);

    if (isref)  // ..maybe it references some shape: the name is stored there...
    {
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
        // execute the callback function
        if (!callback.ForShape(rShape, absloc, mstring, level, label))
            return false;  // skip children recursion
    }

    // Recurse all children
    if (is_assembly) {
        for (Standard_Integer j = 1; j <= child_labels.Length(); j++) {
            TDF_Label clabel = child_labels.Value(j);
            RecurseCascadeDoc(clabel, shapeTool, absloc, (level + 1), callback);
        }
    }

    // If it is a reference, recurse all children of reference
    if (isref) {
        TDF_LabelSequence refchild_labels;
        shapeTool->GetComponents(reflabel, refchild_labels, 0);
        for (Standard_Integer j = 1; j <= refchild_labels.Length(); j++) {
            TDF_Label clabel = refchild_labels.Value(j);
            RecurseCascadeDoc(clabel, shapeTool, absloc, (level + 1), callback);
        }
    }

    return true;
}

static int WildcardCompare(const char* wildcard, const char* string) {
    const char *cp = 0, *mp = 0;

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

class DumpShapesCallback : public ChCascadeDoc::ScanShapesCallback {
  public:
    DumpShapesCallback(std::ostream& ostream) : m_ostream(ostream) {};

    virtual bool ForShape(TopoDS_Shape& shape,
                          TopLoc_Location& loc,
                          const std::string& name,
                          int level,
                          TDF_Label& label) override {
        for (int i = 0; i < level; i++)
            m_ostream << "  ";
        m_ostream << "- Name: " << name;

        if (level == 0)
            m_ostream << " (root)";
        m_ostream << "\n";

        for (int i = 0; i < level; i++)
            m_ostream << "  ";
        gp_XYZ mtr = loc.Transformation().TranslationPart();
        m_ostream << "      pos at: " << mtr.X() << " " << mtr.Y() << " " << mtr.Z() << " (absolute) \n";
        for (int i = 0; i < level; i++)
            m_ostream << "  ";
        gp_XYZ mtr2 = shape.Location().Transformation().TranslationPart();
        m_ostream << "      pos at: " << mtr2.X() << " " << mtr2.Y() << " " << mtr2.Z() << " (.Location)\n";

        return true;
    }

  private:
    std::ostream& m_ostream;
};

class GetNamedShapesCallback : public ChCascadeDoc::ScanShapesCallback {
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

    GetNamedShapesCallback(const char* name_with_path) : res_found(false), set_location_to_root(true), res_level(0) {
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
                copyid = std::atoi(numbuffer);
            }

            if (*mpath == *"/")
                mpath++;

            std::string levelname(abuffer);
            level_names.push_back(levelname);
            level_copy.push_back(copyid);
        }
    }

    virtual bool ForShape(TopoDS_Shape& shape,
                          TopLoc_Location& loc,
                          const std::string& name,
                          int mlevel,
                          TDF_Label& label) override {
        if ((int)level_names.size() > mlevel) {
            if (WildcardCompare(level_names[mlevel].c_str(), name.c_str())) {
                if (level_copy[mlevel] != -2) {
                    level_copy[mlevel] = level_copy[mlevel] - 1;
                    if (level_copy[mlevel] < -1)
                        level_copy[mlevel] = -1;
                }

                if ((level_copy[mlevel] == 0) || (level_copy[mlevel] == -2)) {
                    if (mlevel == (int)level_names.size() - 1) {
                        // Found!!!

                        if (this->set_location_to_root)
                            shape.Location(loc);

                        // For single istance: only 1st found shape is considered
                        if (!res_found) {
                            res_found = true;
                            res_shape = shape;
                            res_loc = loc;
                            res_level = mlevel;
                            res_label = label;
                        }

                        // Add to the shape compound, for multiple results
                        aBuilder.Add(res_comp, shape);
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

// -----------------------------------------------------------------------------
// ChCascadeDoc
// -----------------------------------------------------------------------------
ChCascadeDoc::ChCascadeDoc() {
    doc = new opencascade::handle<TDocStd_Document>;
    (*doc) = new TDocStd_Document("MyStepDoc");
}

ChCascadeDoc::~ChCascadeDoc() {
    if (doc)
        delete doc;
    doc = 0;
}

void ChCascadeDoc::ScanCascadeShapes(ScanShapesCallback& callback) const {
    TopLoc_Location rootloc;
    rootloc.Identity();

    opencascade::handle<XCAFDoc_ShapeTool> shapeTool = XCAFDoc_DocumentTool::ShapeTool((*doc)->Main());
    TDF_LabelSequence root_labels;
    shapeTool->GetFreeShapes(root_labels);
    for (Standard_Integer i = 1; i <= root_labels.Length(); i++) {
        TDF_Label label = root_labels.Value(i);
        int level = 0;
        RecurseCascadeDoc(label, shapeTool, rootloc, level, callback);
    }
}

bool ChCascadeDoc::LoadSTEP(const std::string& filename) {
    STEPCAFControl_Reader cafreader;

    if (!Interface_Static::SetCVal("xstep.cascade.unit", "M"))
        std::cerr << "\n\n ERROR SETTING 'M' UNITS!!!..   \n\n" << std::endl;

    IFSelect_ReturnStatus aStatus = cafreader.ReadFile(filename.c_str());

    if (aStatus == IFSelect_RetDone) {
        /*Standard_Boolean aRes =*/cafreader.Transfer((*doc));
        return true;
    }
    return false;
}

void ChCascadeDoc::Dump(std::ostream& stream) const {
    DumpShapesCallback dump(stream);
    ScanCascadeShapes(dump);
}

bool ChCascadeDoc::GetNamedShape(TopoDS_Shape& shape,
                                 const std::string& name,
                                 bool set_location_to_root,
                                 bool get_multiple) const {
    GetNamedShapesCallback selector(name.c_str());

    ScanCascadeShapes(selector);

    if (selector.res_found) {
        if (get_multiple)
            shape = selector.res_comp;
        else
            shape = selector.res_shape;
        return true;
    } else {
        shape.Nullify();
        return false;
    }
}

bool ChCascadeDoc::GetRootShape(TopoDS_Shape& shape, const int num) const {
    opencascade::handle<XCAFDoc_ShapeTool> shapeTool = XCAFDoc_DocumentTool::ShapeTool((*doc)->Main());
    TDF_LabelSequence root_labels;
    shapeTool->GetFreeShapes(root_labels);
    if (num > root_labels.Length())
        return false;
    shape = shapeTool->GetShape(root_labels.Value(num));
    return true;
}

bool ChCascadeDoc::GetVolumeProperties(const TopoDS_Shape& shape,    ///< pass the shape here
                                       const double density,         ///< pass the density here
                                       ChVector3d& center_position,  ///< get the position center, respect to shape pos.
                                       ChVector3d& inertiaXX,        ///< get the inertia diagonal terms
                                       ChVector3d& inertiaXY,        ///< get the inertia extradiagonal terms
                                       double& volume,               ///< get the volume
                                       double& mass                  ///< get the mass
) {
    if (shape.IsNull())
        return false;

    GProp_GProps vprops;

    // default density = 1;

    BRepGProp::VolumeProperties(shape, vprops);

    mass = vprops.Mass() * density;
    volume = vprops.Mass();
    gp_Pnt G = vprops.CentreOfMass();
    gp_Mat I = vprops.MatrixOfInertia();

    center_position.x() = G.X();
    center_position.y() = G.Y();
    center_position.z() = G.Z();

    inertiaXX.x() = I(1, 1);
    inertiaXX.y() = I(2, 2);
    inertiaXX.z() = I(3, 3);
    inertiaXY.x() = I(1, 2);
    inertiaXY.y() = I(1, 3);
    inertiaXY.z() = I(2, 3);

    inertiaXX *= density;
    inertiaXY *= density;

    return true;
}

void ChCascadeDoc::ConvertFrameCascadeToChrono(const TopLoc_Location& from_coord, ChFramed& to_coord) {
    gp_XYZ mtr = from_coord.Transformation().TranslationPart();
    to_coord.SetPos(ChVector3d(mtr.X(), mtr.Y(), mtr.Z()));

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

void ChCascadeDoc::ConvertFrameChronoToCascade(const ChFramed& from_coord, TopLoc_Location& to_coord) {
    const ChVector3d& mpos = from_coord.GetPos();
    gp_Vec mtr(mpos.x(), mpos.y(), mpos.z());

    const ChMatrix33<>& from_mat = from_coord.GetRotMat();

    gp_Trsf castrasf;
    castrasf.SetValues(from_mat(0, 0), from_mat(0, 1), from_mat(0, 2), mpos.x(), from_mat(1, 0), from_mat(1, 1),
                       from_mat(1, 2), mpos.y(), from_mat(2, 0), from_mat(2, 1), from_mat(2, 2), mpos.z());

    to_coord = TopLoc_Location(castrasf);

    //((gp_Trsf)(to_coord.Transformation()))
    //    .SetValues(from_mat(0, 0), from_mat(0, 1), from_mat(0, 2), mpos.x(), from_mat(1, 0), from_mat(1, 1),
    //               from_mat(1, 2), mpos.y(), from_mat(2, 0), from_mat(2, 1), from_mat(2, 2), mpos.z()); //0, 0);
}