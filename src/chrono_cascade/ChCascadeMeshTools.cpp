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
//   ChCascadeMeshTools.cpp
//
// ------------------------------------------------
//             http://www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono_cascade/ChCascadeMeshTools.h"

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
#include <Poly.hxx>
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
#include <TShort_Array1OfShortReal.hxx>

using namespace chrono;
using namespace cascade;
using namespace geometry;


void ChCascadeMeshTools::fillTriangleMeshFromCascadeFace(
        geometry::ChTriangleMeshConnected& chmesh,  ///< Mesh that will be filled with triangles
        const TopoDS_Face& F) {

    BRepAdaptor_Surface BS(F, Standard_False);
    Handle(BRepAdaptor_HSurface) gFace = new BRepAdaptor_HSurface(BS);
    GeomAbs_SurfaceType thetype = BS.GetType();

    Handle(Poly_Triangulation) T;
    TopLoc_Location theLocation;
    T = BRep_Tool::Triangulation(F, theLocation);
    
    // maybe a face has been already added to this mesh, so:
    int v_offset = (int)chmesh.m_vertices.size();

    if (!T.IsNull()) {

        const TColgp_Array1OfPnt& mNodes = T->Nodes();

        Poly::ComputeNormals(T);
        const TShort_Array1OfShortReal& mNormals = T->Normals();
            

        int ivert = 0;
        for (int j = mNodes.Lower(); j <= mNodes.Upper(); j++) {
            gp_Pnt p;
            gp_Dir pn;
            p = mNodes(j).Transformed(theLocation.Transformation());

            chrono::ChVector<> pos(p.X(), p.Y(), p.Z());
            chrono::ChVector<> nor(mNormals((j-1)*3+1), mNormals((j-1)*3+2), mNormals((j-1)*3+3));
            if (F.Orientation() == TopAbs_REVERSED)
                nor*= -1;

            chmesh.m_vertices.push_back(pos);
            chmesh.m_normals.push_back(nor);
            
            ivert++;
        }

        int itri = 0;
        for (int j = T->Triangles().Lower(); j <= T->Triangles().Upper(); j++) {
            Standard_Integer n[3];
            if (F.Orientation() == TopAbs_REVERSED)
                T->Triangles()(j).Get(n[0], n[2], n[1]);
            else
                T->Triangles()(j).Get(n[0], n[1], n[2]);
            int ia = v_offset + (n[0]) - 1;
            int ib = v_offset + (n[1]) - 1;
            int ic = v_offset + (n[2]) - 1;

            chmesh.m_face_v_indices.push_back(ChVector<int>(ia,ib,ic));
            chmesh.m_face_n_indices.push_back(ChVector<int>(ia,ib,ic));

            itri++;
        }
    }
        
}

/*
void ChCascadeMeshTools::fillTriangleMeshFromCascadeFace(ChTriangleMesh& chmesh, const TopoDS_Face& F) {
    BRepAdaptor_Surface BS(F, Standard_False);
    Handle(BRepAdaptor_HSurface) gFace = new BRepAdaptor_HSurface(BS);

    Handle(Poly_Triangulation) T;
    TopLoc_Location theLocation;
    T = BRep_Tool::Triangulation(F, theLocation);

    if (!T.IsNull()) {
        Standard_Integer n[3];
        gp_Pnt p;
        gp_Vec V;

        const TColgp_Array1OfPnt& mNodes = T->Nodes();
        
        Poly::ComputeNormals(T);
        const TShort_Array1OfShortReal& mNormals = T->Normals();     

        int ivert = 0;
        for (int j = mNodes.Lower(); j <= mNodes.Upper(); j++) {
            p = mNodes(j).Transformed(theLocation.Transformation());
            V.SetX(p.X());
            V.SetY(p.Y());
            V.SetZ(p.Z());

            ivert++;
        }
        int itri = 0;
        for (int j = T->Triangles().Lower(); j <= T->Triangles().Upper(); j++) {
            if (F.Orientation() == TopAbs_REVERSED)
                T->Triangles()(itri + 1).Get(n[0], n[2], n[1]);
            else
                T->Triangles()(itri + 1).Get(n[0], n[1], n[2]);
            int ia = (n[0]) - 1;
            int ib = (n[1]) - 1;
            int ic = (n[2]) - 1;

            p = mNodes(ia + 1).Transformed(theLocation.Transformation());
            V.SetX(p.X());
            V.SetY(p.Y());
            V.SetZ(p.Z());
            ChVector<> cv1(V.X(), V.Y(), V.Z());
            p = mNodes(ib + 1).Transformed(theLocation.Transformation());
            V.SetX(p.X());
            V.SetY(p.Y());
            V.SetZ(p.Z());
            ChVector<> cv2(V.X(), V.Y(), V.Z());
            p = mNodes(ic + 1).Transformed(theLocation.Transformation());
            V.SetX(p.X());
            V.SetY(p.Y());
            V.SetZ(p.Z());
            ChVector<> cv3(V.X(), V.Y(), V.Z());
            chmesh.addTriangle(cv1, cv2, cv3);

            itri++;
        }
    }
}
*/

void ChCascadeMeshTools::fillTriangleMeshFromCascade(ChTriangleMeshConnected& chmesh,
                                                     const TopoDS_Shape& mshape,
                                                     double deflection,
                                                     bool   relative_deflection,
                                                     double angulardeflection) {
    BRepTools::Clean(mshape);
    BRepMesh_IncrementalMesh M(mshape, deflection, relative_deflection , angulardeflection, true);
    // GetLog() << "    ..tesselation done \n";
    
    chmesh.Clear();

    // Loop on faces..
    TopExp_Explorer ex;
    for (ex.Init(mshape, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face& F = TopoDS::Face(ex.Current());
        fillTriangleMeshFromCascadeFace(chmesh, F);
    }
}

void ChCascadeMeshTools::fillObjFileFromCascade(ChStreamOutAscii& objfile,
                                                const TopoDS_Shape& mshape,
                                                double deflection,
                                                bool  relative_deflection,
                                                double angulardeflection) {
    BRepTools::Clean(mshape);
    BRepMesh_IncrementalMesh M(mshape, deflection, relative_deflection , angulardeflection, true);
    // GetLog() << "    ..tesselation done \n";

    TopExp_Explorer ex;
    int vertface = 0;
    int vertshift = 1;

    objfile << "# Wavefront .obj file created from Chrono::Engine\n\n";

    // Loop on faces..
    for (ex.Init(mshape, TopAbs_FACE); ex.More(); ex.Next()) {
        // fcount++;

        const TopoDS_Face& F = TopoDS::Face(ex.Current());

        BRepAdaptor_Surface BS(F, Standard_False);
        Handle(BRepAdaptor_HSurface) gFace = new BRepAdaptor_HSurface(BS);

        Handle(Poly_Triangulation) T;
        TopLoc_Location theLocation;
        T = BRep_Tool::Triangulation(F, theLocation);

        if (!T.IsNull()) {
            Standard_Integer n[3];
            gp_Pnt p;
            gp_Vec V;

            const TColgp_Array1OfPnt& mNodes = T->Nodes();

            Poly::ComputeNormals(T);
            const TShort_Array1OfShortReal& mNormals = T->Normals();  

            int ivert = 0;
            for (int j = mNodes.Lower(); j <= mNodes.Upper(); j++) {
                gp_Pnt p;
                gp_Dir pn;
                p = mNodes(j).Transformed(theLocation.Transformation());
                chrono::ChVector<> pos(p.X(), p.Y(), p.Z());
                chrono::ChVector<> nor(mNormals((j-1)*3+1), mNormals((j-1)*3+2), mNormals((j-1)*3+3));
                if (F.Orientation() == TopAbs_REVERSED)
                    nor*= -1;

                char buff[200];
                sprintf(buff, "v %0.9f %0.9f %0.9f\r\n", pos.x(), pos.y(), pos.z());
                objfile << buff;
                sprintf(buff, "vn %0.9f %0.9f %0.9f\r\n", nor.x(), nor.y(), nor.z());
                objfile << buff;
                vertface++;

                ivert++;
            }
            int itri = 0;
            for (int j = T->Triangles().Lower(); j <= T->Triangles().Upper(); j++) {
                if (F.Orientation() == TopAbs_REVERSED)
                    T->Triangles()(itri + 1).Get(n[0], n[2], n[1]);
                else
                    T->Triangles()(itri + 1).Get(n[0], n[1], n[2]);
                int ia = (n[0]) - 1;
                int ib = (n[1]) - 1;
                int ic = (n[2]) - 1;

                char buff[200];
                sprintf(buff, "f %d//%d %d//%d %d//%d\r\n", (ia + vertshift), (ia + vertshift), (ib + vertshift),
                        (ib + vertshift), (ic + vertshift), (ic + vertshift));
                objfile << buff;

                itri++;
            }
        }
        vertshift += vertface;
        vertface = 0;

    }  // end face loop
}




/////////////////////
