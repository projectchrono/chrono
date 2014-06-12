//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHBODYEASY_H
#define CHBODYEASY_H


#include "physics/ChBody.h"
#include "assets/ChCylinderShape.h"
#include "assets/ChBoxShape.h"
#include "assets/ChSphereShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "collision/ChCCollisionUtils.h"

namespace chrono
{

// This header includes some easy-to-use bodies, that can be used
// to create ChBody objects that already include a visualization shape
// without the need of adding it afterward with AddAsset(). 
// Also, a collision shape is created and added automatically too, if 
// collision is needed.



/// Easy-to-use class for quick creation of rigid bodies with a 
/// spherical shape. 
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added,
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.

class ChBodyEasySphere : public ChBody {

	
public:

		
		/// Creates a ChBody plus adds a visualization shape and, optionally,
		/// a collision shape. Mass and inertia are set automatically depending
		/// on density.
		/// Sphere is assumed with center at body reference coordsystem.
	ChBodyEasySphere ( double radius, double mdensity, bool collide = false)
	{
		ChSharedPtr<ChSphereShape> vshape (new ChSphereShape() );
		vshape->GetSphereGeometry().rad = radius;
		this->AddAsset( vshape );
		
		double mmass =  mdensity * ( (4.0/3.0) * CH_C_PI* pow (radius,3) );
		double inertia = (2.0/5.0)* mmass * pow (radius,2);

		this->SetMass( mmass );
		this->SetInertiaXX( ChVector<> ( inertia,inertia,inertia) );

		if (collide)
		{
			GetCollisionModel()->ClearModel();
			GetCollisionModel()->AddSphere(radius);  // radius, radius, height on y
			GetCollisionModel()->BuildModel();
			SetCollide(true);
		}
	}

};


/// Easy-to-use class for quick creation of rigid bodies with a 
/// cylindrical shape. 
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added,
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.

class ChBodyEasyCylinder : public ChBody {

	
public:

		
		/// Creates a ChBody plus adds a visualization shape and, optionally,
		/// a collision shape. Mass and inertia are set automatically depending
		/// on density.
		/// Cylinder is assumed with body Y axis as vertical, and reference is at half height.
	ChBodyEasyCylinder ( double radius, double height, double mdensity, bool collide = false)
	{
		ChSharedPtr<ChCylinderShape> vshape (new ChCylinderShape() );
		vshape->GetCylinderGeometry().p1 = ChVector<> (0,-height*0.5, 0);
		vshape->GetCylinderGeometry().p2 = ChVector<> (0, height*0.5, 0);
		vshape->GetCylinderGeometry().rad = radius;
		this->AddAsset( vshape );
		
		double mmass =  mdensity * (CH_C_PI* pow (radius,2) * height);
		this->SetMass( mmass );
		this->SetInertiaXX( ChVector<> ( (1.0/12.0)*mmass * (3*pow (radius,2) + pow (height,2)) ,
										  0.5*mmass * pow (radius,2) ,
										 (1.0/12.0)*mmass * (3*pow (radius,2) + pow (height,2)) ));
		if (collide)
		{
			GetCollisionModel()->ClearModel();
			GetCollisionModel()->AddCylinder(radius,  radius,  height*0.5);  // radius x, radiusz, height on y
			GetCollisionModel()->BuildModel();
			SetCollide(true);
		}
	}

};




/// Easy-to-use class for quick creation of rigid bodies with a 
/// box shape. 
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added,
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.

class ChBodyEasyBox : public ChBody {

	
public:

		
		/// Creates a ChBody plus adds a visualization shape and, optionally,
		/// a collision shape. Mass and inertia are set automatically depending
		/// on density.
		/// Box is assumed centered, ie. and reference of body in the middle.
	ChBodyEasyBox ( double Xsize, double Ysize, double Zsize, double mdensity, bool collide = false)
	{
		ChSharedPtr<ChBoxShape> vshape (new ChBoxShape() );
		vshape->GetBoxGeometry().SetLenghts( ChVector<> (Xsize,Ysize,Zsize) );
		this->AddAsset( vshape );
		
		double mmass =  mdensity * (Xsize * Ysize * Zsize);
		this->SetMass( mmass );
		this->SetInertiaXX( ChVector<> (  (1.0/12.0) * mmass * ( pow(Ysize,2) + pow (Zsize,2) ) ,
										  (1.0/12.0) * mmass * ( pow(Xsize,2) + pow (Zsize,2) ),
										  (1.0/12.0) * mmass * ( pow(Xsize,2) + pow (Ysize,2) ) ));
		if (collide)
		{
			GetCollisionModel()->ClearModel();
			GetCollisionModel()->AddBox(Xsize*0.5,  Ysize*0.5,  Zsize*0.5);  // radius x, radius z, height on y
			GetCollisionModel()->BuildModel();
			SetCollide(true);
		}
	}

};



/// Easy-to-use class for quick creation of rigid bodies with a 
/// convex hull shape. 
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added,
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.

class ChBodyEasyConvexHull : public ChBody {

	
public:

		
		/// Creates a ChBody plus adds a visualization shape and, optionally,
		/// a collision shape. Mass and inertia are set automatically depending
		/// on density.
		/// Convex hull is defined with a set of points. 
	ChBodyEasyConvexHull ( std::vector< ChVector<> >& points, double mdensity, bool collide = false)
	{
		ChSharedPtr<ChTriangleMeshShape> vshape (new ChTriangleMeshShape() );
		ChConvexHullLibraryWrapper lh;
		lh.ComputeHull(points, vshape->GetMesh());
		this->AddAsset( vshape );

		double mass;
		ChVector<> baricenter;
		ChMatrix33<> inertia;
		vshape->GetMesh().ComputeMassProperties(true, mass, baricenter, inertia);

		// Translate the convex hull barycenter so that body origin is also baricenter
		for (unsigned int i = 0; i< vshape->GetMesh().getCoordsVertices().size(); ++i)
			vshape->GetMesh().getCoordsVertices()[i] -= baricenter;

		this->SetMass( mass * mdensity );
		this->SetInertia( &(inertia * mdensity) );
		
		if (collide)
		{
			// avoid passing to collision the inner points discarded by convex hull 
			// processor, so use mesh vertexes instead of all argument points
			std::vector< ChVector<> > points_reduced; 
			points_reduced.resize(vshape->GetMesh().getCoordsVertices().size());
			for (unsigned int i = 0; i< vshape->GetMesh().getCoordsVertices().size(); ++i)
				points_reduced[i] = vshape->GetMesh().getCoordsVertices()[i];

			GetCollisionModel()->SetSafeMargin(0.0001);
			//GetCollisionModel()->SetEnvelope(0.002);
			GetCollisionModel()->ClearModel();
			GetCollisionModel()->AddConvexHull(points_reduced);
			GetCollisionModel()->BuildModel();
			SetCollide(true);
		}
	}

};




} // END_OF_NAMESPACE____


#endif
