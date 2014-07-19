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
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.

class ChBodyEasySphere : public ChBody {

	
public:

		
		/// Creates a ChBody plus adds a visualization shape and, optionally,
		/// a collision shape. Mass and inertia are set automatically depending
		/// on density.
		/// Sphere is assumed with center at body reference coordsystem.
	ChBodyEasySphere ( double radius, double mdensity, bool collide = false, bool visual_asset = true)
	{	
		double mmass =  mdensity * ( (4.0/3.0) * CH_C_PI* pow (radius,3) );
		double inertia = (2.0/5.0)* mmass * pow (radius,2);

		this->SetDensity( (float)mdensity );
		this->SetMass( mmass );
		this->SetInertiaXX( ChVector<> ( inertia,inertia,inertia) );

		if (collide)
		{
			GetCollisionModel()->ClearModel();
			GetCollisionModel()->AddSphere(radius);  // radius, radius, height on y
			GetCollisionModel()->BuildModel();
			SetCollide(true);
		}
		if (visual_asset)
		{
			ChSharedPtr<ChSphereShape> vshape (new ChSphereShape() );
			vshape->GetSphereGeometry().rad = radius;
			this->AddAsset( vshape );
		}
	}

};


/// Easy-to-use class for quick creation of rigid bodies with a 
/// cylindrical shape. 
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.

class ChBodyEasyCylinder : public ChBody {

	
public:

		
		/// Creates a ChBody plus adds an optional visualization shape and, optionally,
		/// a collision shape. Mass and inertia are set automatically depending
		/// on density.
		/// Cylinder is assumed with body Y axis as vertical, and reference is at half height.
	ChBodyEasyCylinder ( double radius, double height, double mdensity, bool collide = false, bool visual_asset = true)
	{	
		double mmass =  mdensity * (CH_C_PI* pow (radius,2) * height);
		
		this->SetDensity( (float)mdensity );
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
		if (visual_asset)
		{
			ChSharedPtr<ChCylinderShape> vshape (new ChCylinderShape() );
			vshape->GetCylinderGeometry().p1 = ChVector<> (0,-height*0.5, 0);
			vshape->GetCylinderGeometry().p2 = ChVector<> (0, height*0.5, 0);
			vshape->GetCylinderGeometry().rad = radius;
			this->AddAsset( vshape );
		}
	}

};




/// Easy-to-use class for quick creation of rigid bodies with a 
/// box shape. 
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.

class ChBodyEasyBox : public ChBody {

	
public:

		
		/// Creates a ChBody plus adds an optional visualization shape and, optionally,
		/// a collision shape. Mass and inertia are set automatically depending
		/// on density.
		/// Box is assumed centered, ie. and reference of body in the middle.
	ChBodyEasyBox ( double Xsize, double Ysize, double Zsize, double mdensity, bool collide = false, bool visual_asset = true)
	{
		double mmass =  mdensity * (Xsize * Ysize * Zsize);

		this->SetDensity( (float)mdensity );
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
		if (visual_asset)
		{
			ChSharedPtr<ChBoxShape> vshape (new ChBoxShape() );
			vshape->GetBoxGeometry().SetLenghts( ChVector<> (Xsize,Ysize,Zsize) );
			this->AddAsset( vshape );
		}
	}

};



/// Easy-to-use class for quick creation of rigid bodies with a 
/// convex hull shape. 
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.

class ChBodyEasyConvexHull : public ChBody {

	
public:

		
		/// Creates a ChBody plus adds an optional visualization shape and, optionally,
		/// a collision shape. Mass and inertia are set automatically depending
		/// on density.
		/// Convex hull is defined with a set of points. 
	ChBodyEasyConvexHull ( std::vector< ChVector<> >& points, double mdensity, bool collide = false, bool visual_asset= true)
	{
		ChSharedPtr<ChTriangleMeshShape> vshape (new ChTriangleMeshShape() );
		ChConvexHullLibraryWrapper lh;
		lh.ComputeHull(points, vshape->GetMesh());
		if (visual_asset)
		{	
			this->AddAsset( vshape );
		}

		double mass;
		ChVector<> baricenter;
		ChMatrix33<> inertia;
		vshape->GetMesh().ComputeMassProperties(true, mass, baricenter, inertia);

		// Translate the convex hull baricenter so that body origin is also baricenter
		for (unsigned int i = 0; i< vshape->GetMesh().getCoordsVertices().size(); ++i)
			vshape->GetMesh().getCoordsVertices()[i] -= baricenter;

		this->SetDensity( (float)mdensity );
		this->SetMass( mass * mdensity );
		this->SetInertia( inertia * mdensity );
		
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


/// Easy-to-use class for quick creation of rigid bodies with a 
/// shape made of a cluster of spheres. 
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.
/// NOTE! mass and inertia are computed as if spheres are not intersecting! If you
/// need a more precise mass/inertia estimation when spheres are intersecting, change
/// mass and inertia after creation using more advanced formulas.

class ChBodyEasyClusterOfSpheres : public ChBody {

	
public:

		
		/// Creates a ChBody plus adds an optional visualization shape and, optionally,
		/// a collision shape. Mass and inertia are set automatically depending
		/// on density.
		/// The cluster of spheres will be displaced so that their center of mass 
		/// corresponds to the origin of the ChBody.
	ChBodyEasyClusterOfSpheres ( std::vector< ChVector<> >& positions,
								 std::vector< double >& radii, 
								 double mdensity, 
								 bool collide = false, 
								 bool visual_asset = true)
	{	
		assert (positions.size() == radii.size());

		double totmass = 0;
		ChMatrix33<> totinertia;
		ChVector<> baricenter = VNULL;
		totinertia.Reset();
		for (unsigned int i= 0; i< positions.size(); ++i)
		{
			double sphmass = mdensity * ( (4.0/3.0) * CH_C_PI* pow (radii[i],3) );
			double sphinertia = (2.0/5.0)* sphmass * pow (radii[i],2);
			baricenter = (baricenter*totmass + positions[i]*sphmass)/(totmass+sphmass);
			totmass += sphmass;
			// Huygens-Steiner parallel axis theorem:
			totinertia(0,0) += sphinertia + sphmass* (positions[i].Length2() - positions[i].x*positions[i].x);
			totinertia(1,1) += sphinertia + sphmass* (positions[i].Length2() - positions[i].y*positions[i].y);
			totinertia(2,2) += sphinertia + sphmass* (positions[i].Length2() - positions[i].z*positions[i].z);
			totinertia(0,1) += sphmass* ( - positions[i].x*positions[i].y);
			totinertia(0,2) += sphmass* ( - positions[i].x*positions[i].z);
			totinertia(1,2) += sphmass* ( - positions[i].y*positions[i].z);
			totinertia(1,0) = totinertia(0,1);
			totinertia(2,0) = totinertia(0,2);
			totinertia(2,1) = totinertia(1,2);
		}

		this->SetDensity( (float)mdensity );
		this->SetMass( totmass );
		this->SetInertia( totinertia );

		// Translate the cluster baricenter so that body origin is also baricenter
		std::vector< ChVector<> > offset_positions = positions;
		for (unsigned int i = 0; i< positions.size(); ++i)
			offset_positions[i] -= baricenter;

		if (collide)
		{
			GetCollisionModel()->ClearModel();
			for (unsigned int i= 0; i< positions.size(); ++i)
			{
				GetCollisionModel()->AddSphere(radii[i],offset_positions[i]);  // radius, radius, height on y
			}
			GetCollisionModel()->BuildModel();
			SetCollide(true);
		}
		if (visual_asset)
		{
			for (unsigned int i= 0; i< positions.size(); ++i)
			{
				ChSharedPtr<ChSphereShape> vshape (new ChSphereShape() );
				vshape->GetSphereGeometry().rad = radii[i];
				vshape->GetSphereGeometry().center = offset_positions[i];
				this->AddAsset( vshape );
			}
		}
	}

};



} // END_OF_NAMESPACE____


#endif
