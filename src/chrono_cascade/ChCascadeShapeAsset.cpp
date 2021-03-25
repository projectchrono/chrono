// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono_cascade/ChCascadeShapeAsset.h"

#include <TopoDS_Shape.hxx>

using namespace chrono;
using namespace cascade;


ChCascadeShapeAsset::ChCascadeShapeAsset() {

};


ChCascadeShapeAsset::ChCascadeShapeAsset(const TopoDS_Shape& ms) : mshape(ms) {

};

ChCascadeShapeAsset::~ChCascadeShapeAsset() {
};


void ChCascadeShapeAsset::ArchiveOUT(ChArchiveOut& marchive)
{
	// version number
	marchive.VersionWrite<ChCascadeShapeAsset>();
	// serialize parent class
	ChAsset::ArchiveOUT(marchive);
	// serialize all member data:
	//marchive << ...; //***TODO*** serialize shape chunk using Cascade xml or STEP formats
}

/// Method to allow de serialization of transient data from archives.
void ChCascadeShapeAsset::ArchiveIN(ChArchiveIn& marchive)
{
	// version number
	/*int version =*/ marchive.VersionRead<ChCascadeShapeAsset>();
	// deserialize parent class
	ChAsset::ArchiveIN(marchive);
	// stream in all member data:
	//marchive >> ...; //***TODO*** deserialize shape chunk using Cascade xml or STEP formats
}



/////////////////////
