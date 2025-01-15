// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#include<array>
#include "chrono_multidomain/ChDomainBuilder.h"


namespace chrono {
namespace multidomain {



ChDomainBuilderSlices::ChDomainBuilderSlices(
											int tot_domains,
											double mmin,
											double mmax,
											ChAxis maxis,
											bool build_master) {
	this->axis = maxis;
	domains_bounds.push_back(-1e34);
	for (int islice = 0; islice < (tot_domains - 1); ++islice) {
		domains_bounds.push_back(mmin + (islice + 1.0) * (mmax - mmin) / (double)tot_domains);
	}
	domains_bounds.push_back(1e34);

	m_build_master = build_master;
};

ChDomainBuilderSlices::ChDomainBuilderSlices(
											std::vector<double> axis_cuts,
											ChAxis maxis,
											bool build_master) {
	this->axis = maxis;
	domains_bounds.push_back(-1e34);
	for (double dcut : axis_cuts) {
		domains_bounds.push_back(dcut);
	}
	domains_bounds.push_back(1e34);

	m_build_master = build_master;
}


std::shared_ptr<ChDomain> ChDomainBuilderSlices::BuildDomain(
	ChSystem* msys,
	int this_rank) {
	
	assert(this_rank >= 0);
	assert(this_rank < GetTotSlices());

	auto domain = chrono_types::make_shared<ChDomainSlice>(msys, this_rank, domains_bounds[this_rank], domains_bounds[this_rank + 1], axis);

	if (this_rank > 0) {
		int low_domain_rank = this_rank - 1;
		auto low_domain = chrono_types::make_shared<ChDomainSlice>(nullptr, low_domain_rank, domains_bounds[this_rank-1], domains_bounds[this_rank], axis);
		ChDomainInterface& low_interf = domain->GetInterfaces()[low_domain->GetRank()]; // insert domain interface in unordered map
		low_interf.side_IN = domain.get();
		low_interf.side_OUT = low_domain;
	}
	if (this_rank < domains_bounds.size() - 2) {
		int hi_domain_rank = this_rank + 1;
		auto hi_domain = chrono_types::make_shared<ChDomainSlice>(nullptr, hi_domain_rank, domains_bounds[this_rank+1], domains_bounds[this_rank+2], axis);
		ChDomainInterface& hi_interf = domain->GetInterfaces()[hi_domain->GetRank()];  // insert domain interface in unordered map
		hi_interf.side_IN = domain.get();
		hi_interf.side_OUT = hi_domain;
	}
	
	if (this->m_build_master) {
		int imaster_rank = this->GetTotRanks() - 1;
		auto master_dom = chrono_types::make_shared<ChDomainMaster>(nullptr, imaster_rank);
		ChDomainInterface& master_interf = domain->GetInterfaces()[imaster_rank];  // insert domain interface in unordered map
		master_interf.side_IN = domain.get();
		master_interf.side_OUT = master_dom;
	}

	return domain;
}


std::shared_ptr<ChDomain> ChDomainBuilderSlices::BuildMasterDomain(
	ChSystem* msys) {

	assert(this->m_build_master);
	int this_rank = this->GetTotRanks() - 1;

	auto domain = chrono_types::make_shared<ChDomainMaster>(msys, this_rank);
	
	for (int i = 0; i < this->GetTotSlices(); ++i) {
		int in_domain_rank = i;
		auto in_domain = chrono_types::make_shared<ChDomainSlice>(nullptr, in_domain_rank, domains_bounds[i], domains_bounds[i + 1], axis);
		ChDomainInterface& hi_interf = domain->GetInterfaces()[in_domain_rank];  // insert domain interface in unordered map
		hi_interf.side_IN = domain.get();
		hi_interf.side_OUT = in_domain;
	}

	return domain;
}


//////////////////////////////////////////////////////////////////////////////////////////

ChDomainBuilderGrid::ChDomainBuilderGrid(
	int tot_domains_X,
	double mmin_X,
	double mmax_X,
	int tot_domains_Y,
	double mmin_Y,
	double mmax_Y,
	int tot_domains_Z,
	double mmin_Z,
	double mmax_Z,
	bool build_master) {

	domains_bounds_X.push_back(-1e34);
	for (int islice = 0; islice < (tot_domains_X - 1); ++islice) {
		domains_bounds_X.push_back(mmin_X + (islice + 1.0) * (mmax_X - mmin_X) / (double)tot_domains_X);
	}
	domains_bounds_X.push_back(1e34);

	domains_bounds_Y.push_back(-1e34);
	for (int islice = 0; islice < (tot_domains_Y - 1); ++islice) {
		domains_bounds_Y.push_back(mmin_Y + (islice + 1.0) * (mmax_Y - mmin_Y) / (double)tot_domains_Y);
	}
	domains_bounds_Y.push_back(1e34);

	domains_bounds_Z.push_back(-1e34);
	for (int islice = 0; islice < (tot_domains_Z - 1); ++islice) {
		domains_bounds_Z.push_back(mmin_Z + (islice + 1.0) * (mmax_Z - mmin_Z) / (double)tot_domains_Z);
	}
	domains_bounds_Z.push_back(1e34);

	m_build_master = build_master;
};

ChDomainBuilderGrid::ChDomainBuilderGrid(
	std::vector<double> axis_cuts_X,
	std::vector<double> axis_cuts_Y,
	std::vector<double> axis_cuts_Z,
	bool build_master) {

	domains_bounds_X.push_back(-1e34);
	for (double dcut : axis_cuts_X) {
		domains_bounds_X.push_back(dcut);
	}
	domains_bounds_X.push_back(1e34);

	domains_bounds_Y.push_back(-1e34);
	for (double dcut : axis_cuts_Y) {
		domains_bounds_Y.push_back(dcut);
	}
	domains_bounds_Y.push_back(1e34);

	domains_bounds_Z.push_back(-1e34);
	for (double dcut : axis_cuts_Z) {
		domains_bounds_Z.push_back(dcut);
	}
	domains_bounds_Z.push_back(1e34);

	m_build_master = build_master;
}


std::shared_ptr<ChDomain> ChDomainBuilderGrid::BuildDomain(
	ChSystem* msys,
	int this_rank) {

	assert(this_rank >= 0);
	assert(this_rank < GetTotSlices());

	int i, j, k;
	this->FromRankToIJKindexes(this_rank, i,j,k);

	// Create this domain from rank number

	auto domain = chrono_types::make_shared<ChDomainBox>(msys,
		this_rank,
		ChAABB(ChVector3d(domains_bounds_X[i],   domains_bounds_Y[j],   domains_bounds_Z[k]),
			   ChVector3d(domains_bounds_X[i+1], domains_bounds_Y[j+1], domains_bounds_Z[k+1]))
	);

	// Create the interfaces to 3x3=27 domains surrounding this domain, and the 27 domain placeholders
	
	for (int si = i-1; si <= i+1; ++si) {
		for (int sj = j-1; sj <= j+1; ++sj) {
			for (int sk = k-1; sk <= k+1; ++sk) {
				if (!((si == i) && (sj == j) && (sk == k))			// skip the center, i.e this domain
					&& (si >= 0) && (si < this->GetSlicesX())   // skip surrounding if this already at border of lattice (x dir)
					&& (sj >= 0) && (sj < this->GetSlicesY())   // skip surrounding if this already at border of lattice (y dir)
					&& (sk >= 0) && (sk < this->GetSlicesZ())   // skip surrounding if this already at border of lattice (z dir)
					) {
					int outer_domain_rank = this->FromIJKindexesToRank(si,sj,sk);
					auto outer_domain = chrono_types::make_shared<ChDomainBox>(
						nullptr, 
						outer_domain_rank, 
						ChAABB(ChVector3d(domains_bounds_X[si], domains_bounds_Y[sj], domains_bounds_Z[sk]),
							   ChVector3d(domains_bounds_X[si + 1], domains_bounds_Y[sj + 1], domains_bounds_Z[sk + 1]))
					);

					ChDomainInterface& low_interf = domain->GetInterfaces()[outer_domain->GetRank()]; // insert domain interface in unordered map
					low_interf.side_IN = domain.get();
					low_interf.side_OUT = outer_domain;
				}
			}
		}
	}
	
	if (this->m_build_master) {
		int imaster_rank = this->GetTotRanks() - 1;
		auto master_dom = chrono_types::make_shared<ChDomainMaster>(nullptr, imaster_rank);
		ChDomainInterface& master_interf = domain->GetInterfaces()[imaster_rank];  // insert domain interface in unordered map
		master_interf.side_IN = domain.get();
		master_interf.side_OUT = master_dom;
	}

	return domain;
}


std::shared_ptr<ChDomain> ChDomainBuilderGrid::BuildMasterDomain(
	ChSystem* msys) {

	assert(this->m_build_master);
	int this_rank = this->GetTotRanks() - 1;

	// Create the master domain 
	auto domain = chrono_types::make_shared<ChDomainMaster>(msys, this_rank);

	// Create interfaces from master to all N domains enclosed in master, and their N domain placeholders
	for (int in_domain_rank = 0; in_domain_rank < this->GetTotSlices(); ++in_domain_rank) {
		int si, sj, sk;
		this->FromRankToIJKindexes(in_domain_rank, si, sj, sk);
		auto in_domain = chrono_types::make_shared<ChDomainBox>(
			nullptr,
			in_domain_rank,
			ChAABB(ChVector3d(domains_bounds_X[si], domains_bounds_Y[sj], domains_bounds_Z[sk]),
				   ChVector3d(domains_bounds_X[si + 1], domains_bounds_Y[sj + 1], domains_bounds_Z[sk + 1]))
		);
		ChDomainInterface& sub_interf = domain->GetInterfaces()[in_domain_rank];  // insert domain interface in unordered map
		sub_interf.side_IN = domain.get();
		sub_interf.side_OUT = in_domain;
	}

	return domain;
}


//////////////////////////////////////////////////////////////////////////////////////////7


}  // end namespace multidomain
}  // end namespace chrono
