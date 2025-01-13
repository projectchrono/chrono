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


//////////////////////////////////////////////////////////////////////////////////////////7


}  // end namespace multidomain
}  // end namespace chrono
