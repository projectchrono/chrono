//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
//
// File author: Alessandro Tasora


#include <stdlib.h>
#include <math.h>
#include <float.h>
#include "ChQuadrature.h"


using namespace chrono;



void ChQuadratureTables::glege_coef(ChMatrix<>& lcoef, int N)
{
	int n, i;
	lcoef(0,0) = lcoef(1,1) = 1;
	for (n = 2; n <= N; n++) {
		lcoef(n,0) = -(n - 1) * lcoef(n - 2,0) / n;
		for (i = 1; i <= n; i++)
			lcoef(n,i) = ((2 * n - 1) * lcoef(n - 1,i - 1)
					 - (n - 1) * lcoef(n - 2,i) ) / n;
	}
}
 
double ChQuadratureTables::glege_eval(int n, double x, ChMatrix<>& lcoef)
{
	int i;
	double s = lcoef(n,n);
	for (i = n; i; i--)
		s = s * x + lcoef(n,i - 1);
	return s;
}
 
double ChQuadratureTables::glege_diff(int n, double x, ChMatrix<>& lcoef)
{
	return n * (x * glege_eval(n, x, lcoef) - glege_eval(n - 1, x, lcoef)) / (x * x - 1);
}
 
void ChQuadratureTables::glege_roots(ChMatrix<>& lcoef, int N, int ntable)
{
	int i;
	double x, x1;
	for (i = 1; i <= N; i++) {
		x = cos(CH_C_PI * (i - .25) / (N + .5));
		int iters = 0;
		do {
			++iters;
			x1 = x;
			x -= glege_eval(N, x, lcoef) / glege_diff(N, x, lcoef);
		} while (fabs(x - x1) > 1e-12 && iters < 25);

		Lroots[ntable][i - 1] = x;
 
		x1 = glege_diff(N, x, lcoef);
		Weight[ntable][i - 1] = 2 / ((1 - x * x) * x1 * x1);
	}
}
 
void ChQuadratureTables::PrintTables ()
{
	GetLog() << "PrintTables: \n\n";

	for (unsigned int io = 0; io < Lroots.size(); ++io)
	{
		GetLog() << "\nOrder: " << Lroots[io].size() << "  at table n." << io << " with roots&weights: \n";
		for (unsigned int ir = 0; ir < Lroots[io].size(); ++ir)
			GetLog() << "  " << Lroots[io][ir];
		GetLog() << "\n";
		for (unsigned int ir = 0; ir < Weight[io].size(); ++ir)
			GetLog() << "  " << Weight[io][ir];
	}
}


ChQuadratureTables::ChQuadratureTables(int order_from, int order_to)
{
	int N = order_to - order_from +1;
	assert(N>=1);

	Weight.resize(N);
	Lroots.resize(N);

	for (int io= 0; io< N; io++)
	{

		int Ncoef = io + order_from;

		Weight[io].resize(Ncoef);
		Lroots[io].resize(Ncoef);

		ChMatrixDynamic<> lcoef(Ncoef+1, Ncoef+1);

		glege_coef(lcoef, Ncoef);
		glege_roots(lcoef, Ncoef, io);
	}

}


////////////////////////////////////////////////////////////////

#define CH_QUADRATURE_STATIC_TABLES 10

ChQuadratureTables static_tables(1, CH_QUADRATURE_STATIC_TABLES);


////////////////////////////////////////////////////////////////



ChQuadratureTables* ChQuadrature::GetStaticTables()
{
	return &static_tables;
}




