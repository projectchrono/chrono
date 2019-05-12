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

#include <cstdlib>
#include <cmath>
#include <cfloat>

#include "chrono/core/ChQuadrature.h"
#include "chrono/core/ChMatrixDynamic.h"

namespace chrono {

void ChQuadratureTables::glege_coef(ChMatrix<>& lcoef, int N) {
    int n, i;
    lcoef(0, 0) = lcoef(1, 1) = 1;
    for (n = 2; n <= N; n++) {
        lcoef(n, 0) = -(n - 1) * lcoef(n - 2, 0) / n;
        for (i = 1; i <= n; i++)
            lcoef(n, i) = ((2 * n - 1) * lcoef(n - 1, i - 1) - (n - 1) * lcoef(n - 2, i)) / n;
    }
}

double ChQuadratureTables::glege_eval(int n, double x, ChMatrix<>& lcoef) {
    int i;
    double s = lcoef(n, n);
    for (i = n; i; i--)
        s = s * x + lcoef(n, i - 1);
    return s;
}

double ChQuadratureTables::glege_diff(int n, double x, ChMatrix<>& lcoef) {
    return n * (x * glege_eval(n, x, lcoef) - glege_eval(n - 1, x, lcoef)) / (x * x - 1);
}

void ChQuadratureTables::glege_roots(ChMatrix<>& lcoef, int N, int ntable) {
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

void ChQuadratureTables::PrintTables() {
    GetLog() << "PrintTables: \n\n";

    for (unsigned int io = 0; io < Lroots.size(); ++io) {
        GetLog() << "\nOrder: " << Lroots[io].size() << "  at table n." << io << " with roots&weights: \n";
        for (unsigned int ir = 0; ir < Lroots[io].size(); ++ir)
            GetLog() << "  " << Lroots[io][ir];
        GetLog() << "\n";
        for (unsigned int ir = 0; ir < Weight[io].size(); ++ir)
            GetLog() << "  " << Weight[io][ir];
    }
}

ChQuadratureTables::ChQuadratureTables(int order_from, int order_to) {
    int N = order_to - order_from + 1;
    assert(N >= 1);

    Weight.resize(N);
    Lroots.resize(N);

    for (int io = 0; io < N; io++) {
        int Ncoef = io + order_from;

        Weight[io].resize(Ncoef);
        Lroots[io].resize(Ncoef);

        ChMatrixDynamic<> lcoef(Ncoef + 1, Ncoef + 1);
        lcoef.Reset();

        glege_coef(lcoef, Ncoef);
        glege_roots(lcoef, Ncoef, io);
    }
}


ChQuadratureTablesTriangle::ChQuadratureTablesTriangle() {
    // initialize table with precomputed weights and coordinates
    {  // i=0: 1pt  deg 1
        std::vector<double> xa= {1./3.};
        std::vector<double> ya= {1./3.};
        std::vector<double> wt= {1};
        this->LrootsU.push_back(xa);
        this->LrootsV.push_back(ya);
        this->Weight.push_back(wt);
    }
    {  // i=1: 3pt  deg 2
        std::vector<double> xa= {0.166666666666670,   0.166666666666670,   0.666666666666670};
        std::vector<double> ya= {0.166666666666670,   0.666666666666670,   0.166666666666670};
        std::vector<double> wt= {0.333333333333330,   0.333333333333330,   0.333333333333330};
        this->LrootsU.push_back(xa);
        this->LrootsV.push_back(ya);
        this->Weight.push_back(wt); 
    }
    {   // i=2: 4pt  deg 3
        std::vector<double> xa= {0.333333333333330,  0.200000000000000,   0.200000000000000,  0.600000000000000};
        std::vector<double> ya= {0.333333333333330,  0.200000000000000,   0.600000000000000,  0.200000000000000};
        std::vector<double> wt= {-0.56250000000000,  0.520833333333330,   0.520833333333330,  0.520833333333330};
        this->LrootsU.push_back(xa);
        this->LrootsV.push_back(ya);
        this->Weight.push_back(wt); 
    }
    {   // i=3: 6pt  deg  4
        std::vector<double> xa= {0.445948490915970,   0.445948490915970,   0.108103018168070,   0.091576213509770,   0.091576213509770,   0.816847572980460};
        std::vector<double> ya= {0.445948490915970,   0.108103018168070,   0.445948490915970,   0.091576213509770,   0.816847572980460,   0.091576213509770};
        std::vector<double> wt= {0.223381589678010,   0.223381589678010,   0.223381589678010,   0.109951743655320,   0.109951743655320,   0.109951743655320};
        this->LrootsU.push_back(xa);
        this->LrootsV.push_back(ya);
        this->Weight.push_back(wt);
    }
    {   // i=4: 7 pt deg 5
        std::vector<double> xa= {0.333333333333330,   0.470142064105110,   0.470142064105110,   0.059715871789770,   0.101286507323460,   0.101286507323460,   0.797426985353090};
        std::vector<double> ya= {0.333333333333330,   0.470142064105110,   0.059715871789770,   0.470142064105110,   0.101286507323460,   0.797426985353090,   0.101286507323460};
        std::vector<double> wt= {0.225000000000000,   0.132394152788510,   0.132394152788510,   0.132394152788510,   0.125939180544830,   0.125939180544830,   0.125939180544830};
        this->LrootsU.push_back(xa);
        this->LrootsV.push_back(ya);
        this->Weight.push_back(wt);
    }
}

ChQuadratureTablesTetrahedron::ChQuadratureTablesTetrahedron() {
    // initialize table with precomputed weights and coordinates
    {  // i=0: 1pt  deg 1
        std::vector<double> xa= {0.25}; 
        std::vector<double> ya= {0.25};
        std::vector<double> za= {0.25};
        std::vector<double> wt= {1};
        this->LrootsU.push_back(xa);
        this->LrootsV.push_back(ya);
        this->LrootsW.push_back(za);
        this->Weight.push_back(wt);
    }
    {  // i=1: 4pt  deg 2
        std::vector<double> xa= {0.5854101966249685, 0.1381966011250105, 0.1381966011250105, 0.1381966011250105}; 
        std::vector<double> ya= {0.1381966011250105, 0.1381966011250105, 0.1381966011250105, 0.5854101966249685};
        std::vector<double> za= {0.1381966011250105, 0.1381966011250105, 0.5854101966249685, 0.1381966011250105};
        std::vector<double> wt= {0.2500000000000000, 0.2500000000000000, 0.2500000000000000, 0.2500000000000000};
        this->LrootsU.push_back(xa);
        this->LrootsV.push_back(ya);
        this->LrootsW.push_back(za);
        this->Weight.push_back(wt);
    }
    {   // i=2: 5pt  deg 3
        std::vector<double> xa= {0.2500000000000000, 0.5000000000000000, 0.1666666666666667, 0.1666666666666667, 0.1666666666666667};
        std::vector<double> ya= { 0.2500000000000000, 0.1666666666666667, 0.1666666666666667, 0.1666666666666667, 0.5000000000000000};
        std::vector<double> za= { 0.2500000000000000, 0.1666666666666667, 0.1666666666666667, 0.5000000000000000, 0.1666666666666667};
        std::vector<double> wt= {-0.8000000000000000, 0.4500000000000000, 0.4500000000000000, 0.4500000000000000, 0.4500000000000000};
        this->LrootsU.push_back(xa);
        this->LrootsV.push_back(ya);
        this->LrootsW.push_back(za);
        this->Weight.push_back(wt);
    }
    {   // i=3: 11pt  deg 4
        std::vector<double> xa= {0.2500000000000000, 0.7857142857142857, 0.0714285714285714, 0.0714285714285714, 0.0714285714285714, 0.1005964238332008, 0.3994035761667992, 0.3994035761667992, 0.3994035761667992, 0.1005964238332008, 0.1005964238332008};
        std::vector<double> ya= {0.2500000000000000, 0.0714285714285714, 0.0714285714285714, 0.0714285714285714, 0.7857142857142857, 0.3994035761667992, 0.1005964238332008, 0.3994035761667992, 0.1005964238332008, 0.3994035761667992, 0.1005964238332008};
        std::vector<double> za= {0.2500000000000000, 0.0714285714285714, 0.0714285714285714, 0.7857142857142857, 0.0714285714285714, 0.3994035761667992, 0.3994035761667992, 0.1005964238332008, 0.1005964238332008, 0.1005964238332008, 0.3994035761667992};
        std::vector<double> wt= {-0.0789333333333333, 0.0457333333333333, 0.0457333333333333, 0.0457333333333333, 0.0457333333333333, 0.1493333333333333, 0.1493333333333333, 0.1493333333333333, 0.1493333333333333, 0.1493333333333333, 0.1493333333333333};
        this->LrootsU.push_back(xa);
        this->LrootsV.push_back(ya);
        this->LrootsW.push_back(za);
        this->Weight.push_back(wt);
    }
    {   // i=4: 15 pt
        std::vector<double> xa={0.2500000000000000, 0.0000000000000000, 0.3333333333333333, 0.3333333333333333, 0.3333333333333333, 0.7272727272727273, 0.0909090909090909, 0.0909090909090909, 0.0909090909090909, 0.4334498464263357, 0.0665501535736643, 0.0665501535736643, 0.0665501535736643, 0.4334498464263357, 0.4334498464263357};
        std::vector<double> ya={0.2500000000000000, 0.3333333333333333, 0.3333333333333333, 0.3333333333333333, 0.0000000000000000, 0.0909090909090909, 0.0909090909090909, 0.0909090909090909, 0.7272727272727273, 0.0665501535736643, 0.4334498464263357, 0.0665501535736643, 0.4334498464263357, 0.0665501535736643, 0.4334498464263357};
        std::vector<double> za={0.2500000000000000, 0.3333333333333333, 0.3333333333333333, 0.0000000000000000, 0.3333333333333333, 0.0909090909090909, 0.0909090909090909, 0.7272727272727273, 0.0909090909090909, 0.0665501535736643, 0.0665501535736643, 0.4334498464263357, 0.4334498464263357, 0.4334498464263357, 0.0665501535736643};
        std::vector<double> wt={0.1817020685825351, 0.0361607142857143, 0.0361607142857143, 0.0361607142857143, 0.0361607142857143, 0.0698714945161738, 0.0698714945161738, 0.0698714945161738, 0.0698714945161738, 0.0656948493683187, 0.0656948493683187, 0.0656948493683187, 0.0656948493683187, 0.0656948493683187, 0.0656948493683187};
        this->LrootsU.push_back(xa);
        this->LrootsV.push_back(ya);
        this->LrootsW.push_back(za);
        this->Weight.push_back(wt);
    }
}



////////////////////////////////////////////////////////////////

#define CH_QUADRATURE_STATIC_TABLES 10

ChQuadratureTables static_tables(1, CH_QUADRATURE_STATIC_TABLES);

ChQuadratureTablesTriangle static_tables_triangle; // only 5 tables

ChQuadratureTablesTetrahedron static_tables_tetrahedron; // only 5 tables



////////////////////////////////////////////////////////////////

ChQuadratureTables* ChQuadrature::GetStaticTables() {
    return &static_tables;
}

ChQuadratureTablesTriangle* ChQuadrature::GetStaticTablesTriangle() {
    return &static_tables_triangle;
}

ChQuadratureTablesTetrahedron* ChQuadrature::GetStaticTablesTetrahedron() {
    return &static_tables_tetrahedron;
}

}  // end namespace chrono
