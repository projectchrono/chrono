//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Bryan Peterson
// =============================================================================
// This demo creates a single HMMWV tire model using ANCF shell elements
// that starts rolling immediately by  using consistent input data for
// the initial conditions. Steady-state LuGre friction model is used.
// Global direction of rolling is assumed along the X axis
// [LuGre contact formulation needs to be modified for other cases]

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsValidation.h"
#include "chrono/core/ChMathematics.h"
#include "chrono_mkl/ChLcpMklSolver.h"
#include "physics/ChLoadContainer.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_irrlicht/ChBodySceneNode.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrAppInterface.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include <irrlicht.h>
#include <cmath>

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace fea;
using namespace chrono::irrlicht;

using namespace irr;
using namespace irr::scene;

class ChCoulombFriction : public ChLoadCustomMultiple {
  public:
    ChCoulombFriction(std::vector<std::shared_ptr<ChLoadable>>& mloadables) : ChLoadCustomMultiple(mloadables){};

    int NumContact;
    const double Mu0 = 0.6;
    double ContactLine;
    ChVector<> NetContactForce;
    ChVector<> ContactForce;

    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) {
        ChMatrixDynamic<double> FlexPos(6, (int)loadables.size());
        ChMatrixDynamic<double> FlexVel(6, (int)loadables.size());
        double Kg;
        double Cg;
        double DeltaDis;
        double DeltaVel;
        double Mu;

        if (state_x && state_w) {
            FlexPos.Reset();
            FlexVel.Reset();
            NetContactForce.x = 0.0;
            NetContactForce.y = 0.0;
            NetContactForce.z = 0.0;
            for (int ie = 0; ie < loadables.size(); ie++) {
                ChVector<> P1 = state_x->ClipVector(6 * ie, 0);
                ChVector<> P1d = state_x->ClipVector(6 * ie + 3, 0);
                ChVector<> V1 = state_w->ClipVector(6 * ie, 0);
                ChVector<> V1d = state_w->ClipVector(6 * ie + 3, 0);

                FlexPos(0, ie) = P1.x;
                FlexPos(1, ie) = P1.y;
                FlexPos(2, ie) = P1.z;
                FlexPos(3, ie) = P1d.x;
                FlexPos(4, ie) = P1d.y;
                FlexPos(5, ie) = P1d.z;

                FlexVel(0, ie) = V1.x;
                FlexVel(1, ie) = V1.y;
                FlexVel(2, ie) = V1.z;
                FlexVel(3, ie) = V1d.x;
                FlexVel(4, ie) = V1d.y;
                FlexVel(5, ie) = V1d.z;
            }

            if (NumContact == 0) {
                Kg = 1.0e5;
            } else {
                Kg = 8.0e6 / double(NumContact);
            }
            Cg = Kg * 0.001;

            for (int ie = 0; ie < loadables.size(); ie++) {
                if (FlexPos(2, ie) < ContactLine) {
                    ContactForce.x = 0.0;
                    ContactForce.y = 0.0;
                    ContactForce.z = 0.0;
                    // GetLog() << i << "\n" << DisFlex(0, i) << "\n" << DisFlex(1, i) << "\n" << DisFlex(2, i) << "\n";

                    DeltaDis = FlexPos(2, ie) - ContactLine;
                    DeltaVel = FlexVel(2, ie);
                    ContactForce.z = -Kg * DeltaDis - Cg * DeltaVel * sqrt(DeltaDis * DeltaDis);

                    // Coulomb Friction
                    double RelVelX = FlexVel(0, ie);
                    double NormVelX = sqrt(RelVelX * RelVelX);
                    Mu = Mu0 * atan(2.0 * RelVelX) * 2.0 / CH_C_PI;
                    ContactForce.x = -Mu * (ContactForce.z);
                    // GetLog() << "RelVelX: " << RelVelX << "\nMu: " << Mu << "\nNormVelX: " << NormVelX << "\nFx: " <<
                    // ContactFRC(0, i) << "\nFz: " << ContactFRC(2, i) << "\natan: " << atan(2.0*RelVelX) << "\n\n";
                    if (NormVelX == 0.0) {
                        ContactForce.x = 0.0;
                    }

                    double RelVelY = FlexVel(1, ie);
                    double NormVelY = sqrt(RelVelY * RelVelY);
                    Mu = Mu0 * atan(2.0 * RelVelY) * 2.0 / CH_C_PI;
                    ContactForce.y = -Mu * (ContactForce.z);
                    if (NormVelY == 0.0) {
                        ContactForce.y = 0.0;
                    }

                    if (ContactForce.z != 0.0) {
                        NetContactForce.x += ContactForce.x;
                        NetContactForce.y += ContactForce.y;
                        NetContactForce.z += ContactForce.z;
                    }

                    this->load_Q(6 * ie) = ContactForce.x;
                    this->load_Q(6 * ie + 1) = ContactForce.y;
                    this->load_Q(6 * ie + 2) = ContactForce.z;
                }
            }
        }  // end of if(state) loop

    }  // end of Compute_Q

    virtual bool IsStiff() { return true; }
};

// The current implementation of steady-state LuGre formulation assumes
// that the tire is rolling along the global X direction. This needs to be
// revisited for other scenarios: Integration in vehicle, changes of direction, etc.
class ChLoaderLuGre : public ChLoadCustomMultiple {
  public:
    ChLoaderLuGre(std::vector<std::shared_ptr<ChLoadable>>& mloadables) : ChLoadCustomMultiple(mloadables){};

    std::shared_ptr<ChBody> mRim;
    int NumContact;
    const int NumDofRigid = 14;    // 1 rim, 1 ground, 7 Dofs each
    const int NumDofFlex = 18000;  // 3000 nodes x 6 Dofs each
    const int NumLuGreZ = 20;
    const double Mu0 = 0.6;
    double ContactLine;
    ChMatrixNM<double, 6, 120> NodeContactForce;
    ChVector<> NetContactForce;

    //////////Merge and Sort Function////////////
    // Identification/mapping of contacting nodes. Determines the nodes at the leading/trailing edge
    // x1, x2, x3, arrays of coordinates in x, y, z direction, respecticely. "n" number of nodes into contact
    // x1, x2, x3 provide sorted values for nodal coordinates (which are used as a grid in the LuGre formulation)
    void Mergesort_Real(ChVectorDynamic<double>& x1, ChVectorDynamic<double>& x2, ChVectorDynamic<double>& x3, int n) {
        ChMatrixDynamic<double> x(n, 3);
        int i, m, l, j, k;

        for (int ii = 0; ii < n; ii++) {
            x(ii, 0) = x1(ii);
            x(ii, 1) = x2(ii);
            x(ii, 2) = x3(ii);
        }

        if (n <= 1) {
            return;
        }
        m = n / 2;
        l = n - m;

        ChMatrixDynamic<double> buffer(m, 3);
        ChVectorDynamic<double> yVec1(m);
        ChVectorDynamic<double> yVec2(m);
        ChVectorDynamic<double> yVec3(m);
        ChVectorDynamic<double> zVec1(l);
        ChVectorDynamic<double> zVec2(l);
        ChVectorDynamic<double> zVec3(l);

        for (int ii = 0; ii < n; ii++) {
            if (ii < m) {
                yVec1(ii) = x(ii, 0);
                yVec2(ii) = x(ii, 1);
                yVec3(ii) = x(ii, 2);
            } else if (ii >= m) {
                zVec1(ii - m) = x(ii, 0);
                zVec2(ii - m) = x(ii, 1);
                zVec3(ii - m) = x(ii, 2);
            }
        }

        Mergesort_Real(yVec1, yVec2, yVec3, m);
        Mergesort_Real(zVec1, zVec2, zVec3, l);

        for (int ii = 0; ii < n; ii++) {
            if (ii < m) {
                x(ii, 0) = yVec1(ii);
                x(ii, 1) = yVec2(ii);
                x(ii, 2) = yVec3(ii);

                buffer(ii, 0) = x(ii, 0);
                buffer(ii, 1) = x(ii, 1);
                buffer(ii, 2) = x(ii, 2);
            } else if (ii >= m) {
                x(ii, 0) = zVec1(ii - m);
                x(ii, 1) = zVec2(ii - m);
                x(ii, 2) = zVec3(ii - m);
            }
        }

        j = m;
        i = 0;
        k = 0;

        while (i < m && j < n) {
            if (buffer(i, 0) <= x(j, 0)) {
                x(k, 0) = buffer(i, 0);
                x(k, 1) = buffer(i, 1);
                x(k, 2) = buffer(i, 2);
                k++;
                i++;
            } else {
                x(k, 0) = x(j, 0);
                x(k, 1) = x(j, 1);
                x(k, 2) = x(j, 2);
                k++;
                j++;
            }
        }
        while (i < m) {
            x(k, 0) = buffer(i, 0);
            x(k, 1) = buffer(i, 1);
            x(k, 2) = buffer(i, 2);
            k++;
            i++;
        }

        for (int ii = 0; ii < n; ii++) {
            x1(ii) = x(ii, 0);
            x2(ii) = x(ii, 1);
            x3(ii) = x(ii, 2);
        }
    }

    /////////////Calculate dLength///////////////
    // Calculate the effective length of LuGre element
    void Calculate_dLength(double Vec1x,
                           double Vec1y,
                           double Vec2x,
                           double Vec2y,
                           double Vec3x,
                           double Vec3y,
                           double& Length) {
        double TempR;
        double TempR1;
        ChVectorDynamic<double> TempVecR(2);

        TempVecR(0) = Vec2x - Vec1x;
        TempVecR(1) = Vec2y - Vec1y;
        TempR = sqrt(TempVecR(0) * TempVecR(0) + TempVecR(1) * TempVecR(1));
        TempVecR(0) = Vec3x - Vec2x;
        TempVecR(1) = Vec3y - Vec2y;
        TempR1 = sqrt(TempVecR(0) * TempVecR(0) + TempVecR(1) * TempVecR(1));

        Length = (TempR + TempR1) * 0.5;
    }

    // Cubic Spline Functions //
    // r83_np_fs is used to factor and solve an R83 System
    // n: Order of the linear system; a(3,n): tridiagonal matrix to be factored;
    // b(n): right hand side of the linear system; x(n): solution to the linear system.
    void r83_np_fs(int n, ChMatrixDynamic<double>& a, ChVectorDynamic<double> b, ChVectorDynamic<double>& x) {
        // ChVectorDynamic<double> x(n);
        double xmult;
        for (int i = 0; i < n; i++) {
            if (a(1, i) == 0.0) {
                GetLog() << "\nR83_NP_FS - Fatal error!\n A(1," << i << ") = 0";
                system("pause");
            }
            x(i) = b(i);
        }

        for (int i = 1; i < n; i++) {
            xmult = a(2, i - 1) / a(1, i - 1);
            a(1, i) = a(1, i) - xmult * a(0, i);
            x(i) = x(i) - xmult * x(i - 1);
        }
        x(n - 1) = x(n - 1) / a(1, n - 1);

        for (int j = n - 2; j >= 0; j--) {
            x(j) = (x(j) - a(0, j + 1) * x(j + 1)) / a(1, j);
        }
    }

    // r8vec_bracket searches a sorted vector for successive brackets of a value
    // n: input vector dimension; x(n): vector to be sorted into ascending order;
    // xval: value to be bracketed; left/right: indices such that x(left) <= xval <= x(right).
    void r8vec_bracket(int n, ChVectorDynamic<double> x, double xval, int& left, int& right) {
        for (int i = 1; i < n - 1; i++) {
            if (xval < x(i)) {
                left = i - 1;
                right = i;
                return;
            }
        }
        left = n - 2;
        right = n - 1;
    }

    // Computes the second derivatives of a piecewise cubic spline
    // n: number of data points > 2; t(n): specfied data points in increasing order;
    // y(n): values to be interpolated; ybcbeg/ybcend: left/right boundary value
    // ibcbeg/ibcend: left/right boundary condition flag
    //				= 0: quadratic over first/last interval;
    //				= 1: first derivative at left/right endpoint = ybcbeg/ybcend;
    //				= 2: second derivative at left/right endpoint = ybcbeg/ybcend;
    // ypp(n): second derivatives of cubic spline.
    void spline_cubic_set(int n,
                          ChVectorDynamic<double> t,
                          ChVectorDynamic<double> y,
                          int ibcbeg,
                          double ybcbeg,
                          int ibcend,
                          double ybcend,
                          ChVectorDynamic<double>& ypp) {
        ChMatrixDynamic<double> a(3, n);
        ///////////////
        /////Check/////
        ///////////////
        if (n <= 1) {
            GetLog()
                << "\nSPLINE_CUBIC_SET - Fatal error!\nThe number of knots must be at least 2.\nThe input value of N = "
                << n;
            system("pause");
        }
        for (int i = 0; i < n - 1; i++) {
            if (t(i + 1) <= t(i)) {
                GetLog() << "\nSPLINE_CUBIC_SET - Fatal error!\nThe knots must be strictly increasing, but T(" << i
                         << ") = " << t(i) << " T(" << i + 1 << ") = " << t(i + 1);
                system("pause");
            }
        }
        if (ibcbeg == 0) {
            ypp(0) = 0.0;
            a(1, 0) = 1.0;
            a(0, 1) = -1.0;
        } else if (ibcbeg == 1) {
            ypp(0) = (y(1) - y(0)) / (t(1) - t(0)) - ybcbeg;
            a(1, 0) = (t(1) - t(0)) / 3.0;
            a(0, 1) = (t(1) - t(0)) / 6.0;
        } else if (ibcbeg == 2) {
            ypp(0) = ybcbeg;
            a(1, 0) = 1.0;
            a(0, 1) = 0.0;
        } else {
            GetLog() << "\nSPLINE_CUBIC_SET - Fatal error!\nThe boundary flag IBCBEG must be 0, 1, or 2.\nThe input "
                        "value of IBCBEG = "
                     << ibcbeg;
            system("pause");
        }

        ////////////////////////////////////////
        ////Set the intermediate equations./////
        ////////////////////////////////////////
        for (int i = 1; i < n - 1; i++) {
            ypp(i) = ((y(i + 1) - y(i)) / (t(i + 1) - t(i))) - ((y(i) - y(i - 1)) / (t(i) - t(i - 1)));
            a(2, i - 1) = (t(i) - t(i - 1)) / 6.0;
            a(1, i) = (t(i + 1) - t(i - 1)) / 3.0;
            a(0, i + 1) = (t(i + 1) - t(i)) / 6.0;
        }
        ///////////////////////////////////
        //////Set the last equation.///////
        ///////////////////////////////////
        if (ibcend == 0) {
            ypp(n - 1) = 0.0;
            a(2, n - 2) = -1.0;
            a(1, n - 1) = 1.0;
        } else if (ibcend == 1) {
            ypp(n - 1) = ybcend - (y(n - 1) - y(n - 2)) / (t(n - 1) - t(n - 2));
            a(2, n - 2) = (t(n - 1) - t(n - 2)) / 6.0;
            a(1, n - 1) = (t(n - 1) - t(n - 2)) / 3.0;
        } else if (ibcend == 2) {
            ypp(n - 1) = ybcend;
            a(2, n - 2) = 0.0;
            a(1, n - 1) = 1.0;
        } else {
            GetLog() << "\nSPLINE_CUBIC_SET - Fatal error!\nThe boundary flag IBCBEG must be 0, 1, or 2.\nThe input "
                        "value of IBCBEG = "
                     << ibcbeg;
            system("pause");
        }

        if (n == 2 && ibcbeg == 0 && ibcend == 0) {
            ypp(0) = 0.0;
            ypp(1) = 0.0;
        } else {
            r83_np_fs(n, a, ypp, ypp);
        }
    }

    // Evaluates piecewise cubic spline at a specified point.
    // n: number of data points; t(n): the knot values; y(n): the data values at the knots;
    // ypp(n): second derivatives of the cubic spline generated from spline_cubic_set;
    // tval: a point on the spline to be evaluated;
    // yval/ypval/yppval: The value of the spline, first derivative, and second derivative.
    void spline_cubic_val(int n,
                          ChVectorDynamic<double> t,
                          ChVectorDynamic<double> y,
                          ChVectorDynamic<double> ypp,
                          double tval,
                          double& yval,
                          double& ypval,
                          double& yppval) {
        int left = 0;
        int right = 0;
        double dt;
        double h;

        r8vec_bracket(n, t, tval, left, right);

        dt = tval - t(left);
        h = t(right) - t(left);

        yval = y(left) +
               dt * ((y(right) - y(left)) / h - (ypp(right) / 6.0 + ypp(left) / 3.0) * h +
                     dt * (0.5 * ypp(left) + dt * ((ypp(right) - ypp(left)) / (6.0 * h))));
        ypval = (y(right) - y(left)) / h - (ypp(right) / 6.0 + ypp(left) / 3.0) * h +
                dt * (ypp(left) + dt * (0.5 * (ypp(right) - ypp(left)) / h));
        yppval = ypp(left) + dt * (ypp(right) - ypp(left)) / h;
    }
    // Spline calculation Reference:
    // Carl deBoor,
    // A Practical Guide to Splines,
    // Springer, 2001,
    // IBSN:0387953663

    void G_Function(double Vr, double& G_Func) {
        double h;
        double Vc;
        double Vs_x = 3.5;
        double Alpha_x = 0.6;
        double LuGreBeta_x = 0.0;
        double Mub_x = 0.7;
        double Mus_x = 1.6;

        Vc = pow(sqrt((Vr / Vs_x) * (Vr / Vs_x)), Alpha_x);
        h = -LuGreBeta_x * sqrt((Vr / Vs_x) * (Vr / Vs_x)) + Mub_x;
        G_Func = h + (Mus_x - h) * exp(-1.0 * pow(sqrt((Vr / Vs_x) * (Vr / Vs_x)), Alpha_x));
    }

    void CalculateNormalForce(int StartFlag,
                              int NumElemsX,
                              ChMatrixDynamic<double> DisFlex,
                              ChMatrixDynamic<double> VelFlex,
                              ChMatrixDynamic<double>& ContactFRC,
                              int& SequentialCheck,
                              int& NumContactNode) {
        double Kg;
        double Cg;
        double DeltaDis;
        double DeltaVel;

        // Kg: Penalty contact stiffness; Cg: Damping of ground contact
        if (NumContact == 0) {
            Kg = 1.0e5;
        } else {
            Kg = 8.0e6 / double(NumContact);
        }
        Cg = Kg * 0.001;

        for (int i = 0; i < NumElemsX; i++) {
            if (StartFlag == 0) {
                if (DisFlex(2, i) < ContactLine) {
                    // GetLog() << i << "\n" << DisFlex(0, i) << "\n" << DisFlex(1, i) << "\n" << DisFlex(2, i) << "\n";

                    if (SequentialCheck == 2) {
                        SequentialCheck = 3;
                    } else if (SequentialCheck == 0) {
                        SequentialCheck = 1;
                    }
                    NumContactNode++;
                    DeltaDis = DisFlex(2, i) - ContactLine;
                    DeltaVel = VelFlex(2, i);
                    ContactFRC(2, i) = -Kg * DeltaDis - Cg * DeltaVel * sqrt(DeltaDis * DeltaDis);
                } else {
                    if (SequentialCheck == 1) {
                        SequentialCheck = 2;
                    }
                }
            } else if (StartFlag == 1) {
                if (DisFlex(2, i) < ContactLine) {
                    // GetLog() << i << "\n" << DisFlex(0, i) << "\n" << DisFlex(1, i) << "\n" << DisFlex(2, i) << "\n";

                    if (SequentialCheck == 1) {
                        SequentialCheck = 2;
                    }
                    NumContactNode++;
                    DeltaDis = DisFlex(2, i) - ContactLine;
                    DeltaVel = VelFlex(2, i);
                    ContactFRC(2, i) = -Kg * DeltaDis - Cg * DeltaVel * sqrt(DeltaDis * DeltaDis);

                } else {
                    if (SequentialCheck == 0) {
                        SequentialCheck = 1;
                    } else if (SequentialCheck == 2) {
                        SequentialCheck = 3;
                    }
                }
            }
        }
    }

    void LuGre_SteadyState(int NumElemsX,
                           int NumElemsY,
                           ChMatrixDynamic<double> DisRigid,
                           ChMatrixDynamic<double> VelRigid,
                           ChMatrixDynamic<double> DisFlex,
                           ChMatrixDynamic<double> VelFlex,
                           ChMatrixDynamic<double>& ContactFRC,
                           ChMatrixDynamic<double>& StockSlipVel1,
                           double& StockROMG1) {
        double MinRot = 0.0;
        double MaxRot = 0.0;
        double rot = 0.0;
        int RimBody = 1;
        double RxOmg = 0.0;
        double Length = 0.0;

        int SequentialCheck = 0;
        int NumContactNode = 0;
        int StartFlag = 0;
        double TempR = 0.0;
        ChVectorDynamic<double> TempVecR(3);
        int Count = 0;
        int NDC = 7;  // Rigid Body Dofs
        ChVectorDynamic<double> RearContactPoint(3);
        ChVectorDynamic<double> FrontContactPoint(3);
        ChVectorDynamic<double> ypp(NumLuGreZ);
        double G_Func = 0.0;
        double Gx_Func = 0.0;
        double Gy_Func = 0.0;
        double Dummy = 0.0;
        int MaxRotIndx = 0;

        // LuGre parameters
        double ParaA = 0.0;
        double ParaB = 0.0;
        double sgm0_x = 260.0;
        double sgm1_x = 0.0;
        double sgm2_x = 0.0;
        double sgm0_y = 130.0;
        double sgm1_y = 0.0;
        double sgm2_y = 0.0;

        // VelFlex: Contain all velocities of the nodes in one entire circumferential strip (loop)
        // MaxRot: Most positive value of the vertical velocity of all nodes in entire loop
        // MinRot: Most negative value of the vertical velocity of all nodes in entire loop
        MinRot = VelFlex(2, 0);
        MaxRot = VelFlex(2, 0);
        for (int i = 0; i < NumElemsX; i++) {
            if (VelFlex(2, i) < MinRot) {
                MinRot = VelFlex(2, i);
            }
            if (VelFlex(2, i) > MaxRot) {
                MaxRot = VelFlex(2, i);
                MaxRotIndx = i;
            }
        }
        for (int i = 0; i < NumElemsX; i++) {
            if (MaxRot == VelFlex(2, i)) {
                rot = VelFlex(0, MaxRotIndx) - VelRigid(0, RimBody);
                i = NumElemsX;
            }
        }
        // RxOmg: Measure of an average linear velocity (along the forward direction) of the nodes in the entire loop
        if (rot >= 0.0) {
            RxOmg = (MaxRot - MinRot) / 2.0;
        } else if (rot < 0.0) {
            RxOmg = -(MaxRot - MinRot) / 2.0;
        }

        SequentialCheck = 0;  //
        NumContactNode = 0;
        if (DisFlex(2, 0) < ContactLine)  // To determine if the first node in the entire loop is in contact
        {
            StartFlag = 1;
        }

        CalculateNormalForce(StartFlag, NumElemsX, DisFlex, VelFlex, ContactFRC, SequentialCheck, NumContactNode);

        ChVectorDynamic<int> TempContactNodeNum(NumContactNode);
        ChVectorDynamic<double> TempVecNode(NumContactNode);
        ChVectorDynamic<double> TempFn(NumContactNode + 2);
        ChMatrixDynamic<double> TempNodeInfo(NumContactNode + 2, 3);
        ChVectorDynamic<double> TempNodeInfo1(NumContactNode + 2);
        // ChVectorDynamic<double> TempNodeInfo2(NumContactNode + 2); // Not needed for calculation
        // ChVectorDynamic<double> TempNodeInfo3(NumContactNode + 2);
        ChVectorDynamic<double> ReplaceMap(NumContactNode);
        if (NumContactNode >= 2 &&
            SequentialCheck < 3)  // If SequentialCheck >= 3, discontinuity within contact patch (buckling)
        {
            ChMatrixDynamic<double> NodePos(NumContactNode, 3);
            ChMatrixDynamic<double> NodeTransPos(NumContactNode, 3);
            ChVectorDynamic<double> NodeTransPosX(NumContactNode);
            ChVectorDynamic<double> NodeTransPosY(NumContactNode);
            ChMatrixDynamic<double> NodeVel(NumContactNode, 3);
            ChMatrixDynamic<double> NodeTransVel(NumContactNode, 3);
            ChVectorDynamic<double> NodeTransVelX(NumContactNode);
            ChVectorDynamic<double> NodeTransVelY(NumContactNode);
            ChVectorDynamic<double> NodeFn(NumContactNode);
            ChMatrixDynamic<double> LocalLuGreForce(NumContactNode, 3);
            ChMatrixDynamic<double> LuGreForce(NumContactNode, 3);
            ChMatrixDynamic<double> ZetaPos(NumLuGreZ, 2);
            ChVectorDynamic<double> ZetaPosX(NumLuGreZ);
            ChMatrixDynamic<double> ZetaVr(NumLuGreZ, 2);
            ChVectorDynamic<double> dLength(NumLuGreZ);
            ChVectorDynamic<double> dLength_Node(NumContactNode);
            ChVectorDynamic<double> ZetaFn(NumLuGreZ);
            ChMatrixDynamic<double> ZetaF(NumLuGreZ, 2);
            ChVectorDynamic<double> ZetaFX(NumLuGreZ);
            ChVectorDynamic<double> ZetaFY(NumLuGreZ);
            ChMatrixDynamic<double> ZLuGre(NumLuGreZ, 2);
            ChMatrixDynamic<double> dZLuGre(NumLuGreZ, 2);
            ChVectorDynamic<int> ContactNodeNum(NumContactNode);

            Count = 0;
            for (int i = 0; i < NumElemsX; i++)  // NumElemsX: Number of nodes in the entire loop
            {
                if (DisFlex(2, i) < ContactLine) {
                    NodePos(Count, 0) = DisFlex(0, i);
                    NodePos(Count, 1) = DisFlex(1, i);
                    NodePos(Count, 2) = DisFlex(2, i);
                    NodeVel(Count, 0) = VelFlex(0, i);
                    NodeVel(Count, 1) = VelFlex(1, i);
                    NodeVel(Count, 2) = VelFlex(2, i);
                    TempContactNodeNum(Count) = i;
                    StockSlipVel1(0, i) = VelFlex(0, i);
                    StockSlipVel1(1, i) = VelFlex(1, i);
                    Count++;
                }
            }

            // Position
            TempVecR.Reset();  //  Locate nodes from a reference within contact patch

            for (int i = 0; i < NumContactNode; i++) {
                TempVecR(0) = NodePos(i, 0) - DisRigid(0, RimBody);
                TempVecR(1) = NodePos(i, 1) - DisRigid(1, RimBody);
                TempVecR(2) = NodePos(i, 2);
                NodeTransPos(i, 0) = TempVecR(0);  // TempVecR should be able to be removed
                NodeTransPos(i, 1) = TempVecR(1);
                NodeTransPos(i, 2) = TempVecR(2);
                NodeTransPosX(i) = NodeTransPos(i, 0);
                NodeTransPosY(i) = NodeTransPos(i, 1);
                ReplaceMap(i) = i;
            }

            // Reorder number of contact nodes for cubic spline interpolation in ReplaceMap
            Mergesort_Real(NodeTransPosX, NodeTransPosY, ReplaceMap, NumContactNode);

            for (int i = 0; i < NumContactNode; i++) {
                NodeTransPos(i, 0) = NodeTransPosX(i);
                NodeTransPos(i, 1) = NodeTransPosY(i);
            }
            // Estimate actual trailing/leading point from the nodes that surround the contact patch boundary (one in
            // one out)
            RearContactPoint(0) =
                NodeTransPos(0, 0) -
                sqrt((NodeTransPos(1, 0) - NodeTransPos(0, 0)) * (NodeTransPos(1, 0) - NodeTransPos(0, 0))) / 2.0;
            RearContactPoint(1) =
                NodeTransPos(0, 1) -
                sqrt((NodeTransPos(1, 1) - NodeTransPos(0, 1)) * (NodeTransPos(1, 1) - NodeTransPos(0, 1))) / 2.0;
            RearContactPoint(2) = ContactLine;
            FrontContactPoint(0) = NodeTransPos(NumContactNode - 1, 0) +
                                   sqrt((NodeTransPos(NumContactNode - 1, 0) - NodeTransPos(NumContactNode - 2, 0)) *
                                        (NodeTransPos(NumContactNode - 1, 0) - NodeTransPos(NumContactNode - 2, 0))) /
                                       2.0;
            FrontContactPoint(1) = NodeTransPos(NumContactNode - 1, 1) +
                                   sqrt((NodeTransPos(NumContactNode - 1, 1) - NodeTransPos(NumContactNode - 2, 1)) *
                                        (NodeTransPos(NumContactNode - 1, 1) - NodeTransPos(NumContactNode - 2, 1))) /
                                       2.0;
            FrontContactPoint(2) = ContactLine;

            StockROMG1 = sqrt((RxOmg - VelRigid(0, RimBody)) * (RxOmg - VelRigid(0, RimBody)));

            // Velocity and contact node number
            for (int i = 0; i < NumContactNode; i++) {
                NodeTransVel(i, 0) = NodeVel(int(ReplaceMap(i)), 0);
                NodeTransVelX(i) = NodeTransVel(i, 0);
                NodeTransVel(i, 1) = NodeVel(int(ReplaceMap(i)), 1);
                NodeTransVelY(i) = NodeTransVel(i, 1);
                NodeTransVel(i, 2) = NodeVel(int(ReplaceMap(i)), 2);

                ContactNodeNum(i) = TempContactNodeNum(int(ReplaceMap(i)));
            }
            // ATTENTION: LENGTH CALCULATED ASSUMING VEHICLE RUNNING ALONG GLOBAL X AXIS
            Length = std::abs(RearContactPoint(0) - FrontContactPoint(0));
            TempR = Length / double(NumLuGreZ - 1);  // Distance between LuGre discretization points (NumLuGreZ: Number
                                                     // of "Lugre" elements within a strip)
            ZetaPos(0, 0) = -Length / 2.0;
            ZetaPosX(0) = ZetaPos(0, 0);
            for (int i = 1; i < NumLuGreZ; i++) {
                ZetaPos(i, 0) = ZetaPos(0, 0) + TempR * double(i);
                ZetaPosX(i) = ZetaPos(i, 0);
            }

            // In order to calculate ZetaPos
            for (int i = 0; i < NumLuGreZ; i++) {
                spline_cubic_set(NumContactNode, NodeTransPosX, NodeTransPosY, 2, 0.0, 2, 0.0, ypp);
                spline_cubic_val(NumContactNode, NodeTransPosX, NodeTransPosY, ypp, ZetaPos(i, 0), ZetaPos(i, 1), Dummy,
                                 Dummy);
            }

            // In order to calculate dLength(i)
            for (int i = 0; i < NumLuGreZ; i++) {
                if (i == 0) {
                    Calculate_dLength(ZetaPos(i + 1, 0), ZetaPos(i + 1, 1), ZetaPos(i, 0), ZetaPos(i, 1),
                                      ZetaPos(i + 1, 0), ZetaPos(i + 1, 1), dLength(i));
                } else if (i == NumLuGreZ - 1) {
                    Calculate_dLength(ZetaPos(i - 1, 0), ZetaPos(i - 1, 1), ZetaPos(i, 0), ZetaPos(i, 1),
                                      ZetaPos(i - 1, 0), ZetaPos(i - 1, 1), dLength(i));
                } else {
                    Calculate_dLength(ZetaPos(i - 1, 0), ZetaPos(i - 1, 1), ZetaPos(i, 0), ZetaPos(i, 1),
                                      ZetaPos(i + 1, 0), ZetaPos(i + 1, 1), dLength(i));
                }
            }

            // In order to calculate dLength_Node(i)
            for (int i = 0; i < NumContactNode; i++) {
                if (i == 0) {
                    Calculate_dLength(NodeTransPos(i + 1, 0), NodeTransPos(i + 1, 1), NodeTransPos(i, 0),
                                      NodeTransPos(i, 1), NodeTransPos(i + 1, 0), NodeTransPos(i + 1, 1),
                                      dLength_Node(i));
                } else if (i == NumContactNode - 1) {
                    Calculate_dLength(NodeTransPos(i - 1, 0), NodeTransPos(i - 1, 1), NodeTransPos(i, 0),
                                      NodeTransPos(i, 1), NodeTransPos(i - 1, 0), NodeTransPos(i - 1, 1),
                                      dLength_Node(i));
                } else {
                    Calculate_dLength(NodeTransPos(i - 1, 0), NodeTransPos(i - 1, 1), NodeTransPos(i, 0),
                                      NodeTransPos(i, 1), NodeTransPos(i + 1, 0), NodeTransPos(i + 1, 1),
                                      dLength_Node(i));
                }
            }

            Count = 0;
            for (int i = 0; i < NumElemsX; i++) {
                if (DisFlex(2, i) < ContactLine) {
                    TempVecNode(Count) = ContactFRC(2, i);
                    Count++;
                }
            }

            // Normal Contact Force per node
            TempFn(0) = 0.0;
            for (int i = 0; i < NumContactNode; i++) {
                NodeFn(i) = TempVecNode(int(ReplaceMap(i))) / dLength_Node(i);
                TempFn(i + 1) = NodeFn(i);
            }
            TempFn(NumContactNode + 1) = 0.0;

            TempNodeInfo(0, 0) = RearContactPoint(0);
            TempNodeInfo1(0) = TempNodeInfo(0, 0);
            TempNodeInfo(0, 1) = RearContactPoint(1);
            TempNodeInfo(0, 2) = RearContactPoint(2);
            for (int i = 0; i < NumContactNode; i++) {
                TempNodeInfo(i + 1, 0) = NodeTransPos(i, 0);
                TempNodeInfo1(i + 1) = TempNodeInfo(i + 1, 0);
                TempNodeInfo(i + 1, 1) = NodeTransPos(i, 1);
                TempNodeInfo(i + 1, 2) = NodeTransPos(i, 2);
            }
            TempNodeInfo(NumContactNode + 1, 0) = FrontContactPoint(0);
            TempNodeInfo1(NumContactNode + 1) = TempNodeInfo(NumContactNode + 1, 0);
            TempNodeInfo(NumContactNode + 1, 1) = FrontContactPoint(1);
            TempNodeInfo(NumContactNode + 1, 2) = FrontContactPoint(2);

            //// Interpolation for Normal Contact Force at LuGre point *Size is NumContacNode+2 due to Rear and Front
            for (int i = 0; i < NumLuGreZ; i++) {
                if (RearContactPoint(0) <= ZetaPos(i, 0) && ZetaPos(i, 0) <= FrontContactPoint(0)) {
                    spline_cubic_set(NumContactNode + 2, TempNodeInfo1, TempFn, 2, 0.0, 2, 0.0, ypp);
                    spline_cubic_val(NumContactNode + 2, TempNodeInfo1, TempFn, ypp, ZetaPos(i, 0), ZetaFn(i), Dummy,
                                     Dummy);
                }
            }

            //// Interpolation for Slip Velocity at LuGre point *Size is NumContactNode
            for (int i = 0; i < NumLuGreZ; i++) {
                if (RearContactPoint(0) <= ZetaPos(i, 0) && ZetaPos(i, 0) <= FrontContactPoint(0)) {
                    // Slip in X-direction
                    spline_cubic_set(NumContactNode, NodeTransPosX, NodeTransVelX, 2, 0.0, 2, 0.0, ypp);
                    spline_cubic_val(NumContactNode, NodeTransPosX, NodeTransVelX, ypp, ZetaPos(i, 0), ZetaVr(i, 0),
                                     Dummy, Dummy);
                    // Slip in Y-direction
                    spline_cubic_set(NumContactNode, NodeTransPosX, NodeTransVelY, 2, 0.0, 2, 0.0, ypp);
                    spline_cubic_val(NumContactNode, NodeTransPosX, NodeTransVelY, ypp, ZetaPos(i, 0), ZetaVr(i, 1),
                                     Dummy, Dummy);
                }
            }

            ////////////////////////////////////
            ////g function for combined slip////
            ////////////////////////////////////
            // dZ, derivative of LuGre parameters (1st loop)
            if (RearContactPoint(0) <= ZetaPos(0, 0) && ZetaPos(0, 0) <= FrontContactPoint(0)) {
                G_Function(sqrt(ZetaVr(0, 0) * ZetaVr(0, 0) + ZetaVr(0, 1) * ZetaVr(0, 1)), G_Func);
                if (G_Func < 0.0) {
                    GetLog() << "G_Func is negative!"
                             << "\n";
                    system("pause");
                }
                Gx_Func =
                    G_Func * sqrt((ZetaVr(0, 0) / sqrt(ZetaVr(0, 0) * ZetaVr(0, 0) + ZetaVr(0, 1) * ZetaVr(0, 1))) *
                                  (ZetaVr(0, 0) / sqrt(ZetaVr(0, 0) * ZetaVr(0, 0) + ZetaVr(0, 1) * ZetaVr(0, 1))));
                ParaA = sqrt(RxOmg * RxOmg) / dLength(0);
                ParaB = -(sgm0_x * sqrt(ZetaVr(0, 0) * ZetaVr(0, 0)) / Gx_Func + ParaA);
                ZLuGre(0, 0) = -ZetaVr(0, 0) / ParaB;
                dZLuGre(0, 0) = 0.0;
                // dZLuGre(0, 0) = ZetaVr(0, 0) + ParaB*ZLuGre(0, 0); // For transient LuGre calculation

                Gy_Func =
                    G_Func * sqrt((ZetaVr(0, 1) / sqrt(ZetaVr(0, 0) * ZetaVr(0, 0) + ZetaVr(0, 1) * ZetaVr(0, 1))) *
                                  (ZetaVr(0, 1) / sqrt(ZetaVr(0, 0) * ZetaVr(0, 0) + ZetaVr(0, 1) * ZetaVr(0, 1))));
                ParaA = sqrt(RxOmg * RxOmg) / dLength(0);
                ParaB = -(sgm0_y * sqrt(ZetaVr(0, 1) * ZetaVr(0, 1)) / Gy_Func + ParaA);
                ZLuGre(0, 1) = -ZetaVr(0, 1) / ParaB;
                dZLuGre(0, 1) = 0.0;
                // dZLuGre(0, 1) = ZetaVr(0, 1) + ParaB*ZLuGre(0, 1); // For transient LuGre calculation
            }
            // dZ, derivative of LuGre parameters (2nd loop)
            for (int i = 1; i < NumLuGreZ; i++) {
                if (RearContactPoint(0) <= ZetaPos(i, 0) && ZetaPos(i, 0) <= FrontContactPoint(0)) {
                    G_Function(sqrt(ZetaVr(i, 0) * ZetaVr(i, 0) + ZetaVr(i, 1) * ZetaVr(i, 1)), G_Func);
                    Gx_Func =
                        G_Func * sqrt((ZetaVr(i, 0) / sqrt(ZetaVr(i, 0) * ZetaVr(i, 0) + ZetaVr(i, 1) * ZetaVr(i, 1))) *
                                      (ZetaVr(i, 0) / sqrt(ZetaVr(i, 0) * ZetaVr(i, 0) + ZetaVr(i, 1) * ZetaVr(i, 1))));
                    ParaA = sqrt(RxOmg * RxOmg) / dLength(i);
                    ParaB = -(sgm0_x * sqrt(ZetaVr(i, 0) * ZetaVr(i, 0)) / Gx_Func + ParaA);
                    ZLuGre(i, 0) = -(ZetaVr(i, 0) + ParaA * ZLuGre(i - 1, 0)) / ParaB;
                    dZLuGre(i, 0) = 0.0;
                    // dZLuGre(i, 0) = ZetaVr(i, 0) + ParaA*ZLuGre(i - 1, 0) + ParaB*ZLuGre(i, 0); // For transient
                    // LuGre

                    Gy_Func =
                        G_Func * sqrt((ZetaVr(i, 1) / sqrt(ZetaVr(i, 0) * ZetaVr(i, 0) + ZetaVr(i, 1) * ZetaVr(i, 1))) *
                                      (ZetaVr(i, 1) / sqrt(ZetaVr(i, 0) * ZetaVr(i, 0) + ZetaVr(i, 1) * ZetaVr(i, 1))));
                    ParaA = sqrt(RxOmg * RxOmg) / dLength(i);
                    ParaB = -(sgm0_y * sqrt(ZetaVr(i, 1) * ZetaVr(i, 1)) / Gy_Func + ParaA);
                    ZLuGre(i, 1) = -(ZetaVr(i, 1) + ParaA * ZLuGre(i - 1, 1)) / ParaB;
                    dZLuGre(i, 1) = 0.0;
                    // dZLuGre(i, 1) = ZetaVr(i, 1) + ParaA*ZLuGre(i - 1, 1) + ParaB*ZLuGre(i, 1); // For transient
                    // LuGre
                }
            }

            // LuGre Friction at LuGre points
            for (int i = 0; i < NumLuGreZ; i++) {
                ZetaF(i, 0) = (sgm0_x * ZLuGre(i, 0) + sgm1_x * dZLuGre(i, 0) + sgm2_x * ZetaVr(i, 0)) * ZetaFn(i);
                ZetaFX(i) = ZetaF(i, 0);
                ZetaF(i, 1) = (sgm0_y * ZLuGre(i, 1) + sgm1_y * dZLuGre(i, 1) + sgm2_y * ZetaVr(i, 1)) * ZetaFn(i);
                ZetaFY(i) = ZetaF(i, 1);
                // GetLog() << "ZLuGRe: " << ZLuGre(i,0) << "\n";
            }

            TempR = 0.0;

            // Evaluate LuGre friction force at node points
            for (int i = 0; i < NumContactNode; i++) {
                // Traveling direction
                spline_cubic_set(NumLuGreZ, ZetaPosX, ZetaFX, 2, 0.0, 2, 0.0, ypp);
                spline_cubic_val(NumLuGreZ, ZetaPosX, ZetaFX, ypp, NodeTransPosX(i), TempR, Dummy, Dummy);
                LocalLuGreForce(i, 0) = TempR * dLength_Node(i);

                // Width direction
                spline_cubic_set(NumLuGreZ, ZetaPosX, ZetaFY, 2, 0.0, 2, 0.0, ypp);
                spline_cubic_val(NumLuGreZ, ZetaPosX, ZetaFY, ypp, NodeTransPosX(i), TempR, Dummy, Dummy);
                LocalLuGreForce(i, 1) = TempR * dLength_Node(i);
            }

            for (int i = 0; i < NumContactNode; i++) {
                ContactFRC(0, ContactNodeNum(i)) = -LocalLuGreForce(i, 0);
                ContactFRC(1, ContactNodeNum(i)) = -LocalLuGreForce(i, 1);
            }
        }
    }

    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) {
        ChMatrixDynamic<double> FlexPos(6, (int)loadables.size());  // Matrix of nodal coordinates
        ChMatrixDynamic<double> FlexVel(6, (int)loadables.size());  // Matrix of nodal velocities
        ChMatrixDynamic<double> RigidPos(7, 2);                // Matrix for rigid body positions (includes ground body)
        ChMatrixDynamic<double> RigidVel(7, 2);  // Matrix for rigid body velocities (includes ground body)
        ChMatrixDynamic<int> TempCJN((int)loadables.size(), 1);
        ChMatrixDynamic<double> TempContactFRCE(3, (int)loadables.size());
        ChMatrixDynamic<double> TempSlipVel(6, (int)loadables.size());
        double TempROMG = 0.0;

        NetContactForce.x = 0.0;
        NetContactForce.y = 0.0;
        NetContactForce.z = 0.0;

        // Set position and velocity for Ground and Rim
        RigidPos(0, 0) = 0.0;
        RigidPos(1, 0) = 0.0;
        RigidPos(2, 0) = 0.0;
        RigidPos(3, 0) = 1.0;
        RigidPos(4, 0) = 0.0;
        RigidPos(5, 0) = 0.0;
        RigidPos(6, 0) = 0.0;
        RigidPos(0, 1) = mRim->GetPos().x;
        RigidPos(1, 1) = mRim->GetPos().y;
        RigidPos(2, 1) = mRim->GetPos().z;
        RigidPos(3, 1) = mRim->GetRot().e0;
        RigidPos(4, 1) = mRim->GetRot().e1;
        RigidPos(5, 1) = mRim->GetRot().e2;
        RigidPos(6, 1) = mRim->GetRot().e3;

        RigidVel(0, 0) = 0.0;
        RigidVel(1, 0) = 0.0;
        RigidVel(2, 0) = 0.0;
        RigidVel(3, 0) = 0.0;
        RigidVel(4, 0) = 0.0;
        RigidVel(5, 0) = 0.0;
        RigidVel(6, 0) = 0.0;
        RigidVel(0, 1) = mRim->GetPos_dt().x;
        RigidVel(1, 1) = mRim->GetPos_dt().y;
        RigidVel(2, 1) = mRim->GetPos_dt().z;
        RigidVel(3, 1) = mRim->GetRot_dt().e0;
        RigidVel(4, 1) = mRim->GetRot_dt().e1;
        RigidVel(5, 1) = mRim->GetRot_dt().e2;
        RigidVel(6, 1) = mRim->GetRot_dt().e3;

        if (state_x && state_w) {
            for (int ie = 0; ie < loadables.size(); ie++)  // Loop over the nodes in the circumferential direction
            {
                ChVector<> P1 = state_x->ClipVector(6 * ie, 0);
                ChVector<> P1d = state_x->ClipVector(6 * ie + 3, 0);
                ChVector<> V1 = state_w->ClipVector(6 * ie, 0);
                ChVector<> V1d = state_w->ClipVector(6 * ie + 3, 0);

                // Set the position and velocty for the node in the entire loop
                FlexPos(0, ie) = P1.x;
                FlexPos(1, ie) = P1.y;
                FlexPos(2, ie) = P1.z;
                FlexPos(3, ie) = P1d.x;
                FlexPos(4, ie) = P1d.y;
                FlexPos(5, ie) = P1d.z;

                FlexVel(0, ie) = V1.x;
                FlexVel(1, ie) = V1.y;
                FlexVel(2, ie) = V1.z;
                FlexVel(3, ie) = V1d.x;
                FlexVel(4, ie) = V1d.y;
                FlexVel(5, ie) = V1d.z;
            }

            // Function for LuGre force calculation
            LuGre_SteadyState((int)loadables.size(), 24, RigidPos, RigidVel, FlexPos, FlexVel, TempContactFRCE, TempSlipVel,
                              TempROMG);

            for (int ie = 0; ie < loadables.size(); ie++)  // Loop over all nodes in the circumferential direction
            {
                // Load the contact force into the Q vector
                this->load_Q(6 * ie + 0) = TempContactFRCE(0, ie);
                this->load_Q(6 * ie + 1) = TempContactFRCE(1, ie);
                this->load_Q(6 * ie + 2) = TempContactFRCE(2, ie);

                // Load the contact force to be written to a file
                NodeContactForce(0, ie) = TempContactFRCE(0, ie);
                NodeContactForce(1, ie) = TempContactFRCE(1, ie); 
                NodeContactForce(2, ie) = TempContactFRCE(2, ie);
                NodeContactForce(3, ie) = 0.0;
                NodeContactForce(4, ie) = 0.0;
                NodeContactForce(5, ie) = 0.0;

                // In order to monitor the net contact force
                if (TempContactFRCE(2, ie) != 0.0) {
                    NetContactForce.x += TempContactFRCE(0, ie);
                    NetContactForce.y += TempContactFRCE(1, ie);
                    NetContactForce.z += TempContactFRCE(2, ie);
                }
            }
        }  // end of if(state) loop

    }  // end of Compute_Q

    // Activate jacobian calculation
    virtual bool IsStiff() { return true; }

};  // end of ChLoaderLuGre

// Reads the input file for creating the HMMWV tire.
// Determines initial configuration and the properties
// of the elements, layers, and materials.
void ReadInputFile(ChMatrixNM<double, 3000, 6>& COORDFlex,
                   ChMatrixNM<double, 3000, 6>& VELCYFlex,
                   ChMatrixNM<int, 2880, 4>& NodesPerElement,
                   int& TotalNumElements,
                   int& NumElements_x,
                   int& NumElements_y,
                   int& TotalNumNodes,
                   ChMatrixNM<int, 2880, 1>& SectionID,
                   ChMatrixNM<double, 15, 2>& LayerPROP,
                   ChMatrixNM<int, 15, 7>& MatID,
                   ChMatrixNM<double, 7, 12>& MPROP,
                   ChMatrixNM<double, 2880, 2>& ElementLength,
                   ChMatrixNM<int, 3, 1>& NumLayPerSect) {
    FILE* inputfile;
    char str1[100];
    int numFlexBody = 0;
    int dummy;
    int count;

    int NDR[4000][6];
    int NumLayer[10];
    int MaxSectionNumber = 0;
    int MaxMatID = 0;
    int MTYPE = 0;

    int MAXCOUNT = 100;

    inputfile = fopen(GetChronoDataFile("fea/ANCFtire/HMMWVBiLinearShell_Tire.INP").c_str(), "r");
    printf("Open HMMWVBiLinearShell_Tire.INP\n");
    if (inputfile == NULL) {
        printf("Input data file not found!!\n");
        system("pause");
        exit(1);
    }

    TotalNumElements = 0;
    NumElements_x = 0;
    NumElements_y = 0;
    TotalNumNodes = 0;

    //!--------------------------------------!
    //!-- Elememt data            -----------!
    //!--------------------------------------!

    fgets(str1, MAXCOUNT, inputfile);
    printf("%s\n", str1);
    fscanf(inputfile, "%d\n", &numFlexBody);

    fgets(str1, MAXCOUNT, inputfile);
    printf("%s\n", str1);
    fscanf(inputfile, "%d %d %d %d\n", &TotalNumElements, &NumElements_x, &NumElements_y, &TotalNumNodes);
    fgets(str1, MAXCOUNT, inputfile);

    printf("%s\n", str1);
    for (int i = 0; i < TotalNumElements; i++) {
        fscanf(inputfile, "%d %d %d %d %d %d %d\n", &count, &dummy, &SectionID(i, 0), &NodesPerElement(i, 0),
               &NodesPerElement(i, 1), &NodesPerElement(i, 2), &NodesPerElement(i, 3));
        printf("SectionID[i] %d\n  ", SectionID(i, 0));

        fscanf(inputfile, " %lf %lf\n", &ElementLength(i, 0), &ElementLength(i, 1));
        if (MaxSectionNumber < SectionID(i, 0)) {
            MaxSectionNumber = SectionID(i, 0);
        }
    }

    //!--------------------------------------!
    //!-- NDR,COORDFlex,VELCYFlex -----------!
    //!--------------------------------------!
    fgets(str1, MAXCOUNT, inputfile);
    printf("%s\n", str1);
    for (int i = 0; i < TotalNumNodes; i++) {
        fscanf(inputfile, "%d %d %d %d %d %d %d\n", &count, &NDR[i][0], &NDR[i][1], &NDR[i][2], &NDR[i][3], &NDR[i][4],
               &NDR[i][5]);
        fscanf(inputfile, "%lf %lf %lf %lf %lf %lf\n", &COORDFlex(i, 0), &COORDFlex(i, 1), &COORDFlex(i, 2),
               &COORDFlex(i, 3), &COORDFlex(i, 4), &COORDFlex(i, 5));
        fscanf(inputfile, "%lf %lf %lf %lf %lf %lf\n", &VELCYFlex(i, 0), &VELCYFlex(i, 1), &VELCYFlex(i, 2),
               &VELCYFlex(i, 3), &VELCYFlex(i, 4), &VELCYFlex(i, 5));
    }

    //!--------------------------------------!
    //!--- Read Layer Data ------------------!
    //!--------------------------------------!
    // fscanf(inputfile,"%s\n",str1);
    fgets(str1, MAXCOUNT, inputfile);
    printf("%s\n", str1);
    int counted = 0;
    for (int i = 0; i < MaxSectionNumber; i++) {
        fscanf(inputfile, "%d %d\n", &count, &NumLayer[i]);
        for (int j = 0; j < NumLayer[i]; j++) {
            fscanf(inputfile, "%lf %lf %d\n", &LayerPROP(counted + j, 0), &LayerPROP(counted + j, 1), &MatID(i, j));
            if (MaxMatID < MatID(i, j)) {
                MaxMatID = MatID(i, j);
            }
            NumLayPerSect(i) = NumLayer[i];
        }
        counted += NumLayPerSect(i);
    }

    //!--------------------------------------!
    //!--- Read Material Data ---------------!
    //!--------------------------------------!
    // fscanf(inputfile,"%s\n",str1);
    fgets(str1, MAXCOUNT, inputfile);
    printf("%s\n", str1);
    for (int i = 0; i < MaxMatID; i++) {
        fscanf(inputfile, "%d %d\n", &count, &MTYPE);
        if (MTYPE == 1) {
            fscanf(inputfile, "%lf %lf %lf %lf\n", &MPROP(i, 0), &MPROP(i, 1), &MPROP(i, 2), &MPROP(i, 3));
        }
        if (MTYPE == 2) {
            fscanf(inputfile, "%lf %lf %lf %lf\n", &MPROP(i, 0), &MPROP(i, 1), &MPROP(i, 2), &MPROP(i, 3));
            fscanf(inputfile, "%lf %lf %lf %lf %lf %lf\n", &MPROP(i, 4), &MPROP(i, 5), &MPROP(i, 6), &MPROP(i, 7),
                   &MPROP(i, 8), &MPROP(i, 9));
        }
    }
};

// Reads an input file to start the HMMWV tire in an
// initial state. Returns state information for the
// nodes and the rigid bodies.
void ReadRestartInput(ChMatrixNM<double, 2, 7>& COORDRigid,
                      ChMatrixNM<double, 2, 7>& VELCYRigid,
                      ChMatrixNM<double, 2, 7>& ACCELRigid,
                      ChMatrixNM<double, 3000, 6>& COORDFlex,
                      ChMatrixNM<double, 3000, 6>& VELCYFlex,
                      ChMatrixNM<double, 3000, 6>& ACCELFlex,
                      int TotalNumNodes) {
    FILE* inputfile1;
    char str1[100];
    int MAXCOUNT = 100;
    double LuGreZStart[25][40];
    double LuGreZStart_dt[25][40];
    double LuGreZStart_dtdt[25][40];

    inputfile1 = fopen(GetChronoDataFile("fea/ANCFtire/QSOL0_All0.txt").c_str(), "r");
    printf("Open QSOL0_All0.txt \n");

    if (inputfile1 == NULL) {
        printf("Restart Information not found!\n");
        system("pause");
        exit(1);
    }

    fgets(str1, MAXCOUNT, inputfile1);
    printf("%s\n", str1);
    fscanf(inputfile1, "%lf %lf %lf %lf %lf %lf %lf", &COORDRigid(0, 0), &COORDRigid(0, 1), &COORDRigid(0, 2),
           &COORDRigid(0, 3), &COORDRigid(0, 4), &COORDRigid(0, 5), &COORDRigid(0, 6));
    fscanf(inputfile1, "%lf %lf %lf %lf %lf %lf %lf", &COORDRigid(1, 0), &COORDRigid(1, 1), &COORDRigid(1, 2),
           &COORDRigid(1, 3), &COORDRigid(1, 4), &COORDRigid(1, 5), &COORDRigid(1, 6));
    for (int i = 0; i < TotalNumNodes; i++) {
        fscanf(inputfile1, "%lf %lf %lf %lf %lf %lf", &COORDFlex(i, 0), &COORDFlex(i, 1), &COORDFlex(i, 2),
               &COORDFlex(i, 3), &COORDFlex(i, 4), &COORDFlex(i, 5));
    }
    for (int i = 0; i < 25; i++) {
        for (int j = 0; j < 40; j++) {
            fscanf(inputfile1, "%lf ", &LuGreZStart[i][j]);
        }
    }
    fscanf(inputfile1, "\n");
    fgets(str1, MAXCOUNT, inputfile1);
    printf("%s\n", str1);
    fscanf(inputfile1, "%lf %lf %lf %lf %lf %lf %lf", &VELCYRigid(0, 0), &VELCYRigid(0, 1), &VELCYRigid(0, 2),
           &VELCYRigid(0, 3), &VELCYRigid(0, 4), &VELCYRigid(0, 5), &VELCYRigid(0, 6));
    fscanf(inputfile1, "%lf %lf %lf %lf %lf %lf %lf", &VELCYRigid(1, 0), &VELCYRigid(1, 1), &VELCYRigid(1, 2),
           &VELCYRigid(1, 3), &VELCYRigid(1, 4), &VELCYRigid(1, 5), &VELCYRigid(1, 6));
    for (int i = 0; i < TotalNumNodes; i++) {
        fscanf(inputfile1, "%lf %lf %lf %lf %lf %lf", &VELCYFlex(i, 0), &VELCYFlex(i, 1), &VELCYFlex(i, 2),
               &VELCYFlex(i, 3), &VELCYFlex(i, 4), &VELCYFlex(i, 5));
    }
    for (int i = 0; i < 25; i++) {
        for (int j = 0; j < 40; j++) {
            fscanf(inputfile1, "%lf ", &LuGreZStart_dt[i][j]);
        }
    }
    fscanf(inputfile1, "\n");
    fgets(str1, MAXCOUNT, inputfile1);
    printf("%s\n", str1);
    fscanf(inputfile1, "%lf %lf %lf %lf %lf %lf %lf", &ACCELRigid(0, 0), &ACCELRigid(0, 1), &ACCELRigid(0, 2),
           &ACCELRigid(0, 3), &ACCELRigid(0, 4), &ACCELRigid(0, 5), &ACCELRigid(0, 6));
    fscanf(inputfile1, "%lf %lf %lf %lf %lf %lf %lf", &ACCELRigid(1, 0), &ACCELRigid(1, 1), &ACCELRigid(1, 2),
           &ACCELRigid(1, 3), &ACCELRigid(1, 4), &ACCELRigid(1, 5), &ACCELRigid(1, 6));
    for (int i = 0; i < TotalNumNodes; i++) {
        fscanf(inputfile1, "%lf %lf %lf %lf %lf %lf", &ACCELFlex(i, 0), &ACCELFlex(i, 1), &ACCELFlex(i, 2),
               &ACCELFlex(i, 3), &ACCELFlex(i, 4), &ACCELFlex(i, 5));
    }
    for (int i = 0; i < 25; i++) {
        for (int j = 0; j < 40; j++) {
            fscanf(inputfile1, "%lf ", &LuGreZStart_dtdt[i][j]);
        }
    }
    GetLog() << "Restart Complete!\n\n";
};

int main(int argc, char* argv[]) {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: ANCF Tire (Fixed),  implicit integration \n\n";

    FILE* outputfile;   // Time history of nodal coordinates
    FILE* outputfile1;  // Time history of rigid bodies
    FILE* outputfile2;  // Ground line and normal contact force
    FILE* outputfile3;  // Number of iterations and CPU time
    FILE* outputfile4;  // Time history of contact forces per nodal coordinate

    // Boolean variables to determine which output files are written
    bool output = true;
    bool output1 = true;
    bool output2 = true;
    bool output3 = true;
    bool output4 = true;

    // The physical system: it contains all physical objects.
    ChSystem my_system;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();
    // Visualization:
    ChIrrApp application(&my_system, L"ANCF Rolling Tire", core::dimension2d<u32>(1080, 800), false);
    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0.5f, 0.5f, 1.15f),    // camera location
                                 core::vector3df(-1.15f, 0.0f, 0.0f));  // "look at" location
    application.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 160,
                                 70);
    utils::CSV_writer out("\t");
    out.stream().setf(std::ios::scientific | std::ios::showpos);
    out.stream().precision(7);

    int TotalNumNodes;
    // Matricies to hold the state informatino for the nodes and rigid bodies
    ChMatrixNM<double, 3000, 6> COORDFlex;
    ChMatrixNM<double, 3000, 6> VELCYFlex;
    ChMatrixNM<double, 3000, 6> ACCELFlex;
    ChMatrixNM<double, 2, 7> COORDRigid;
    ChMatrixNM<double, 2, 7> VELCYRigid;
    ChMatrixNM<double, 2, 7> ACCELRigid;

    ChMatrixNM<int, 2880, 4> NodesPerElement;  // Defines the connectivity between the elements and nodes
    ChMatrixNM<double, 2880, 2> ElemLength;    // X and Y dimensions of the shell elements
    int TotalNumElements;
    int NumElements_x;
    int NumElements_y;
    ChMatrixNM<int, 2880, 1> SectionID;  // Catagorizes which tire section the elements are a part of
    ChMatrixNM<double, 15, 2> LayPROP;   // Thickness and ply angles of the layered elements
    ChMatrixNM<int, 15, 7> MatID;        // Catagorizes the material of each layer
    ChMatrixNM<double, 7, 12> MPROP;     // Material properties
    ChMatrixNM<int, 3, 1> NumLayPerSection;
    int NumCont = 0;        // Number of nodes in contact with the ground
    double ContactZ = 0.0;  // Vertical location of the flat ground
    ChVector<> NetContact;  // Net contact forces

    // Option to use visualization
    bool UseVisualization = true;

    // Take initial conf. from input file for "steady" LuGre
    bool Restart = true;

    // First input file: Initial (reference) configuration
    ReadInputFile(COORDFlex, VELCYFlex, NodesPerElement, TotalNumElements, NumElements_x, NumElements_y, TotalNumNodes,
                  SectionID, LayPROP, MatID, MPROP, ElemLength, NumLayPerSection);

    if (Restart) {
        // Second input: Read dynamic state (velocities, accelerations)
        ReadRestartInput(COORDRigid, VELCYRigid, ACCELRigid, COORDFlex, VELCYFlex, ACCELFlex, TotalNumNodes);
    }

    // Material List (for HMMWV)
    // i=0: Carcass
    // i=1: Steel belt in rubber matrix
    // i=2: Rubber

    std::vector<std::shared_ptr<ChMaterialShellANCF>> MaterialList(MPROP.GetRows());
    for (int i = 0; i < MPROP.GetRows(); i++) {
        double rho = MPROP(i, 0);
        ChVector<double> E(MPROP(i, 1), MPROP(i, 2), MPROP(i, 3));
        ChVector<double> nu(MPROP(i, 4), MPROP(i, 5), MPROP(i, 6));
        ChVector<double> G(MPROP(i, 7), MPROP(i, 8), MPROP(i, 9));
        MaterialList[i] = std::make_shared<ChMaterialShellANCF>(rho, E, nu, G);
    }

    // Create a set of nodes for the tire based on the input data
    for (int i = 0; i < TotalNumNodes; i++) {
        auto node = std::make_shared<ChNodeFEAxyzD>(ChVector<>(COORDFlex(i, 0), COORDFlex(i, 1), COORDFlex(i, 2)),
                              ChVector<>(COORDFlex(i, 3), COORDFlex(i, 4), COORDFlex(i, 5)));
        node->SetPos_dt(ChVector<>(VELCYFlex(i, 0), VELCYFlex(i, 1), VELCYFlex(i, 2)));
        node->SetD_dt(ChVector<>(VELCYFlex(i, 3), VELCYFlex(i, 4), VELCYFlex(i, 5)));
        node->SetPos_dtdt(ChVector<>(ACCELFlex(i, 0), ACCELFlex(i, 1), ACCELFlex(i, 2)));
        node->SetD_dtdt(ChVector<>(ACCELFlex(i, 3), ACCELFlex(i, 4), ACCELFlex(i, 5)));
        node->SetMass(0.0);
        // Determine initial contact
        if (COORDFlex(i, 2) < ContactZ) {
            NumCont++;
        }
        my_mesh->AddNode(node);  // Add nodes to the system
    }
    // Check position of the bottom node
    GetLog() << "TotalNumNodes: " << TotalNumNodes << "\n\n";
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode((TotalNumElements / 2)));
    GetLog() << "X : " << nodetip->GetPos().x << " Y : " << nodetip->GetPos().y << " Z : " << nodetip->GetPos().z
             << "\n\n";
    GetLog() << "dX : " << nodetip->GetD().x << " dY : " << nodetip->GetD().y << " dZ : " << nodetip->GetD().z
             << "\n\n";

    double timestep = 0.0001;  // Initial time step
    int LayerHist = 0;         // Number of layers in the previous tire sections

    // Create all elements of the tire
    for (int i = 0; i < TotalNumElements; i++) {
        auto element = std::make_shared<ChElementShellANCF>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(NodesPerElement(i, 0) - 1)),
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(NodesPerElement(i, 1) - 1)),
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(NodesPerElement(i, 2) - 1)),
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(NodesPerElement(i, 3) - 1)));
         
        element->SetDimensions(ElemLength(i, 0), ElemLength(i, 1));

        // Determine the section in which the current element resides
        switch (SectionID(i)) {
            // Bead section
            case 1:
                LayerHist = 0;
                break;

            // Sidewall section
            case 2:
                LayerHist = NumLayPerSection(0);
                break;

            // Tread section
            case 3:
                LayerHist = NumLayPerSection(0) + NumLayPerSection(1);
                break;
        }  // End of switch

        // Give material properties to elements as a construction of layers
        for (int j = 0; j < NumLayPerSection(SectionID(i) - 1); j++) {
            element->AddLayer(LayPROP(LayerHist + j, 0), LayPROP(LayerHist + j, 1) * CH_C_DEG_TO_RAD,
                              MaterialList[MatID(SectionID(i) - 1, j) - 1]);
        }

        element->SetAlphaDamp(0.0005);
        element->SetGravityOn(true);
        my_mesh->AddElement(element);
    }

    // Create rim body
    auto Rim = std::make_shared<ChBody>();
    my_system.Add(Rim);
    Rim->SetBodyFixed(false);
    Rim->SetPos(ChVector<>(0.0, 0.0, 0.4673));
    Rim->SetRot(ChQuaternion<>(1.0, 0.0, 0.0, 0.0));
    Rim->SetPos_dt(ChVector<>(0.0, 0.0, 0.0));
    Rim->SetRot_dt(ChQuaternion<>(0.0, 0.0, 0.0, 0.0));
    Rim->SetMass(509.68);  // 509.68
    Rim->SetInertiaXX(ChVector<>(0.1457270, 0.2359220, 0.1457270));
    // Give initial state to the rim (from input file)
    if (Restart) {
        Rim->SetPos(ChVector<>(COORDRigid(1, 0), COORDRigid(1, 1), COORDRigid(1, 2)));
        Rim->SetRot(ChQuaternion<>(COORDRigid(1, 3), COORDRigid(1, 4), COORDRigid(1, 5), COORDRigid(1, 6)));
        Rim->SetPos_dt(ChVector<>(VELCYRigid(1, 0), VELCYRigid(1, 1), VELCYRigid(1, 2)));
        Rim->SetRot_dt(ChQuaternion<>(VELCYRigid(1, 3), VELCYRigid(1, 4), VELCYRigid(1, 5), VELCYRigid(1, 6)));
        Rim->SetPos_dtdt(ChVector<>(ACCELRigid(1, 0), ACCELRigid(1, 1), ACCELRigid(1, 2)));
        Rim->SetRot_dtdt(ChQuaternion<>(ACCELRigid(1, 3), ACCELRigid(1, 4), ACCELRigid(1, 5), ACCELRigid(1, 6)));
    }

    // Create ground body
    auto Ground = std::make_shared<ChBody>();
    Ground->SetBodyFixed(true);
    Ground->SetPos(ChVector<>(0.0, 0.0, -0.02));
    Ground->SetRot(ChQuaternion<>(1.0, 0.0, 0.0, 0.0));
    my_system.Add(Ground);

    // Apply gravitational acceleration
    my_system.Set_G_acc(ChVector<>(0.0, 0.0, 0.0));
    my_mesh->SetAutomaticGravity(false);

    // First: loads must be added to "load containers",
    // and load containers must be added to your ChSystem
    auto Mloadcontainer = std::make_shared<ChLoadContainer>();

    ////// LuGre Load Class initialization
    std::vector<std::shared_ptr<ChLoaderLuGre>> LoadList(NumElements_y + 1);
    for (int i = 0; i < NumElements_y + 1; i++) {
        std::vector<std::shared_ptr<ChLoadable>> mnodelist;
        for (int inode = 0; inode < (TotalNumNodes / (NumElements_y + 1)); inode++) {
            auto FrictionNode = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode((i * TotalNumNodes / (NumElements_y + 1)) + inode));
            mnodelist.push_back(FrictionNode);
        }

        LoadList[i] = std::make_shared<ChLoaderLuGre>(mnodelist);
        LoadList[i]->ContactLine = ContactZ;
        LoadList[i]->NumContact = NumCont;
        LoadList[i]->mRim = Rim;
        Mloadcontainer->Add(LoadList[i]);
    }

    class MyPressureLoad : public ChLoaderUVdistributed {
      private:
        std::shared_ptr<ChElementShellANCF> m_element;

      public:
        // Useful: a constructor that also sets ChLoadable
        MyPressureLoad(std::shared_ptr<ChLoadableUV> element) : ChLoaderUVdistributed(element) {
            m_element = std::static_pointer_cast<ChElementShellANCF>(element);
        };
        virtual bool IsStiff() override { return true; }
        virtual int GetIntegrationPointsU() { return 2; }
        virtual int GetIntegrationPointsV() { return 2; }
        // Compute F=F(u)
        // This is the function that you have to implement. It should return the
        // load at U. For Eulero beams, loads are expected as 6-rows vectors, containing
        // a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ.
        ChVector<> FPressure;
        virtual void ComputeF(
            const double U,
            const double V,              ///< parametric coordinate in line
            ChVectorDynamic<>& F,        ///< Result F vector here, size must be = n.field coords.of loadable
            ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
            ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
            ) {
            ChVector<> Position1;
            ChVector<> Gradient1;
            ChVector<> Position2;
            ChVector<> Gradient2;
            ChVector<> Position3;
            ChVector<> Gradient3;
            ChVector<> Position4;
            ChVector<> Gradient4;
            double PressureVal = 220e3;  // Pressure

            if (state_x && state_w) {
                Position1 = state_x->ClipVector(0, 0);
                Gradient1 = state_x->ClipVector(3, 0);
                Position2 = state_x->ClipVector(6, 0);
                Gradient2 = state_x->ClipVector(9, 0);
                Position3 = state_x->ClipVector(12, 0);
                Gradient3 = state_x->ClipVector(15, 0);
                Position4 = state_x->ClipVector(18, 0);
                Gradient4 = state_x->ClipVector(21, 0);

                ChMatrixNM<double, 1, 8> Nx;
                ChMatrixNM<double, 1, 8> N;
                ChMatrixNM<double, 1, 8> Ny;
                ChMatrixNM<double, 1, 8> Nz;
                ChMatrixNM<double, 3, 8> d;
                (d).PasteVector(Position1, 0, 0);
                (d).PasteVector(Gradient1, 0, 1);
                (d).PasteVector(Position2, 0, 2);
                (d).PasteVector(Gradient2, 0, 3);
                (d).PasteVector(Position3, 0, 4);
                (d).PasteVector(Gradient3, 0, 5);
                (d).PasteVector(Position4, 0, 6);
                (d).PasteVector(Gradient4, 0, 7);
                m_element->ShapeFunctions(N, U, V, 0);
                m_element->ShapeFunctionsDerivativeX(Nx, U, V, 0);
                m_element->ShapeFunctionsDerivativeY(Ny, U, V, 0);
                m_element->ShapeFunctionsDerivativeZ(Nz, U, V, 0);

                ChMatrixNM<double, 1, 3> Nx_d;
                Nx_d.MatrMultiplyT(Nx, d);

                ChMatrixNM<double, 1, 3> Ny_d;
                Ny_d.MatrMultiplyT(Ny, d);

                ChMatrixNM<double, 1, 3> Nz_d;
                Nz_d.MatrMultiplyT(Nz, d);

                ChMatrixNM<double, 3, 3> rd;
                rd(0, 0) = Nx_d(0, 0);
                rd(1, 0) = Nx_d(0, 1);
                rd(2, 0) = Nx_d(0, 2);
                rd(0, 1) = Ny_d(0, 0);
                rd(1, 1) = Ny_d(0, 1);
                rd(2, 1) = Ny_d(0, 2);
                rd(0, 2) = Nz_d(0, 0);
                rd(1, 2) = Nz_d(0, 1);
                rd(2, 2) = Nz_d(0, 2);

                ChVector<> G1xG2;
                G1xG2(0) = rd(1, 0) * rd(2, 1) - rd(2, 0) * rd(1, 1);
                G1xG2(1) = rd(2, 0) * rd(0, 1) - rd(0, 0) * rd(2, 1);
                G1xG2(2) = rd(0, 0) * rd(1, 1) - rd(1, 0) * rd(0, 1);
                G1xG2.Normalize();
                FPressure = -G1xG2 * PressureVal;
            }
            F.PasteVector(FPressure, 0, 0);
        }
    };

    // Create the load (and handle it with a shared pointer).
    // The ChLoad is a 'container' for your ChLoader.
    // It is created using templates, that is instancing a ChLoad<a_loader_class>()
    // initiate for loop for all the elements
    for (int NoElmPre = 0; NoElmPre < TotalNumElements; NoElmPre++) {
        auto PressureElement = std::make_shared<ChLoad<MyPressureLoad > >(std::static_pointer_cast<ChElementShellANCF>(my_mesh->GetElement(NoElmPre)));
        Mloadcontainer->Add(PressureElement);  // do not forget to add the load to the load container.
    }

    my_system.Add(Mloadcontainer);
    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Create constraints for the tire and rim
    std::shared_ptr<ChLinkPointFrame> constraint;
    std::shared_ptr<ChLinkDirFrame> constraintD;
    std::shared_ptr<ChNodeFEAxyzD> ConstrainedNode;
    std::shared_ptr<ChLinkLockPlanePlane> constraintRim;
    std::shared_ptr<ChLinkLockPointPlane> constraintLateral;

    // Constrain the flexible tire to the rigid rim body.
    for (int i = 0; i < TotalNumNodes; i++) {
        if (i < NumElements_x ||
            i >= TotalNumNodes - NumElements_x) {  // Only constrain the nodes at the ends of the bead section

            ConstrainedNode = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(i));

            // Add position constraints
            constraint = std::make_shared<ChLinkPointFrame>(); 
            constraint->Initialize(ConstrainedNode, Rim);
            my_system.Add(constraint);

            // Add rotation constraints
            constraintD = std::make_shared<ChLinkDirFrame>();
            constraintD->Initialize(ConstrainedNode, Rim);
            constraintD->SetDirectionInAbsoluteCoords(ConstrainedNode->GetD());
            my_system.Add(constraintD);
        }
    }

    // Constrain only the lateral displacement of the Rim
    constraintLateral = std::make_shared<ChLinkLockPointPlane>(); 
    my_system.AddLink(constraintLateral);
    constraintLateral->Initialize(Rim, Ground, ChCoordsys<>(Rim->GetPos(), Q_from_AngX(CH_C_PI_2)));

    // This is mandatory
    my_system.SetupInitial();

    // Use the MKL Solver
    ChLcpMklSolver* mkl_solver_stab = new ChLcpMklSolver;
    ChLcpMklSolver* mkl_solver_speed = new ChLcpMklSolver;
    my_system.ChangeLcpSolverStab(mkl_solver_stab);
    my_system.ChangeLcpSolverSpeed(mkl_solver_speed);
    mkl_solver_stab->SetSparsityPatternLock(true);
    mkl_solver_speed->SetSparsityPatternLock(true);
    my_system.Update();

    // Set the time integrator parameters
    my_system.SetIntegrationType(ChSystem::INT_HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(20);
    mystepper->SetAbsTolerances(4e-4, 1e-1);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetVerbose(true);
    mystepper->SetScaling(true);

    // Visualization
    // auto mobjmesh = std::make_shared<ChObjShapeFile>();
    // mobjmesh->SetFilename(GetChronoDataFile("fea/tractor_wheel_rim.obj"));
    // Rim->AddAsset(mobjmesh);

    double start = std::clock();
    auto mvisualizemeshC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshC->SetSymbolsThickness(0.003);
    my_mesh->AddAsset(mvisualizemeshC);

    auto mvisualizemeshwire = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshwire->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshwire->SetWireframe(true);
    my_mesh->AddAsset(mvisualizemeshwire);

    auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 30);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh);


    // Create the a plane using body of 'box' type:
    auto mrigidBody = std::make_shared<ChBodyEasyBox>(10, 10, 0.000001, 1000, false, true);
    my_system.Add(mrigidBody);
    mrigidBody->SetPos(ChVector<>(0, 0, ContactZ));
    mrigidBody->SetBodyFixed(true);
    mrigidBody->GetMaterialSurface()->SetFriction(0.5);


    application.AssetBindAll();
    application.AssetUpdateAll();

    ///////

    my_system.Setup();
    my_system.Update();

    // Create output files
    outputfile = fopen("OutPutBody1.txt", "w");          // Time history of nodal coordinates
    outputfile1 = fopen("AllPositions.txt", "w");        // Time history of rigid bodies
    outputfile2 = fopen("ContactInformation.txt", "w");  // Ground line and normal contact force
    outputfile3 = fopen("RES.txt", "w");                 // Number of iterations and CPU time
    outputfile4 = fopen("StockContactForce.txt", "w");   // Time history of contact forces per nodal coordinate

    // Monitor values in output window
    GetLog() << "Contact Line: " << ContactZ << "\n";
    GetLog() << "Contact ForceX: " << NetContact.x << "\n";
    GetLog() << "Contact ForceY: " << NetContact.y << "\n";
    GetLog() << "Contact ForceZ: " << NetContact.z << "\n";
    GetLog() << "Number of Contact Points: " << NumCont << "\n";
    GetLog() << " t=  " << my_system.GetChTime() << "\n\n";

    // Output time history of nodal coordinates
    if (output) {
        fprintf(outputfile, "%15.7e  ", my_system.GetChTime());  // Current time

        // Nodal coordinates for each node
        for (int ii = 0; ii < TotalNumNodes; ii++) {
            auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(ii));
            fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x);
            fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y);
            fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z);
            fprintf(outputfile, "%15.7e  ", nodetip->GetD().x);
            fprintf(outputfile, "%15.7e  ", nodetip->GetD().y);
            fprintf(outputfile, "%15.7e  ", nodetip->GetD().z);
        }

        // Nodal velocities for each node
        for (int ii = 0; ii < TotalNumNodes; ii++) {
            auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode((ii)));
            fprintf(outputfile, "%15.7e  ", nodetip->GetPos_dt().x);
            fprintf(outputfile, "%15.7e  ", nodetip->GetPos_dt().y);
            fprintf(outputfile, "%15.7e  ", nodetip->GetPos_dt().z);
            fprintf(outputfile, "%15.7e  ", nodetip->GetD_dt().x);
            fprintf(outputfile, "%15.7e  ", nodetip->GetD_dt().y);
            fprintf(outputfile, "%15.7e  ", nodetip->GetD_dt().z);
        }
        fprintf(outputfile, "\n  ");
    }

    // Output time history of rigid body coordinates
    if (output1) {
        fprintf(outputfile1, "%15.7e  ", my_system.GetChTime());
        fprintf(outputfile1, "%15.7e  ", 0.0);
        fprintf(outputfile1, "%15.7e  ", 0.0);
        fprintf(outputfile1, "%15.7e  ", 0.0);
        fprintf(outputfile1, "%15.7e  ", 1.0);
        fprintf(outputfile1, "%15.7e  ", 0.0);
        fprintf(outputfile1, "%15.7e  ", 0.0);
        fprintf(outputfile1, "%15.7e  ", 0.0);
        fprintf(outputfile1, "%15.7e  ", Rim->GetPos().x);
        fprintf(outputfile1, "%15.7e  ", Rim->GetPos().y);
        fprintf(outputfile1, "%15.7e  ", Rim->GetPos().z);
        fprintf(outputfile1, "%15.7e  ", Rim->GetRot().e0);
        fprintf(outputfile1, "%15.7e  ", Rim->GetRot().e1);
        fprintf(outputfile1, "%15.7e  ", Rim->GetRot().e2);
        fprintf(outputfile1, "%15.7e  ", Rim->GetRot().e3);
        fprintf(outputfile1, "\n  ");
    }

    // Output time history contact forces per nodal coordinate
    if (output4) {
        fprintf(outputfile4, "%15.7e  ", my_system.GetChTime());
        for (int i = 0; i < NumElements_y + 1; i++) {
            for (int j = 0; j < NumElements_x; j++) {
                fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(0, j));
                fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(1, j));
                fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(2, j));
                fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(3, j));
                fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(4, j));
                fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(5, j));
            }
        }
        fprintf(outputfile4, "\n   ");
    }

    if (UseVisualization) {
        // Visualization
        GetLog() << "\n\nREADME\n\n"
                 << " - Press SPACE to start dynamic simulation \n - Press F10 for nonlinear statics - Press F11 for "
                    "linear statics. \n";

        // at beginning, no analysis is running..
        application.SetPaused(true);
        int AccuNoIterations = 0;
        application.SetStepManage(true);
        application.SetTimestep(timestep);
        application.SetTryRealtime(true);
        double ChTime = 0.0;

        // utils::CSV_writer out("\t");
        //	out.stream().setf(std::ios::scientific | std::ios::showpos);
        out.stream().precision(7);
        while (application.GetDevice()->run()) {
            // Apply vertical load to rim body
            Rim->Empty_forces_accumulators();
            Rim->Accumulate_force(ChVector<>(0.0, 0.0, -5000.0), Rim->GetPos(), 0);
            application.BeginScene();
            application.DrawAll();
            application.DoStep();
            application.EndScene();
            if (!application.GetPaused()) {
                std::cout << "Time t = " << my_system.GetChTime() << "s \n";
                AccuNoIterations += mystepper->GetNumIterations();

                //==============================//
                //== Output programs ===========//
                //==============================//

                GetLog() << " t=  " << my_system.GetChTime() << "\n\n";
                NumCont = 0;
                NetContact.x = 0.0;
                NetContact.y = 0.0;
                NetContact.z = 0.0;

                // Loop over nodes to determine total number of contact nodes
                for (int ii = 0; ii < TotalNumNodes; ii++) {
                    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(ii));
                    if (nodetip->GetPos().z < ContactZ) {
                        NumCont++;
                    }
                }

                // Set number of contact nodes and collect net contact forces
                for (int i = 0; i < NumElements_y + 1; i++) {
                    LoadList[i]->NumContact = NumCont;
                    NetContact += LoadList[i]->NetContactForce;
                }

                // Output time history of nodal coordinates
                if (output) {
                    fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
                    for (int ii = 0; ii < TotalNumNodes; ii++) {
                        auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(ii));
                        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x);
                        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y);
                        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z);
                        fprintf(outputfile, "%15.7e  ", nodetip->GetD().x);
                        fprintf(outputfile, "%15.7e  ", nodetip->GetD().y);
                        fprintf(outputfile, "%15.7e  ", nodetip->GetD().z);
                    }

                    for (int ii = 0; ii < TotalNumNodes; ii++) {
                        auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(ii));
                        fprintf(outputfile, "%15.7e  ", nodetip->GetPos_dt().x);
                        fprintf(outputfile, "%15.7e  ", nodetip->GetPos_dt().y);
                        fprintf(outputfile, "%15.7e  ", nodetip->GetPos_dt().z);
                        fprintf(outputfile, "%15.7e  ", nodetip->GetD_dt().x);
                        fprintf(outputfile, "%15.7e  ", nodetip->GetD_dt().y);
                        fprintf(outputfile, "%15.7e  ", nodetip->GetD_dt().z);
                    }
                    fprintf(outputfile, "\n  ");
                }

                // Output time history of rigid bodies
                if (output1) {
                    fprintf(outputfile1, "%15.7e  ", my_system.GetChTime());
                    fprintf(outputfile1, "%15.7e  ", 0.0);
                    fprintf(outputfile1, "%15.7e  ", 0.0);
                    fprintf(outputfile1, "%15.7e  ", 0.0);
                    fprintf(outputfile1, "%15.7e  ", 1.0);
                    fprintf(outputfile1, "%15.7e  ", 0.0);
                    fprintf(outputfile1, "%15.7e  ", 0.0);
                    fprintf(outputfile1, "%15.7e  ", 0.0);
                    fprintf(outputfile1, "%15.7e  ", Rim->GetPos().x);
                    fprintf(outputfile1, "%15.7e  ", Rim->GetPos().y);
                    fprintf(outputfile1, "%15.7e  ", Rim->GetPos().z);
                    fprintf(outputfile1, "%15.7e  ", Rim->GetRot().e0);
                    fprintf(outputfile1, "%15.7e  ", Rim->GetRot().e1);
                    fprintf(outputfile1, "%15.7e  ", Rim->GetRot().e2);
                    fprintf(outputfile1, "%15.7e  ", Rim->GetRot().e3);
                    fprintf(outputfile1, "\n  ");
                }

                // Output ground location, net contact forces, and number of contact nodes
                if (output2) {
                    fprintf(outputfile2, "%15.7e  ", my_system.GetChTime());
                    fprintf(outputfile2, "%15.7e  ", ContactZ);
                    fprintf(outputfile2, "%15.7e  ", NetContact.x);
                    fprintf(outputfile2, "%15.7e  ", NetContact.y);
                    fprintf(outputfile2, "%15.7e  ", NetContact.z);
                    fprintf(outputfile2, "%d  ", NumCont);
                    fprintf(outputfile2, "\n  ");
                }

                // Output current time and number of iterations per time step
                if (output3) {
                    fprintf(outputfile3, "%15.7e  ", my_system.GetChTime());
                    fprintf(outputfile3, "%d  ", mystepper->GetNumIterations());
                    fprintf(outputfile3, "\n  ");
                }

                // Output time history contact forces per nodal coordinate
                if (output4) {
                    fprintf(outputfile4, "%15.7e  ", my_system.GetChTime());
                    for (int i = 0; i < NumElements_y + 1; i++) {
                        for (int j = 0; j < NumElements_x; j++) {
                            fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(0, j));
                            fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(1, j));
                            fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(2, j));
                            fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(3, j));
                            fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(4, j));
                            fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(5, j));
                        }
                    }
                    fprintf(outputfile4, "\n   ");
                }

                GetLog() << "Contact Line: " << ContactZ << "\n";
                GetLog() << "Contact ForceX: " << NetContact.x << "\n";
                GetLog() << "Contact ForceY: " << NetContact.y << "\n";
                GetLog() << "Contact ForceZ: " << NetContact.z << "\n";
                GetLog() << "Rim X: " << Rim->GetPos().x << "\n";
                GetLog() << "Rim Y: " << Rim->GetPos().y << "\n";
                GetLog() << "Rim Z: " << Rim->GetPos().z << "\n";
                GetLog() << "Rim Vel X: " << Rim->GetPos_dt().x << "\n";
                GetLog() << "Number of Contact Points: " << NumCont << "\n\n\n";

                // out << my_system.GetChTime() << Rim->GetPos().x << Rim->GetPos().y << Rim->GetPos().z<< std::endl;
                // out.write_to_file("../VertPosRim.txt");
            }
        }
    } else {
        ////////////////////////////////////////////////////
        double start = std::clock();
        while (my_system.GetChTime() < 0.2) {
            Rim->Empty_forces_accumulators();
            Rim->Accumulate_force(ChVector<>(0.0, 0.0, -5000.0), Rim->GetPos(), 0);
            //==Start analysis==//
            my_system.DoStepDynamics(timestep);

            //==============================//
            //== Output programs ===========//
            //==============================//

            GetLog() << " t=  " << my_system.GetChTime() << "\n\n";
            NumCont = 0;
            NetContact.x = 0.0;
            NetContact.y = 0.0;
            NetContact.z = 0.0;

            // Loop over nodes to determine total number of contact nodes
            for (int ii = 0; ii < TotalNumNodes; ii++) {
                auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(ii));
                if (nodetip->GetPos().z < ContactZ) {
                    NumCont++;
                }
            }

            // Set number of contact nodes and collect net contact forces
            for (int i = 0; i < NumElements_y + 1; i++) {
                LoadList[i]->NumContact = NumCont;
                NetContact += LoadList[i]->NetContactForce;
            }

            // Output time history of nodal coordinates
            if (output) {
                fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
                for (int ii = 0; ii < TotalNumNodes; ii++) {
                    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(ii));
                    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x);
                    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y);
                    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z);
                    fprintf(outputfile, "%15.7e  ", nodetip->GetD().x);
                    fprintf(outputfile, "%15.7e  ", nodetip->GetD().y);
                    fprintf(outputfile, "%15.7e  ", nodetip->GetD().z);
                }

                for (int ii = 0; ii < TotalNumNodes; ii++) {
                    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(ii));
                    fprintf(outputfile, "%15.7e  ", nodetip->GetPos_dt().x);
                    fprintf(outputfile, "%15.7e  ", nodetip->GetPos_dt().y);
                    fprintf(outputfile, "%15.7e  ", nodetip->GetPos_dt().z);
                    fprintf(outputfile, "%15.7e  ", nodetip->GetD_dt().x);
                    fprintf(outputfile, "%15.7e  ", nodetip->GetD_dt().y);
                    fprintf(outputfile, "%15.7e  ", nodetip->GetD_dt().z);
                }
                fprintf(outputfile, "\n  ");
            }

            // Output time history of rigid bodies
            if (output1) {
                fprintf(outputfile1, "%15.7e  ", my_system.GetChTime());
                fprintf(outputfile1, "%15.7e  ", 0.0);
                fprintf(outputfile1, "%15.7e  ", 0.0);
                fprintf(outputfile1, "%15.7e  ", 0.0);
                fprintf(outputfile1, "%15.7e  ", 1.0);
                fprintf(outputfile1, "%15.7e  ", 0.0);
                fprintf(outputfile1, "%15.7e  ", 0.0);
                fprintf(outputfile1, "%15.7e  ", 0.0);
                fprintf(outputfile1, "%15.7e  ", Rim->GetPos().x);
                fprintf(outputfile1, "%15.7e  ", Rim->GetPos().y);
                fprintf(outputfile1, "%15.7e  ", Rim->GetPos().z);
                fprintf(outputfile1, "%15.7e  ", Rim->GetRot().e0);
                fprintf(outputfile1, "%15.7e  ", Rim->GetRot().e1);
                fprintf(outputfile1, "%15.7e  ", Rim->GetRot().e2);
                fprintf(outputfile1, "%15.7e  ", Rim->GetRot().e3);
                fprintf(outputfile1, "\n  ");
            }

            // Output ground location, net contact forces, and number of contact nodes
            if (output2) {
                fprintf(outputfile2, "%15.7e  ", my_system.GetChTime());
                fprintf(outputfile2, "%15.7e  ", ContactZ);
                fprintf(outputfile2, "%15.7e  ", NetContact.x);
                fprintf(outputfile2, "%15.7e  ", NetContact.y);
                fprintf(outputfile2, "%15.7e  ", NetContact.z);
                fprintf(outputfile2, "%d  ", NumCont);
                fprintf(outputfile2, "\n  ");
            }

            // Output current time and number of iterations per time step
            if (output3) {
                fprintf(outputfile3, "%15.7e  ", my_system.GetChTime());
                fprintf(outputfile3, "%d  ", mystepper->GetNumIterations());
                fprintf(outputfile3, "\n  ");
            }

            // Output time history contact forces per nodal coordinate
            if (output4) {
                fprintf(outputfile4, "%15.7e  ", my_system.GetChTime());
                for (int i = 0; i < NumElements_y + 1; i++) {
                    for (int j = 0; j < NumElements_x; j++) {
                        fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(0, j));
                        fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(1, j));
                        fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(2, j));
                        fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(3, j));
                        fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(4, j));
                        fprintf(outputfile4, "%15.7e  ", LoadList[i]->NodeContactForce(5, j));
                    }
                }
                fprintf(outputfile4, "\n   ");
            }

            double nowtime = (std::clock() - start) / (double)CLOCKS_PER_SEC;
            GetLog() << "Sim Time: " << nowtime << "\n";
            GetLog() << "Contact Line: " << ContactZ << "\n";
            GetLog() << "Contact ForceX: " << NetContact.x << "\n";
            GetLog() << "Contact ForceY: " << NetContact.y << "\n";
            GetLog() << "Contact ForceZ: " << NetContact.z << "\n";
            GetLog() << "Rim X: " << Rim->GetPos().x << "\n";
            GetLog() << "Rim Y: " << Rim->GetPos().y << "\n";
            GetLog() << "Rim Z: " << Rim->GetPos().z << "\n";
            GetLog() << "Rim Vel X: " << Rim->GetPos_dt().x << "\n";
            GetLog() << "Number of Contact Points: " << NumCont << "\n\n\n";
        }
    }

    double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    auto mystepper1 = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    GetLog() << "Simulation Time: " << duration << "\n";
    fprintf(outputfile3, "%15.7e  ", duration);
    fprintf(outputfile3, "%d  ", mystepper1->GetNumIterations());
    fprintf(outputfile3, "\n  ");

    system("pause");
    return 0;
}