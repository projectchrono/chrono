// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHFUNCT_BASE_H
#define CHFUNCT_BASE_H

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>
#include <list>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMath.h"
#include "chrono/physics/ChFilePS.h"
#include "chrono/physics/ChProplist.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Interface base class for scalar functions of the type:
///    y= f(x)
///
/// The ChFunction class defines the base class for all Chrono
/// functions of type y=f(x), that is scalar functions of an
/// input variable x (usually, the time). ChFunctions are often
/// used to set time-dependent properties, for example to set
/// motion laws in linear actuators, engines, etc.
/// This base class just represent a constant function of
/// the type y= C.  Inherited classes must override at least the
/// Get_y() method, in order to represent more complex functions.

class ChApi ChFunction {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI_ROOT(ChFunction_base);

  public:
    ChFunction() {}
    ChFunction(const ChFunction& other) {}
    virtual ~ChFunction() {}

    /// "Virtual" copy constructor.
    virtual ChFunction* Clone() const = 0;

    /// Each class inherited from the ChFunction class must
    /// return an unique integer identifier with the virtual
    /// function Get_type(). This is useful for fast run-time downcasting etc.
    virtual int Get_Type() const { return -1; }

    // THE MOST IMPORTANT MEMBER FUNCTIONS
    // At least Get_y() should be overridden by derived classes.

    /// Returns the y value of the function, at position x.
    virtual double Get_y(double x) const = 0;

    /// Returns the dy/dx derivative of the function, at position x.
    /// Note that inherited classes may also avoid overriding this method,
    /// because this base method already provide a general-purpose numerical differentiation
    /// to get dy/dx only from the Get_y() function. (however, if the analytical derivative
    /// is known, it may better to implement a custom method).
    virtual double Get_y_dx(double x) const { return ((Get_y(x + BDF_STEP_LOW) - Get_y(x)) / BDF_STEP_LOW); }

    /// Returns the ddy/dxdx double derivative of the function, at position x.
    ///  Note that inherited classes may also avoid overriding this method,
    /// because this base method already provide a general-purpose numerical differentiation
    /// to get ddy/dxdx only from the Get_y() function. (however, if the analytical derivative
    /// is known, it may be better to implement a custom method).
    virtual double Get_y_dxdx(double x) const { return ((Get_y_dx(x + BDF_STEP_LOW) - Get_y_dx(x)) / BDF_STEP_LOW); };

    /// Returns the weight of the function (useful for
    /// applications where you need to mix different weighted ChFunctions)
    virtual double Get_weight(double x) const { return 1.0; };

    /// These functions can be used to implement automatic zooming
    /// on the most representative range of function (if GUI is implemented)
    virtual void Estimate_x_range(double& xmin, double& xmax) const {
        xmin = 0.0;
        xmax = 1.2;
    }

    virtual void Estimate_y_range(double xmin, double xmax, double& ymin, double& ymax, int derivate) const;

    /// Generic derivative: if derivate=0 is like Get_y(), if derivate=1
    /// is like Get_y_dx(), etc.
    /// Note: in current release 'derivate' can be only 0,1,2
    virtual double Get_y_dN(double x, int derivate) const;

    //
    // Some analysis functions. If derivate=0, they are applied on y(x), if derivate =1, on dy/dx, etc.
    //

    /// Computes the maximum of y(x) in a range xmin-xmax, using a sampling method.
    virtual double Compute_max(double xmin, double xmax, double sampling_step, int derivate) const;
    /// Computes the minimum of y(x) in a range xmin-xmax, using a sampling method.
    virtual double Compute_min(double xmin, double xmax, double sampling_step, int derivate) const;
    /// Computes the mean value of y(x) in a range xmin-xmax, using a sampling method.
    virtual double Compute_mean(double xmin, double xmax, double sampling_step, int derivate) const;
    /// Computes the square mean val. of y(x) in a range xmin-xmax, using sampling.
    virtual double Compute_sqrmean(double xmin, double xmax, double sampling_step, int derivate) const;
    /// Computes the integral of y(x) in a range xmin-xmax, using a sampling method.
    virtual double Compute_int(double xmin, double xmax, double sampling_step, int derivate) const;
    /// Computes the positive acceleration coefficient (inherited classes should customize this).
    virtual double Get_Ca_pos() const { return 0; }
    /// Computes the positive acceleration coefficient (inherited classes should customize this).
    virtual double Get_Ca_neg() const { return 0; }
    /// Computes the speed coefficient (inherited classes must customize this).
    virtual double Get_Cv() const { return 0; }

    // If the function has some handles (mouse-sensible markers on screen), implement these functions

    /// Returns the number of handles of the function
    virtual int HandleNumber() const { return 0; }

    /// Gets the x and y position of handle, given identifier.
    /// If set mode, x and y values are stored. Return false if handle not found.
    virtual int HandleAccess(int handle_id, double mx, double my, bool set_mode) { return TRUE; }

    /// Method to allow serialization of transient data to archives
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);

    /// Plot function in graph space of the ChFile_ps postscript file
    /// where zoom factor, centering, colour, thickness etc. are already defined.
    /// If plotDY=true, plots also the derivative, etc.
    virtual int FilePostscriptPlot(ChFile_ps* m_file, int plotY, int plotDY, int plotDDY);

    /// Save function as X-Y pairs separated by space, with CR at each pair,
    /// into an Ascii file.
    /// The output file can be later loaded into Excel, GnuPlot or other tools.
    /// The function is 'sampled' for nsteps times, from xmin to xmax.
    virtual int FileAsciiPairsSave(ChStreamOutAscii& m_file, double xmin = 0, double xmax = 1, int msamples = 200);
};

/// @} chrono_functions

}  // end namespace chrono

#endif
