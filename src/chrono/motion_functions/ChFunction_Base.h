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

#ifndef CHFUNCT_BASE_H
#define CHFUNCT_BASE_H

//////////////////////////////////////////////////
//
//   ChFunction_Base.h
//
//   Base class for all ChFunction objects,
//   as scalar functions of scalar variable y=f(t)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>
#include <list>

#include "core/ChApiCE.h"
#include "core/ChMath.h"
#include "physics/ChFilePS.h"
#include "physics/ChProplist.h"

namespace chrono {

#define CHF_FILE_HEADER_ID 1234.56

#define OPT_VARIABLES_START                  \
    virtual const char** GetOptVariables() { \
    static const char* mOptVars[] = {
#define OPT_VARIABLES_END \
    0                     \
    }                     \
    ;                     \
    return mOptVars;      \
    }

/// @addtogroup chrono_functions
/// @{

/// THE INTERFACE BASE CLASS FOR SCALAR FUNCTIONS OF TYPE:
///
///  y= f(x)
///
///
///  The ChFunction class defines the base class for all Chrono
/// functions of type y=f(x), that is scalar functions of an
/// input variable x (usually, the time). ChFunctions are often
/// used to set time-dependent properties, for example to set
/// motion laws in linear actuators, engines, etc.
///  This base class just represent a constant function of
/// the type y= C.  Inherited classes must override at least the
/// Get_y() method, in order to represent more complex functions.
class ChApi ChFunction {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI_ROOT(ChFunction_base);

  public:
    ChFunction(){};
    virtual ~ChFunction(){};
    // virtual void Copy (ChFunction* source) =0;
    virtual ChFunction* new_Duplicate() = 0;

    /// Each class inherited from the ChFunction class must
    /// return an unique integer identifier with the virtual
    /// function Get_type(). This is useful for fast run-time downcasting etc.
    virtual int Get_Type() { return -1; }

    ///// THE MOST IMPORTANT MEMBER FUNCTIONS /////
    //       At least Get_y() should be overridden
    //       by inherited classes.

    /// Returns the y value of the function, at position x. (For this
    /// base class, it will be a constant value y=C).
    virtual double Get_y(double x) = 0;

    /// Returns the dy/dx derivative of the function, at position x.
    ///  Note that inherited classes may also avoid overriding this method,
    /// because this base method already provide a general-purpose numerical differentiation
    /// to get dy/dx only from the Get_y() function. (however, if the analytical derivative
    /// is known, it may better to implement a custom method).
    virtual double Get_y_dx(double x) { return ((Get_y(x + BDF_STEP_LOW) - Get_y(x)) / BDF_STEP_LOW); }

    /// Returns the ddy/dxdx double derivative of the function, at position x.
    ///  Note that inherited classes may also avoid overriding this method,
    /// because this base method already provide a general-purpose numerical differentiation
    /// to get ddy/dxdx only from the Get_y() function. (however, if the analytical derivative
    /// is known, it may be better to implement a custom method).
    virtual double Get_y_dxdx(double x) { return ((Get_y_dx(x + BDF_STEP_LOW) - Get_y_dx(x)) / BDF_STEP_LOW); };

    //
    ////////////////////////////////////////

    /// Returns the weight of the function (useful for
    /// applications where you need to mix different weighted ChFunctions)
    virtual double Get_weight(double x) { return 1.0; };

    /// These functions can be used to implement automatic zooming
    /// on the most representative range of function (if GUI is implemented)
    virtual void Estimate_x_range(double& xmin, double& xmax) {
        xmin = 0.0;
        xmax = 1.2;
    };
    virtual void Estimate_y_range(double xmin, double xmax, double& ymin, double& ymax, int derivate);

    /// Generic derivative: if derivate=0 is like Get_y(), if derivate=1
    /// is like Get_y_dx(), etc.
    /// Note: in current release 'derivate' can be only 0,1,2
    virtual double Get_y_dN(double x, int derivate);

    //
    // Some analysis functions. If derivate=0, they are applied on y(x), if derivate =1, on dy/dx, etc.
    //

    /// Computes the maximum of y(x) in a range xmin-xmax, using a sampling method.
    virtual double Compute_max(double xmin, double xmax, double sampling_step, int derivate);
    /// Computes the minimum of y(x) in a range xmin-xmax, using a sampling method.
    virtual double Compute_min(double xmin, double xmax, double sampling_step, int derivate);
    /// Computes the mean value of y(x) in a range xmin-xmax, using a sampling method.
    virtual double Compute_mean(double xmin, double xmax, double sampling_step, int derivate);
    /// Computes the square mean val. of y(x) in a range xmin-xmax, using sampling.
    virtual double Compute_sqrmean(double xmin, double xmax, double sampling_step, int derivate);
    /// Computes the integral of y(x) in a range xmin-xmax, using a sampling method.
    virtual double Compute_int(double xmin, double xmax, double sampling_step, int derivate);
    /// Computes the positive acceleration coefficient (inherited classes should customize this).
    virtual double Get_Ca_pos() { return 0; };
    /// Computes the positive acceleration coefficient (inherited classes should customize this).
    virtual double Get_Ca_neg() { return 0; };
    /// Computes the speed coefficient (inherited classes must customize this).
    virtual double Get_Cv() { return 0; };

    // Functions which build the tree of optimization variables (or expands it, from given tree)
    // Each function class should implement (at least) the OPT_VARIABLES_START/END macro, that is return a
    // null-terminated
    // array of variables which can be get/set as floating point during optimizations.
    // Variables are strings in C++ /Java/Javascript syntax, that must be interpreted with
    // some scripting engine (ex. the Javascript one); see OptVariablesToVector and VectorToOptVariables js tools.

    virtual int MakeOptVariableTree(ChList<chjs_propdata>* mtree);
    static int VariableTreeToFullNameVar(ChList<chjs_propdata>* mtree, ChList<chjs_fullnamevar>* mlist);
    virtual int OptVariableCount();

    OPT_VARIABLES_START    // here expose no vars ('C' var is added hard-wired in MakeOptVariableTree)
        OPT_VARIABLES_END  // Inherited classes, at least implement this. If they enclose children funct. obj, also
                           // implement the MakeOptVariableTree().

        /// If the function has some handles (mouse-sensible markers on screen),
        /// implement these functions:
        virtual int
        HandleNumber() {
        return 0;
    }  // Returns the number of handles of the function
       /// Gets the x and y position of handle, given identifier.
       /// If set mode, x and y values are stored. Return false if handle not found.
    virtual int HandleAccess(int handle_id, double mx, double my, bool set_mode) { return TRUE; };

    //
    // SERIALIZATION
    //

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

}  // END_OF_NAMESPACE____

#endif
