//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that c%an be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSPARSEMATRIX_H
#define CHSPARSEMATRIX_H

//////////////////////////////////////////////////
//
//   ChLinkedListMatrix.h
//
//   Base matrix class for all sparse matrices:
//		- ChLinkedListMatrix
//      - ChCSR3Matrix
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "core/ChMatrix.h"
//#include "core/ChApiCE.h"

// default predicted density, for allocation, 0<..<1 , default 1/10 th
#define SPM_DEF_FULLNESS 0.1
// default limit on initial number of extradiagonal elements, for allocation, default 10000
#define SPM_DEF_MAXELEMENTS 10000

namespace chrono {
	
	class ChApi ChSparseMatrix {
	protected:
		int rows;
		int columns;
		double dummy;

	public:
		ChSparseMatrix(): rows(-1), columns(-1), dummy(0.0) {};
		virtual ~ChSparseMatrix(){};

		virtual void SetElement(int insrow, int inscol, double insval, bool overwrite = true){assert(0);};
		virtual double GetElement(int row, int col) { assert(0);	return 0; }; // can't be const because of implementation in ChLinkedListMatrix

		virtual double& Element(int row, int col){ assert(0); return dummy; };

		virtual void PasteMatrix(ChMatrix<>* matra, int insrow, int inscol, bool overwrite = true, bool transp = false)	{ assert(0);};
		virtual void PasteMatrixFloat(ChMatrix<float>* matra, int insrow, int inscol, bool overwrite = true, bool transp = false){assert(0);};
		virtual void PasteClippedMatrix(ChMatrix<>* matra, int cliprow, int clipcol, int nrows, int ncolumns, int insrow, int inscol, bool overwrite = true){assert(0);};
		
		virtual void Reset(int row, int col, int nonzeros = 0){ assert(0); };
		virtual bool Resize(int nrows, int ncols, int nonzeros = 0){ assert(0); return 1; };

		// Redirected functions
		virtual void PasteTranspMatrix(ChMatrix<>* matra, int insrow, int inscol){ PasteMatrix(matra, insrow, inscol, true, true); };
		virtual void PasteSumMatrix(ChMatrix<>* matra, int insrow, int inscol){ PasteMatrix(matra, insrow, inscol, false, false); };
		virtual void PasteSumTranspMatrix(ChMatrix<>* matra, int insrow, int inscol){ PasteMatrix(matra, insrow, inscol, false, true); };
		virtual void PasteTranspMatrixFloat(ChMatrix<float>* matra, int insrow, int inscol){ PasteMatrixFloat(matra, insrow, inscol, true, true);  };
		virtual void PasteSumClippedMatrix(ChMatrix<>* matra, int cliprow, int clipcol, int nrows, int ncolumns, int insrow, int inscol){ PasteClippedMatrix(matra, cliprow, clipcol, nrows, ncolumns, insrow, inscol, false); };

		virtual int GetRows() const { return rows; };
		virtual int GetColumns() const { return columns; };

	};

}  // END_OF_NAMESPACE____

#endif
