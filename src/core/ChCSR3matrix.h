#ifndef CHCSR3MATRIX_H
#define CHCSR3MATRIX_H


#include <Eigen\Sparse>

namespace chrono{
	class ChEigenMatrix : public Eigen::SparseMatrix < double, 1 > {
	public:
		ChEigenMatrix() : Eigen::SparseMatrix<double,1>() {};
		ChEigenMatrix(Index rows, Index cols) : Eigen::SparseMatrix<double,1>(rows, cols){};
		ChEigenMatrix(Index dimension) : Eigen::SparseMatrix<double, 1>(dimension, dimension){};
		/*template<typename OtherDerived>
		ChEigenMatrix(const Eigen::SparseMatrixBase<SparseMatrix<OtherDerived>>& other) : Eigen::SparseMatrix<other>(){};
		template<typename OtherDerived>
		ChEigenMatrix& operator= (const Eigen::SparseMatrixBase<SparseMatrix<OtherDerived>>& other)	{
			this->Eigen::SparseMatrix<double>::operator=(other);
			return *this;
		};*/



	public:
		template <bool overwrite = 1>
		inline void SetElement(int insrow, int inscol, double insval){
			if (overwrite) coeffRef(insrow, inscol) = insval;
			else coeffRef(insrow, inscol) += insval;
		};

		inline double& Element(int row, int col){
			return coeffRef(row, col);
		};

		inline double* GetValueArray(){
			return this->valuePtr();
		}

		inline int* GetColumnIndex(){
			return this->innerIndexPtr();
		}

		inline int* GetRowIndex(){
			return this->outerIndexPtr();
		}


		template <bool overwrite = 1, class ChMatrixIN>
		void PasteMatrix(ChMatrixIN* matra, int insrow, int inscol){
			int maxrows = matra->GetRows();
			int maxcols = matra->GetColumns();
			int i, j;

			// can't use triplets because they expect a compressed matrix with
			// non existing entries

			for (i = 0; i < maxrows; i++){
				for (j = 0; j < maxcols; j++){
					if ((*matra)(i, j)!=0) this->SetElement<overwrite>(insrow + i, inscol + j, (*matra)(i, j));
				};
			};
		};

		template <bool overwrite = 1, class ChMatrixIN>
		void PasteMatrixFloat(ChMatrixIN* matra, int insrow, int inscol){ PasteMatrix(matra, insrow, inscol); };

		template <class ChMatrixIN>
		inline void PasteSumMatrix(ChMatrixIN* matra, int insrow, int inscol){ PasteMatrix<0>(matra, insrow, inscol); };

		template <bool overwrite = 1, class ChMatrixIN>
		void PasteTranspMatrix(ChMatrixIN* matra, int insrow, int inscol){
			int maxrows = matra->GetRows();
			int maxcols = matra->GetColumns();
			int i, j;

			// can't use triplets because they expect a compressed matrix with
			// non existing entries

			for (i = 0; i < maxcols; i++){
				for (j = 0; j < maxrows; j++){
					if ((*matra)(j, i) != 0) this->SetElement<overwrite>(insrow + i, inscol + j, (*matra)(j, i));
				};
			};
		};

		template <class ChMatrixIN>
		inline void PasteSumTranspMatrix(ChMatrixIN* matra, int insrow, int inscol){ PasteTranspMatrix<0>(matra, insrow, inscol); };

		template <class ChMatrixIN>
		inline void PasteTranspMatrixFloat(ChMatrixIN* matra, int insrow, int inscol) { PasteTranspMatrix(matra, insrow, inscol); };

		// GetElement returns 0 also if indexes are out of bound!
		inline double GetElement(const int row, const int col) const{
			return this->coeff(row, col);
		};


		inline int GetRows(){
			return (int) this->rows();
		}

		inline int GetColumns(){
			return (int) this->cols();
		}


		template <bool overwrite = 1, class ChMatrixIN>
		void PasteClippedMatrix(const ChMatrixIN* matra, int cliprow, int clipcol, int nrows, int ncolumns, int insrow, int inscol) {
		/*#pragma omp parallel for if (nrows > CH_OMP_MATR)
			for (int i = 0; i < nrows; ++i)
				for (int j = 0; j < ncolumns; ++j)
					Element(insrow + i, inscol + j) = matra->Element(i + cliprow, j + clipcol);*/

		/*#pragma omp parallel for if (nrows > CH_OMP_MATR)*/
			for (int i = 0; i < nrows; ++i)
				for (int j = 0; j < ncolumns; ++j)
					this->SetElement<overwrite>(insrow + i, inscol + j, matra->GetElement(i + cliprow, j + clipcol));

		}

		template <class ChMatrixIN>
		inline void PasteSumClippedMatrix(const ChMatrixIN* matra, int cliprow, int clipcol, int nrows, int ncolumns, int insrow, int inscol) {
			PasteClippedMatrix<0>(matra, cliprow, clipcol, nrows, ncolumns, insrow, inscol);
		}

		double& operator()(const int row, const int col) { return Element(row, col); }
		double& operator()(const int index)              { return Element( (int)index/cols(), index % cols() ); }


		inline void Reset(int new_rows, int new_cols){
			if ( (new_rows != this->rows()) || (new_cols != this->cols()) )
				resize(new_rows, new_cols);
		}

		inline void Reset(int){}; // square matrix

	}; // END class
}; // END namespace

#endif