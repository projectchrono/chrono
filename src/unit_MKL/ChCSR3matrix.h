#ifndef CHCSR3MATRIX_H
#define CHCSR3MATRIX_H

#include <Eigen/Sparse>
#include "core/ChSpmatrix.h"


namespace chrono{


	class ChEigenMatrixPTR : public ChSparseMatrixBase, public Eigen::SparseMatrix<double, Eigen::RowMajor, int> {
	public:
		ChEigenMatrixPTR() : Eigen::SparseMatrix<double, Eigen::RowMajor, int>() {};
		ChEigenMatrixPTR(int rows, int cols) : Eigen::SparseMatrix<double, Eigen::RowMajor, int>(rows, cols) {};
		template<class SizesType>
		ChEigenMatrixPTR(int rows, int cols, SizesType& reserveSize) : Eigen::SparseMatrix<double, Eigen::RowMajor, int>(rows, cols) { reserve(reserveSize); };
		ChEigenMatrixPTR(int dimension) : Eigen::SparseMatrix<double, Eigen::RowMajor, int>(dimension, dimension) {};

		/*template<typename OtherDerived>
		ChEigenMatrix(const Eigen::SparseMatrixBase<SparseMatrix<OtherDerived>>& other) : Eigen::SparseMatrix<other>(){};

		template<typename OtherDerived>
		ChEigenMatrix& operator= (const Eigen::SparseMatrixBase<SparseMatrix<OtherDerived>>& other)	{
			this->Eigen::SparseMatrix<double>::operator=(other);
			return *this;
		};*/
		
		/// Templatized functions
		template <bool overwrite = 1>
		void SetElement(int insrow, int inscol, double insval){
			if (overwrite) coeffRef(insrow, inscol) = insval;
			else coeffRef(insrow, inscol) += insval;
		};

		template <>
		void SetElement<1>(int insrow, int inscol, double insval){
			coeffRef(insrow, inscol) = insval;
		};

		template <>
		void SetElement<0>(int insrow, int inscol, double insval){
			coeffRef(insrow, inscol) += insval;
		};

		inline double& Element(int row, int col){
			return coeffRef(row, col);
		};

		inline double* GetValueArray(){
			return valuePtr();
		}

		inline int* GetColumnIndex(){
			return innerIndexPtr();
		}

		inline int* GetRowIndex(){
			return outerIndexPtr();
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
				}
			}
		}

		template <bool overwrite = 1, class ChMatrixIN>
		void PasteMatrixFloat(ChMatrixIN* matra, int insrow, int inscol){ PasteMatrix(matra, insrow, inscol); };

		
		template <class ChMatrixIN>
		void PasteSumMatrix(ChMatrixIN* matra, int insrow, int inscol){ PasteMatrix<0>(matra, insrow, inscol); };

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
		void PasteSumTranspMatrix(ChMatrixIN* matra, int insrow, int inscol){ PasteTranspMatrix<0>(matra, insrow, inscol); };

		template <class ChMatrixIN>
		void PasteTranspMatrixFloat(ChMatrixIN* matra, int insrow, int inscol) { PasteTranspMatrix(matra, insrow, inscol); };

		// GetElement returns 0 also if indexes are out of bound!
		//***TODO*** see http://eigen.tuxfamily.org/dox/group__TutorialSparse.html for a better way to find elements
		double GetElement(const int row, const int col) const{
			return this->coeff(row, col);
		};

		inline int GetRows(){
			return static_cast<int>(this->rows());
		}

		inline int GetColumns(){
			return static_cast<int>(this->cols());
		}

		//void makeCompressed(); // if we choose to have Eigen::SparseMatrix as PRIVATE base


		template <bool overwrite = 1, class ChMatrixIN>
		void PasteClippedMatrix(ChMatrixIN* matra, int cliprow, int clipcol, int nrows, int ncolumns, int insrow, int inscol) {
		/*#pragma omp parallel for if (nrows > CH_OMP_MATR)*/
			for (int i = 0; i < nrows; ++i)
				for (int j = 0; j < ncolumns; ++j)
					this->SetElement<overwrite>(insrow + i, inscol + j, matra->GetElement(i + cliprow, j + clipcol));
		}

		template <class ChMatrixIN>
		void PasteSumClippedMatrix(ChMatrixIN* matra, int cliprow, int clipcol, int nrows, int ncolumns, int insrow, int inscol) {
			PasteClippedMatrix<0>(matra, cliprow, clipcol, nrows, ncolumns, insrow, inscol);
		}

		double& operator()(const int row, const int col) { return Element(row, col); }
		double& operator()(const int index)              { return Element( (int)index/cols(), index % cols() ); }

		void Reset(int new_rows, int new_cols){
			if ( (new_rows != this->rows()) || (new_cols != this->cols()) )
				resize(new_rows, new_cols);
		}

		void Reset(int mat_size){
			Reset(mat_size, mat_size);
		}



		// Import function from ChSparseMatrix format; this function resets the matrix!
		void LoadFromChSparseMatrix(ChSparseMatrix* mat)
		{
			// Create the CSR3 matrix with just the right amount of elements
			int mat_rows = mat->GetRows();
			resize(mat_rows, mat->GetColumns());	// rectangular matrices allowed; matrix still empty
			std::vector<int> reserveSize(mat_rows); // will take the number of elements for each row
			mat->CountNonZeros<std::vector<int>>(reserveSize);
			reserve(reserveSize);

			// Import values from ChSparseMatrix
			ChMelement* elarray = mat->GetElarrayDereferenced();
			ChMelement* el_temp;
			for (int i = 0; i < mat_rows; i++){
				el_temp = &elarray[i];
				while (el_temp){
					if (el_temp->val != 0)
						SetElement(i, el_temp->col, el_temp->val);
					el_temp = el_temp->next;
				};

			};

		} // END LoadFromChSparseMatrix;

		void LoadFromChSparseMatrix(ChSparseMatrix* M, ChSparseMatrix* Cq, ChSparseMatrix* E)
		{
			// Create the CSR3 matrix
			int M_rows = M->GetRows();
			int Cq_rows = Cq->GetRows();
			int mat_rows = M_rows + Cq_rows;
			resize(mat_rows, mat_rows); // only square matrices allowed

			// Preallocate a ChEigenMatrix with the exact number of non-zeros of the ChSparseMatrix
			std::vector<int> reserveSize(mat_rows); // will take the number of elements for each row
			ChMelement* Cq_elarray = Cq->GetElarrayDereferenced();
			ChMelement* el_temp;

			// scan Cq matrix for non-zeros
			for (int i = 0; i < Cq_rows; i++){
				el_temp = &Cq_elarray[i];
				while (el_temp){
					if (el_temp->val != 0){
						reserveSize[M_rows + i]++;
						reserveSize[el_temp->col]++;
					};
					el_temp = el_temp->next;
				}
			}

			M->CountNonZeros<std::vector<int>>(reserveSize);
			E->CountNonZeros<std::vector<int>>(reserveSize, M_rows);

			reserve(reserveSize);

			// Import values from ChSparseMatrix

			for (int i = 0; i < Cq_rows; i++){
				el_temp = &Cq_elarray[i];
				while (el_temp){
					if (el_temp->val != 0){
						SetElement(M_rows + i, el_temp->col, el_temp->val); // sets the Cq
						SetElement(el_temp->col, M_rows + i, el_temp->val); // sets the Cq'
					};
					el_temp = el_temp->next;
				}
			}

			ChMelement* M_elarray = M->GetElarrayDereferenced();
			for (int i = 0; i < M_rows; i++){
				el_temp = &M_elarray[i];
				while (el_temp){
					if (el_temp->val != 0)
						SetElement(i, el_temp->col, el_temp->val);
					el_temp = el_temp->next;
				}
			}

			ChMelement* E_elarray = E->GetElarrayDereferenced();
			for (int i = 0; i < Cq_rows; i++){
				el_temp = &E_elarray[i];
				while (el_temp){
					if (el_temp->val != 0)
						SetElement(i + M_rows, M_rows + el_temp->col, el_temp->val);
					el_temp = el_temp->next;
				}
			}
		} // END LoadFromChSparseMatrix

	}; // END class


	class ChEigenMatrix : public ChSparseMatrixBase, public Eigen::SparseMatrix<double, Eigen::RowMajor, int> {
	public:
		ChEigenMatrix() : Eigen::SparseMatrix<double, Eigen::RowMajor, int>() {};
		ChEigenMatrix(int rows, int cols) : Eigen::SparseMatrix<double, Eigen::RowMajor, int>(rows, cols) {};
		template<class SizesType>
		ChEigenMatrix(int rows, int cols, SizesType& reserveSize) : Eigen::SparseMatrix<double, Eigen::RowMajor, int>(rows, cols) { reserve(reserveSize); };
		ChEigenMatrix(int dimension) : Eigen::SparseMatrix<double, Eigen::RowMajor, int>(dimension, dimension) {};
		virtual ~ChEigenMatrix(){};

		/*template<typename OtherDerived>
		ChEigenMatrix(const Eigen::SparseMatrixBase<SparseMatrix<OtherDerived>>& other) : Eigen::SparseMatrix<other>(){};

		template<typename OtherDerived>
		ChEigenMatrix& operator= (const Eigen::SparseMatrixBase<SparseMatrix<OtherDerived>>& other)	{
		this->Eigen::SparseMatrix<double>::operator=(other);
		return *this;
		};*/

		/// Virtualizabile functions
		virtual void SetElement(int insrow, int inscol, double insval) override {
			coeffRef(insrow, inscol) = insval;
		};

		virtual void PasteMatrix(ChMatrix<>* matra, int insrow, int inscol) override
		{
			int maxrows = matra->GetRows();
			int maxcols = matra->GetColumns();
			int i, j;

			// can't use triplets because they expect a compressed matrix with
			// non existing entries

			for (i = 0; i < maxrows; i++){
				for (j = 0; j < maxcols; j++){
					if ((*matra)(i, j) != 0) this->SetElement<1>(insrow + i, inscol + j, (*matra)(i, j));
				}
			}
		}

		virtual void PasteMatrixFloat(ChMatrix<float>* matra, int insrow, int inscol) override
		{
			int maxrows = matra->GetRows();
			int maxcols = matra->GetColumns();
			int i, j;

			// can't use triplets because they expect a compressed matrix with
			// non existing entries

			for (i = 0; i < maxrows; i++){
				for (j = 0; j < maxcols; j++){
					if ((*matra)(i, j) != 0) this->SetElement<1>(insrow + i, inscol + j, (*matra)(i, j));
				}
			}
		};


		virtual void PasteSumMatrix(ChMatrix<>* matra, int insrow, int inscol) override
		{
			int maxrows = matra->GetRows();
			int maxcols = matra->GetColumns();
			int i, j;

			// can't use triplets because they expect a compressed matrix with
			// non existing entries

			for (i = 0; i < maxrows; i++){
				for (j = 0; j < maxcols; j++){
					if ((*matra)(i, j) != 0) this->SetElement<0>(insrow + i, inscol + j, (*matra)(i, j));
				}
			}
		};

		virtual void PasteTranspMatrix(ChMatrix<>* matra, int insrow, int inscol) override
		{
			int maxrows = matra->GetRows();
			int maxcols = matra->GetColumns();
			int i, j;

			// can't use triplets because they expect a compressed matrix with
			// non existing entries

			for (i = 0; i < maxcols; i++){
				for (j = 0; j < maxrows; j++){
					if ((*matra)(j, i) != 0) this->SetElement<1>(insrow + i, inscol + j, (*matra)(j, i));
				}
			}
		}

		virtual void PasteSumTranspMatrix(ChMatrix<>* matra, int insrow, int inscol) override
		{
			int maxrows = matra->GetRows();
			int maxcols = matra->GetColumns();
			int i, j;

			// can't use triplets because they expect a compressed matrix with
			// non existing entries

			for (i = 0; i < maxcols; i++){
				for (j = 0; j < maxrows; j++){
					if ((*matra)(j, i) != 0) this->SetElement<0>(insrow + i, inscol + j, (*matra)(j, i));
				}
			}
		};

		virtual void PasteTranspMatrixFloat(ChMatrix<float>* matra, int insrow, int inscol) override
		{
			int maxrows = matra->GetRows();
			int maxcols = matra->GetColumns();
			int i, j;

			// can't use triplets because they expect a compressed matrix with
			// non existing entries

			for (i = 0; i < maxcols; i++){
				for (j = 0; j < maxrows; j++){
					if ((*matra)(j, i) != 0) this->SetElement<1>(insrow + i, inscol + j, (*matra)(j, i));
				}
			}
		}

		virtual void PasteClippedMatrix(ChMatrix<>* matra, int cliprow, int clipcol, int nrows, int ncolumns, int insrow, int inscol) override
		{
			/*#pragma omp parallel for if (nrows > CH_OMP_MATR)*/
			for (int i = 0; i < nrows; ++i)
				for (int j = 0; j < ncolumns; ++j)
					this->SetElement<1>(insrow + i, inscol + j, matra->GetElement(i + cliprow, j + clipcol));
		}

		virtual void PasteSumClippedMatrix(ChMatrix<>* matra, int cliprow, int clipcol, int nrows, int ncolumns, int insrow, int inscol) override
		{
			/*#pragma omp parallel for if (nrows > CH_OMP_MATR)*/
			for (int i = 0; i < nrows; ++i)
				for (int j = 0; j < ncolumns; ++j)
					this->SetElement<0>(insrow + i, inscol + j, matra->GetElement(i + cliprow, j + clipcol));
		}

		/// Templatized functions
		template <bool overwrite = 1>
		void SetElement(int insrow, int inscol, double insval){
			if (overwrite) coeffRef(insrow, inscol) = insval;
			else coeffRef(insrow, inscol) += insval;
		};

		template <>
		void SetElement<1>(int insrow, int inscol, double insval){
			coeffRef(insrow, inscol) = insval;
		};

		template <>
		void SetElement<0>(int insrow, int inscol, double insval){
			coeffRef(insrow, inscol) += insval;
		};

		inline double& Element(int row, int col){
			return coeffRef(row, col);
		};

		inline double* GetValueArray(){
			return valuePtr();
		}

		inline int* GetColumnIndex(){
			return innerIndexPtr();
		}

		inline int* GetRowIndex(){
			return outerIndexPtr();
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
					if ((*matra)(i, j) != 0) this->SetElement<overwrite>(insrow + i, inscol + j, (*matra)(i, j));
				}
			}
		}

		template <bool overwrite = 1, class ChMatrixIN>
		void PasteMatrixFloat(ChMatrixIN* matra, int insrow, int inscol){ PasteMatrix(matra, insrow, inscol); };


		template <class ChMatrixIN>
		void PasteSumMatrix(ChMatrixIN* matra, int insrow, int inscol){ PasteMatrix<0>(matra, insrow, inscol); };

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
		void PasteSumTranspMatrix(ChMatrixIN* matra, int insrow, int inscol){ PasteTranspMatrix<0>(matra, insrow, inscol); };

		template <class ChMatrixIN>
		void PasteTranspMatrixFloat(ChMatrixIN* matra, int insrow, int inscol) { PasteTranspMatrix(matra, insrow, inscol); };

		// GetElement returns 0 also if indexes are out of bound!
		//***TODO*** see http://eigen.tuxfamily.org/dox/group__TutorialSparse.html for a better way to find elements
		double GetElement(const int row, const int col) const{
			return this->coeff(row, col);
		};

		inline int GetRows(){
			return static_cast<int>(this->rows());
		}

		inline int GetColumns(){
			return static_cast<int>(this->cols());
		}

		//void makeCompressed(); // if we choose to have Eigen::SparseMatrix as PRIVATE base


		template <bool overwrite = 1, class ChMatrixIN>
		void PasteClippedMatrix(ChMatrixIN* matra, int cliprow, int clipcol, int nrows, int ncolumns, int insrow, int inscol) {
			/*#pragma omp parallel for if (nrows > CH_OMP_MATR)*/
			for (int i = 0; i < nrows; ++i)
				for (int j = 0; j < ncolumns; ++j)
					this->SetElement<overwrite>(insrow + i, inscol + j, matra->GetElement(i + cliprow, j + clipcol));
		}

		template <class ChMatrixIN>
		void PasteSumClippedMatrix(ChMatrixIN* matra, int cliprow, int clipcol, int nrows, int ncolumns, int insrow, int inscol) {
			PasteClippedMatrix<0>(matra, cliprow, clipcol, nrows, ncolumns, insrow, inscol);
		}

		double& operator()(const int row, const int col) { return Element(row, col); }
		double& operator()(const int index)              { return Element((int)index / cols(), index % cols()); }

		void Reset(int new_rows, int new_cols){
			if ((new_rows != this->rows()) || (new_cols != this->cols()))
				resize(new_rows, new_cols);
		}

		void Reset(int mat_size){
			Reset(mat_size, mat_size);
		}



		// Import function from ChSparseMatrix format; this function resets the matrix!
		void LoadFromChSparseMatrix(ChSparseMatrix* mat)
		{
			// Create the CSR3 matrix with just the right amount of elements
			int mat_rows = mat->GetRows();
			resize(mat_rows, mat->GetColumns());	// rectangular matrices allowed; matrix still empty
			std::vector<int> reserveSize(mat_rows); // will take the number of elements for each row
			mat->CountNonZeros<std::vector<int>>(reserveSize);
			reserve(reserveSize);

			// Import values from ChSparseMatrix
			ChMelement* elarray = mat->GetElarrayDereferenced();
			ChMelement* el_temp;
			for (int i = 0; i < mat_rows; i++){
				el_temp = &elarray[i];
				while (el_temp){
					if (el_temp->val != 0)
						SetElement(i, el_temp->col, el_temp->val);
					el_temp = el_temp->next;
				};

			};

		} // END LoadFromChSparseMatrix;

		void LoadFromChSparseMatrix(ChSparseMatrix* M, ChSparseMatrix* Cq, ChSparseMatrix* E)
		{
			// Create the CSR3 matrix
			int M_rows = M->GetRows();
			int Cq_rows = Cq->GetRows();
			int mat_rows = M_rows + Cq_rows;
			resize(mat_rows, mat_rows); // only square matrices allowed

			// Preallocate a ChEigenMatrix with the exact number of non-zeros of the ChSparseMatrix
			std::vector<int> reserveSize(mat_rows); // will take the number of elements for each row
			ChMelement* Cq_elarray = Cq->GetElarrayDereferenced();
			ChMelement* el_temp;

			// scan Cq matrix for non-zeros
			for (int i = 0; i < Cq_rows; i++){
				el_temp = &Cq_elarray[i];
				while (el_temp){
					if (el_temp->val != 0){
						reserveSize[M_rows + i]++;
						reserveSize[el_temp->col]++;
					};
					el_temp = el_temp->next;
				}
			}

			M->CountNonZeros<std::vector<int>>(reserveSize);
			E->CountNonZeros<std::vector<int>>(reserveSize, M_rows);

			reserve(reserveSize);

			// Import values from ChSparseMatrix

			for (int i = 0; i < Cq_rows; i++){
				el_temp = &Cq_elarray[i];
				while (el_temp){
					if (el_temp->val != 0){
						SetElement(M_rows + i, el_temp->col, el_temp->val); // sets the Cq
						SetElement(el_temp->col, M_rows + i, el_temp->val); // sets the Cq'
					};
					el_temp = el_temp->next;
				}
			}

			ChMelement* M_elarray = M->GetElarrayDereferenced();
			for (int i = 0; i < M_rows; i++){
				el_temp = &M_elarray[i];
				while (el_temp){
					if (el_temp->val != 0)
						SetElement(i, el_temp->col, el_temp->val);
					el_temp = el_temp->next;
				}
			}

			ChMelement* E_elarray = E->GetElarrayDereferenced();
			for (int i = 0; i < Cq_rows; i++){
				el_temp = &E_elarray[i];
				while (el_temp){
					if (el_temp->val != 0)
						SetElement(i + M_rows, M_rows + el_temp->col, el_temp->val);
					el_temp = el_temp->next;
				}
			}
		} // END LoadFromChSparseMatrix

	}; // END class


}; // END namespace

#endif