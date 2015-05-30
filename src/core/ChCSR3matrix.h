#include <Eigen\Sparse>
#include <vector>
#include <core\ChSpmatrix.h>

namespace chrono{
	class ChEigenMatrix : public Eigen::SparseMatrix < double, 1 > {
	public:
		ChEigenMatrix() : Eigen::SparseMatrix<double,1>() {};
		ChEigenMatrix(Index rows, Index cols) : Eigen::SparseMatrix<double,1>(rows, cols){};
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


		//inline void PasteMatrix(ChMatrix<>* matra, int insrow, int inscol){
		//	int maxrows = matra->GetRows();
		//	int maxcols = matra->GetColumns();
		//	int i, j;
		//	
		//	// can't use triplets because they expect a compressed matrix with
		//	// non existing entries

		//	for (i = 0; i < maxrows; i++){
		//		for (j = 0; j < maxcols; j++){
		//			this->SetElement(insrow + i, inscol + j, (*matra)(i, j));
		//		};
		//	};
		//};

		template <bool overwrite = 1, class ChMatrixIN>
		void PasteMatrix(ChMatrixIN* matra, int insrow, int inscol){
			int maxrows = matra->GetRows();
			int maxcols = matra->GetColumns();
			int i, j;

			// can't use triplets because they expect a compressed matrix with
			// non existing entries

			for (i = 0; i < maxrows; i++){
				for (j = 0; j < maxcols; j++){
					this->SetElement<overwrite>(insrow + i, inscol + j, (*matra)(i, j));
				};
			};
		};

		template <class ChMatrixIN>
		inline void PasteSumMatrix(ChMatrixIN* matra, int insrow, int inscol){ PasteMatrix<0>(matra, insrow, inscol); };

		template <bool overwrite = 1, class ChMatrixIN>
		void PasteTranspMatrix(ChMatrixIN* matra, int insrow, int inscol){
			int maxrows = matra->GetRows();
			int maxcols = matra->GetColumns();
			int i, j;

			// can't use triplets because they expect a compressed matrix with
			// non existing entries

			for (i = 0; i < maxrows; i++){
				for (j = 0; j < maxcols; j++){
					this->SetElement<overwrite>(insrow + i, inscol + j, (*matra)(j,i));
				};
			};
		};

		template <class ChMatrixIN>
		inline void PasteSumTranspMatrix(ChMatrixIN* matra, int insrow, int inscol){ PasteTranspMatrix<0>(matra, insrow, inscol); };


		inline double GetElement(const int row, const int col) const{
			return this->coeff(row, col);
		};

	};
};