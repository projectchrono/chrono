#include "chrono_mkl/ChCSR3matrix.h"
#include "chrono/core/ChSpmatrix.h"
#include "chrono/core/ChMatrixDynamic.h"

using namespace chrono;
//
//#include <vector>
//#include <stdlib.h>     /* srand, rand */
//#include <stdio.h>
//
//typedef Eigen::SparseMatrix<double> MatriceSparsa;
//typedef Eigen::Triplet<double> Tripletta;
//
//
//void setCoeff(std::vector<Tripletta>* coeff_list){
//	for (std::vector<Tripletta>::iterator iterator_loop = coeff_list->begin(); iterator_loop != coeff_list->end(); iterator_loop++){
//		*iterator_loop = Tripletta(rand() % 10, rand() % 10, (double)(rand() % 1000) / 100);
//	}
//
//}
//
//void printCoeff(std::vector<Tripletta>* coeff_list){
//	for (std::vector<Tripletta>::iterator iterator_loop = coeff_list->begin(); iterator_loop != coeff_list->end(); ++iterator_loop){
//		printf("%d %d %f\n", iterator_loop->row(), iterator_loop->col(), iterator_loop->value());
//	}
//
//}
template<class ChMatrixIN>
void PrintMatrix(ChMatrixIN* matrice){
	for (int i = 0; i < matrice->GetRows(); i++){
		for (int j = 0; j < matrice->GetColumns(); j++){
			printf("%.1f ", matrice->GetElement(i,j));
		}
		printf("\n");
	}
}

int main(){


	int m = 3;
	int n = 5; // TODO: check for row!=cols
	ChEigenMatrixNEW mat(m, n, 1);

	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			mat.SetElement(i, j, 2.5);
		}
	}

	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			printf("%f ", mat.GetElement(i, j));
		}
		printf("\n");
	}
	
	getchar();
}