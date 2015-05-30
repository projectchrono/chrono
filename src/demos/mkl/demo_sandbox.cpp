
#include "demo_sandbox.h"

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

void printMatrix(ChEigenMatrix* matrice){
	for (int i = 0; i < matrice->rows(); i++){
		for (int j = 0; j < matrice->cols(); j++){
			printf("%.1f ", matrice->GetElement(i,j));
		}
		printf("\n");
	}
}

int main(){

	chrono::ChMatrixDynamic<> matricedensa(5, 5);
	matricedensa.FillElem(3.7);


	ChEigenMatrix matricesparsa(10, 10);

	matricesparsa.PasteMatrix(&matricedensa, 0, 0);
	matricesparsa.PasteMatrix(&matricedensa, 5, 5);

	matricesparsa.PasteMatrix(&matricedensa, 0, 0);
	matricesparsa.PasteSumMatrix(&matricedensa, 5, 5);

	printMatrix(&matricesparsa);




	//std::vector<Tripletta> coefficienti(10);

	//MatriceSparsa mat_sparsa(10, 10);
	//Tripletta tripletta_prova(0, 0, 3);
	//setCoeff(&coefficienti);
	//printCoeff(&coefficienti);

	//double& a = mat_sparsa.coeffRef(1, 1);
	//coefficienti[1] = Tripletta(1, 2, 30);

	//mat_sparsa.setFromTriplets(coefficienti.begin(), coefficienti.end());


	//for (int i = 0; i < 10; i++){
	//	for (int j = 0; j < 10; j++){
	//		printf(" %.1f ", mat_sparsa.coeff(i, j));
	//	}
	//	printf("\n");
	//}
	//getchar();
}