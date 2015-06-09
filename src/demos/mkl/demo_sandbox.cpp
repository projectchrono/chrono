#include <core/ChCSR3matrix.h>
#include <core/ChSpmatrix.h>
#include <core/ChMatrixDynamic.h>

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
void printMatrix(ChMatrixIN* matrice){
	for (int i = 0; i < matrice->GetRows(); i++){
		for (int j = 0; j < matrice->GetColumns(); j++){
			printf("%.1f ", matrice->GetElement(i,j));
		}
		printf("\n");
	}
}

int main(){

	chrono::ChMatrixDynamic<double> matricedensa(5, 5);
	matricedensa.FillRandom(13, 4);
	printMatrix(&matricedensa);


	ChEigenMatrix matricesparsa(10, 10);

	/*matricesparsa.PasteMatrix(&matricedensa, 0, 0);
	matricesparsa.PasteMatrix(&matricedensa, 5, 5);

	matricesparsa.PasteMatrix(&matricedensa, 0, 0);
	matricesparsa.PasteSumMatrix(&matricedensa, 5, 5);

	*/

	matricesparsa.PasteSumClippedMatrix(&matricedensa, 2, 2, 2, 2, 3, 3);

	printMatrix(&matricesparsa);
	double* a =  matricesparsa.valuePtr();

	if (!NULL) 
		printMatrix(&matricesparsa);

	const int n = 10 ;
	ChMatrixDynamic<double> vettoredenso(n,1);
	double* puntatore = vettoredenso.GetAddress();

	for (int i = 0; i < n; i++){
		printf("\n%.1f", puntatore+i);
	};


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