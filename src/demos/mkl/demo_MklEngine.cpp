#include "chrono_mkl/ChCSR3matrix.h"
#include "chrono/core/ChSpmatrix.h"
#include "chrono/core/ChMatrixDynamic.h"

#include "chrono_mkl/ChLcpMklSolver.h"


#define ALIGNMENTTOCHECK 32
#include <chrono/core/ChMatrixNM.h>

using namespace chrono;

//#include <vector>
//#include <stdlib.h>     /* srand, rand */
//#include <stdio.h>

//typedef Eigen::SparseMatrix<double> MatriceSparsa;
//typedef Eigen::Triplet<double> Tripletta;

//void setCoeff(std::vector<Tripletta>* coeff_list){
//	for (std::vector<Tripletta>::iterator iterator_loop = coeff_list->begin(); iterator_loop != coeff_list->end(); iterator_loop++){
//		*iterator_loop = Tripletta(rand() % 10, rand() % 10, (double)(rand() % 1000) / 100);
//	}
//}

//void printCoeff(std::vector<Tripletta>* coeff_list){
//	for (std::vector<Tripletta>::iterator iterator_loop = coeff_list->begin(); iterator_loop != coeff_list->end(); ++iterator_loop){
//		printf("%d %d %f\n", iterator_loop->row(), iterator_loop->col(), iterator_loop->value());
//	}
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

void LoadFromMatrix(ChMatrix<>& output_mat, std::string filename)
{
	std::ifstream my_file;
	my_file.open(filename);

	double temp;
	int row_sel = -1;
	for (row_sel = 0; row_sel < output_mat.GetRows(); row_sel++)
	{
		my_file >> temp;
		output_mat.SetElement(row_sel,0,temp);
	}
	my_file.close();
}


int main(){

	cout << "//////////// CSR3 Matrix: basic functions testing //////////////" << endl;
	int m = 3;
	int n = 5;
	ChCSR3Matrix matCSR3_1(m, n, 0.1);
	int prova = 0;
	MKL_INT64 mem = mkl_mem_stat(&prova);

	for (int i = 0; i < m; i++)
	{
		for (int j = i; j < n; j++)
		{
			matCSR3_1.SetElement(i, j, 2.5);
		}
	}

	mem = mkl_mem_stat(&prova);

	matCSR3_1.SetElement(0, 1, 0.7);

	double elem = matCSR3_1.Element(2, 0);
	matCSR3_1.Element(2, 0) = matCSR3_1.Element(0,1);
	//double* elem = &(matCSR3_1.Element(2, 0));

	matCSR3_1(2,2 ) = 5;

	matCSR3_1.Compress(true);
	
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			printf("%f ", matCSR3_1.GetElement(i, j));
		}
		printf("\n");
	}

	//////////////////////////////
	cout << endl << "//////////// CSR3 Matrix: Resize and Reset testing //////////////" << endl;
	m = m + 3;
	n = n + 1;
	matCSR3_1.Resize(m, n, 25);

	matCSR3_1(5, 1) = 5;

	matCSR3_1.Compress();

	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			printf("%f ", matCSR3_1.GetElement(i, j));
		}
		printf("\n");
	}


	/////////////////
	cout << endl << "//////////// CSR3 Matrix: Sparsity pattern testing //////////////" << endl;
	matCSR3_1.SetRowIndexLock(true);
	matCSR3_1.SetColIndexLock(true);
	matCSR3_1.Reset(matCSR3_1.GetRows(), matCSR3_1.GetColumns());

	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			printf("%f ", matCSR3_1.GetElement(i, j));
		}
		printf("\n");
	}
	

	///////////////////////////////////
	cout << endl << "//////////// Comparison MKL Pardiso: different precisions //////////////";
	n = 540;
	ChCSR3Matrix matCSR3_prec12(n, n);
	ChCSR3Matrix matCSR3_prec24(n, n);
	ChMatrixDynamic<double> rhs(n, 1);
	ChMatrixDynamic<double> solution_vector_prec12(n, 1);
	ChMatrixDynamic<double> solution_vector_prec24(n, 1);

	matCSR3_prec12.ImportFromDatFile("./prec12/");
	matCSR3_prec24.ImportFromDatFile("./prec24/");
	LoadFromMatrix(rhs, "rhs_chrono24.dat");

	// Solve with Pardiso Sparse Direct Solver
	ChMklEngine pardiso_solver_prec12(n, 11);
	ChMklEngine pardiso_solver_prec24(n, 11);
	matCSR3_prec12.Compress();
	matCSR3_prec24.Compress();
	pardiso_solver_prec12.SetProblem(matCSR3_prec12, rhs, solution_vector_prec12);
	pardiso_solver_prec24.SetProblem(matCSR3_prec24, rhs, solution_vector_prec24);

	int pardiso_message = pardiso_solver_prec12.PardisoCall(13,0);
	printf("\nPardiso prec12 exited with code: %d", pardiso_message);
	pardiso_message = pardiso_solver_prec24.PardisoCall(13, 0);
	printf("\nPardiso prec24 exited with code: %d", pardiso_message);

	// Print statistics
	ChMatrixDynamic<double> residual_prec12(n, 1);
	ChMatrixDynamic<double> residual_prec24(n, 1);
	pardiso_solver_prec12.GetResidual(residual_prec12);
	pardiso_solver_prec24.GetResidual(residual_prec24);
	double residual_norm_prec12 = pardiso_solver_prec12.GetResidualNorm(residual_prec12);
	double residual_norm_prec24 = pardiso_solver_prec24.GetResidualNorm(residual_prec24);
	GetLog() << "\nResidual norm prec12: " << residual_norm_prec12;
	GetLog() << "\nResidual norm prec24: " << residual_norm_prec24;

	getchar();

}