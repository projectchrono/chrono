#include "ChMklEngine.h"
using namespace chrono;
using namespace std;


ChMklEngine::ChMklEngine(int problem_size, MKL_INT matrix_type=11) : parconf{1,1,0,1}, error(0), phase(-10), n(problem_size){
	if ((matrix_type == 1) || (matrix_type == 2) || (matrix_type == -2) || (matrix_type == 3)
		|| (matrix_type == 4) || (matrix_type == -4) || (matrix_type == 6) || (matrix_type == 11) || (matrix_type == 13)){
		mtype = matrix_type;
	}
	else{
		mtype = 11;
	}
	pardisoinit(pt, &mtype, iparm); // This is the default initialization; could be done also manually
}

ChMklEngine::~ChMklEngine() {
	//pardiso(pt, MKL_INT *maxfct, MKL_INT *mnum, MKL_INT *mtype, -1 , MKL_INT *n, void *a, MKL_INT *ia, MKL_INT *ja, MKL_INT *perm, MKL_INT *nrhs, MKL_INT *iparm, MKL_INT *msglvl, void *b, void *x, MKL_INT *error);
}

bool ChMklEngine::SetMatrix(const ChSparseMatrix* mdCq, const ChSparseMatrix* mdM, const ChSparseMatrix* mdE, const ChMatrixDynamic<double>* mdf, const ChMatrixDynamic<double>* mdb){
	


	int j = 0;
	int row;
	for (row = 0; row < n; row++){
		for (int col = 0; col < n; col++){
			if (A[row*n + col] != 0){
				A_CSR3_values[j] = A[row*n + col];
				A_CSR3_columns[j] = col;
				if (A_CSR3_RowIndex[row] == -1)
					A_CSR3_RowIndex[row] = j;
				j++;
			}
		}
	}
	A_CSR3_RowIndex[row] = j;
}


bool ChMklEngine::SetMatrix(const double* A_CSR_value, const MKL_INT* A_CSR3_columnIndex, const MKL_INT* A_CSR3_rowIndex){
	a = (double*)A_CSR_value;
	ja = (MKL_INT*)A_CSR3_columnIndex;
	ia = (MKL_INT*)A_CSR3_rowIndex;
	return 0;
}

bool ChMklEngine::SetConfig(const MKL_PARDISO_CONFIG* parconf_input) {
	if (parconf_input->maxfct > 0) parconf.maxfct = parconf_input->maxfct;
	else return 1;
	if ((parconf_input->mnum > 0) && (parconf_input->mnum <= parconf.maxfct)) parconf.mnum = parconf_input->mnum;
	else return 1;
	if ((parconf_input->msglvl == 0) || (parconf_input->msglvl == 1))   parconf.msglvl = parconf_input->msglvl;
	else return 1;
	if (parconf_input->nrhs > 0) parconf.nrhs = parconf_input->nrhs;
	else return 1;
	return 0;
};

bool ChMklEngine::GetConfig(MKL_PARDISO_CONFIG* parconf_dest) const{
	parconf_dest->maxfct = parconf.maxfct;
	parconf_dest->mnum = parconf.mnum;
	parconf_dest->msglvl = parconf.msglvl;
	parconf_dest->nrhs = parconf.nrhs;
	return 0;
}

// Solve using the Intel® MKL Pardiso Sparse Direct Solver
double ChMklEngine::PardisoSolve() {
	MKL_INT idum;
	phase = 13;
	PARDISO(pt, &(parconf.maxfct), &(parconf.mnum), &mtype, &phase, &n, a, ia, ja, &idum, &(parconf.nrhs), iparm, &(parconf.msglvl), f, u, &error);
}
