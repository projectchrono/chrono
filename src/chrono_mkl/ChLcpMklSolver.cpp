#include "ChLcpMklSolver.h"


#define TESTING_MKL false

namespace chrono
{
	double ChLcpMklSolver::Solve(ChLcpSystemDescriptor& sysd) ///< system description with constraints and variables
	{
		solver_call++;
		
		const int n = sysd.CountActiveVariables() + sysd.CountActiveConstraints();
		ChCSR3Matrix matCSR3(n, n, 1);
		ChMatrixDynamic<double> rhs(n, 1);
		ChMatrixDynamic<double> solution_vector(n, 1);

		// Build matrix and rhs
		//matCSR3.Resize(matCSR3.GetRows(), matCSR3.GetArrayDimension());
		sysd.ConvertToMatrixForm(&matCSR3, &rhs);

		// Solve with Pardiso Sparse Direct Solver
		ChMklEngine pardiso_solver(n, 11, 13);
		matCSR3.Compress(false);
		if (int err_mat = matCSR3.VerifyMatrix() != 0)
			cout << endl << "Matrix error: " << err_mat << endl;
		pardiso_solver.SetProblem(&matCSR3, &rhs, &solution_vector);

		int pardiso_message = pardiso_solver.PardisoSolve(false);
		if (pardiso_message != 0) printf("\nPardiso exited with code: %d\n", pardiso_message);

		// Print statistics
		ChMatrixDynamic<double> residual(n, 1);
		pardiso_solver.GetResidual(&residual);
		double residual_norm = pardiso_solver.GetResidualNorm(&residual);
		GetLog() << "\nCall: " << solver_call << "; Residual norm: " << residual_norm;
		cout << endl;
	
		sysd.FromVectorToUnknowns(solution_vector);

		if (TESTING_MKL){

			int num_arrays;
			printf("MKL memory usage: %.2f MB", static_cast<double> (mkl_mem_stat(&num_arrays)) / 1000000);
			mkl_verbose(1);

			//std::string s;
			//s = "Z_chrono" + std::to_string(solver_call) + ".dat";	PrintMatrix(s, matCSR3);
			//s = "sol_chrono" + std::to_string(solver_call) + ".dat";	PrintMatrix(s, solution_vector);
			//s = "rhs_chrono" + std::to_string(solver_call) + ".dat";	PrintMatrix(s, rhs);
			//s = "res_chrono" + std::to_string(solver_call) + ".dat";	PrintMatrix(s, residual);

			if (solver_call == solver_call_request || (residual_norm_tolerance > 0 && pardiso_solver.GetResidualNorm(&residual) > residual_norm_tolerance)){

				matCSR3.ExportToDatFile("");
				matCSR3.ExportToDatFile("./prec12/");
				matCSR3.ExportToDatFile("./prec24/", 24);
				pardiso_solver.GetIPARMoutput();

				// DOUBLE CHECK
				ChCSR3Matrix matCSR3_bis(n, n, 0.5);
				ChMatrixDynamic<double> rhs_bis(n, 1);
				ChMatrixDynamic<double> solution_vector_bis(n, 1);

				// Build matrix and rhs
				sysd.ConvertToMatrixForm(nullptr, &rhs_bis);
				matCSR3_bis.ImportFromDatFile("");


				// Solve with Pardiso Sparse Direct Solver
				ChMklEngine pardiso_solver_bis(n, 11, 13);

				matCSR3_bis.Compress();
				pardiso_solver_bis.SetProblem(&matCSR3_bis, &rhs_bis, &solution_vector_bis);

				int pardiso_message_bis = pardiso_solver_bis.PardisoSolve(false);
				if (pardiso_message_bis != 0) printf("\nPardiso exited with code: %d", pardiso_message_bis);

				// Print statistics
				ChMatrixDynamic<double> residual_bis(n, 1);
				pardiso_solver_bis.GetResidual(&residual_bis);
				double residual_norm_bis = pardiso_solver_bis.GetResidualNorm(&residual_bis);
				GetLog() << "\nCall: " << solver_call << " DOUBLECHECK; Residual norm: " << residual_norm_bis;
				cout << endl << "Matrix error: " << matCSR3_bis.VerifyMatrix() << endl;

				sysd.FromVectorToUnknowns(solution_vector);

				pardiso_solver_bis.GetIPARMoutput();
				matCSR3_bis.ExportToDatFile("", 24);

				/////////

				

				PrintMatrix("Z_chrono.dat", matCSR3, 12);
				PrintMatrix("sol_chrono.dat", solution_vector, 12);
				PrintMatrix("rhs_chrono.dat", rhs, 12);
				PrintMatrix("res_chrono.dat", residual, 12);

				PrintMatrix("Z_chrono24.dat", matCSR3, 24);
				PrintMatrix("sol_chrono24.dat", solution_vector, 24);
				PrintMatrix("rhs_chrono24.dat", rhs, 24);
				PrintMatrix("res_chrono24.dat", residual, 24);

				//ChSparseMatrix mdM;
				//ChSparseMatrix mdCq;
				//ChSparseMatrix mdE;
				//ChMatrixDynamic<double> mdf;
				//ChMatrixDynamic<double> mdb;
				//ChMatrixDynamic<double> mdfric;
				//sysd.ConvertToMatrixForm(&mdCq, &mdM, &mdE, &mdf, &mdb, &mdfric);

				//PrintMatrix("M.dat", mdM);
				//PrintMatrix("Cq.dat", mdCq);
				//PrintMatrix("E.dat", mdE);
				//PrintMatrix("f.dat", mdf);
				//PrintMatrix("b.dat", mdb);

				std::cout << "\nPrint solver call number: ";
				std::cin >> solver_call_request;
				std::cout << "Residual norm threshold: ";
				std::cin >> residual_norm_tolerance;

			}
		}



		return 0;
	}
}