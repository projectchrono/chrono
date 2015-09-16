#include "ChLcpMklSolver.h"


#define TESTING_MKL false

namespace chrono
{
	template <class matrix_t>	void PrintMatrix(std::string filename, matrix_t& mat, int precision = 12); // forwarding

	// Functions for testing purpose
	static int solver_call_request = 0;
	static double residual_norm_tolerance = 1e-8;

	double ChLcpMklSolver::Solve(ChLcpSystemDescriptor& sysd) ///< system description with constraints and variables
	{
		if (solver_call == 0 || !size_lock )
		{
			n = sysd.CountActiveVariables() + sysd.CountActiveConstraints();
			matCSR3.Reset(n, n);
			rhs.Reset(n, 1);
			sol.Reset(n, 1);
			res.Reset(n, 1);
			
		}


		// Build matrix and rhs
		sysd.ConvertToMatrixForm(&matCSR3, &rhs);
		if (!sparsity_pattern_lock || solver_call == 0)
			matCSR3.Compress(false);
		if (sparsity_pattern_lock)
			matCSR3.SetRowIndexLock(true);

		// Solve with Pardiso Sparse Direct Solver
		mkl_engine.SetProblem(matCSR3, rhs, sol);
		int pardiso_message = mkl_engine.PardisoCall(13, 0);
		solver_call++;
		if (pardiso_message != 0)
		{
			printf("\nPardiso exited with code: %d", pardiso_message);
			printf("\nMatrix verification returned: %d\n", matCSR3.VerifyMatrix());
		}

		// Print statistics
		mkl_engine.GetResidual(res);
		res_norm = mkl_engine.GetResidualNorm(res);
		cout << "\nCall: " << solver_call << "; Residual norm: " << res_norm;

		sysd.FromVectorToUnknowns(sol);

		if (TESTING_MKL){

			int num_arrays;
			printf("MKL memory usage: %.2f MB", static_cast<double> (mkl_mem_stat(&num_arrays)) / 1000000);
			mkl_verbose(1);

			//std::string s;
			//s = "Z_chrono" + std::to_string(solver_call) + ".dat";	PrintMatrix(s, matCSR3);
			//s = "sol_chrono" + std::to_string(solver_call) + ".dat";	PrintMatrix(s, solution_vector);
			//s = "rhs_chrono" + std::to_string(solver_call) + ".dat";	PrintMatrix(s, rhs);
			//s = "res_chrono" + std::to_string(solver_call) + ".dat";	PrintMatrix(s, residual);

			if (solver_call == solver_call_request || (residual_norm_tolerance > 0 && mkl_engine.GetResidualNorm(res) > residual_norm_tolerance)){

				matCSR3.ExportToDatFile("");
				matCSR3.ExportToDatFile("./prec12/");
				matCSR3.ExportToDatFile("./prec24/", 24);
				mkl_engine.PrintIparmOutput();

				// DOUBLE CHECK
				ChCSR3Matrix matCSR3_bis(n, n);
				ChMatrixDynamic<double> rhs_bis(n, 1);
				ChMatrixDynamic<double> solution_vector_bis(n, 1);

				// Build matrix and rhs
				sysd.ConvertToMatrixForm(nullptr, &rhs_bis);
				matCSR3_bis.ImportFromDatFile("");


				// Solve with Pardiso Sparse Direct Solver
				ChMklEngine mkl_engine_bis(n, 11);

				matCSR3_bis.Compress();
				mkl_engine_bis.SetProblem(matCSR3_bis, rhs_bis, solution_vector_bis);

				int pardiso_message_bis = mkl_engine_bis.PardisoCall(13,false);
				if (pardiso_message_bis != 0) printf("\nPardiso exited with code: %d", pardiso_message_bis);

				// Print statistics
				ChMatrixDynamic<double> residual_bis(n, 1);
				mkl_engine_bis.GetResidual(residual_bis);
				double residual_norm_bis = mkl_engine_bis.GetResidualNorm(residual_bis);
				cout << "\nCall: " << solver_call << " DOUBLECHECK; Residual norm: " << residual_norm_bis;
				cout << endl << "Matrix error: " << matCSR3_bis.VerifyMatrix() << endl;

				sysd.FromVectorToUnknowns(solution_vector_bis);

				mkl_engine_bis.PrintIparmOutput();
				matCSR3_bis.ExportToDatFile("", 24);

				/////////

				PrintMatrix("Z_chrono.dat", matCSR3, 12);
				PrintMatrix("sol_chrono.dat", sol, 12);
				PrintMatrix("rhs_chrono.dat", rhs, 12);
				PrintMatrix("res_chrono.dat", res, 12);

				PrintMatrix("Z_chrono24.dat", matCSR3, 24);
				PrintMatrix("sol_chrono24.dat", sol, 24);
				PrintMatrix("rhs_chrono24.dat", rhs, 24);
				PrintMatrix("res_chrono24.dat", res, 24);

				ChSparseMatrix mdM;
				ChSparseMatrix mdCq;
				ChSparseMatrix mdE;
				ChMatrixDynamic<double> mdf;
				ChMatrixDynamic<double> mdb;
				ChMatrixDynamic<double> mdfric;
				sysd.ConvertToMatrixForm(&mdCq, &mdM, &mdE, &mdf, &mdb, &mdfric);

				PrintMatrix("M.dat", mdM);
				PrintMatrix("Cq.dat", mdCq);
				PrintMatrix("E.dat", mdE);
				PrintMatrix("f.dat", mdf);
				PrintMatrix("b.dat", mdb);

				std::cout << "\nPrint solver call number: ";
				std::cin >> solver_call_request;
				std::cout << "Residual norm threshold: ";
				std::cin >> residual_norm_tolerance;

			}
		}



		return 0;
	}


	template <class matrix_t>
	void PrintMatrix(std::string filename, matrix_t& mat, int precision)
	{
		std::ofstream myfile;
		myfile.open(filename);
		myfile << std::scientific << std::setprecision(precision);
		for (int ii = 0; ii < mat.GetRows(); ii++){
			for (int jj = 0; jj < mat.GetColumns(); jj++)
				myfile << mat.GetElement(ii, jj) << "\t";
			myfile << std::endl;
		}
		myfile.close();
	}

} // namespace chrono