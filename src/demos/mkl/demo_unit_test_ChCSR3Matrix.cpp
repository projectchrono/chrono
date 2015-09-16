#include "chrono_mkl/ChCSR3matrix.h"
#include "chrono/core/ChSpmatrix.h"
#include "chrono/core/ChMatrixDynamic.h"

#include "chrono_mkl/ChLcpMklSolver.h"
#include <chrono/core/ChMatrixNM.h>

using namespace chrono;

int main(){
	{
		const int m = 5;
		const int n = 3;
		ChCSR3Matrix matCSR3_1;
		matCSR3_1(m, n);

		ChMatrixNM<double, m, n> mat_base;
		for (int m_sel = 0; m_sel < m; m_sel++)
			for (int n_sel = 0; n_sel < n; n_sel++)
				mat_base(m_sel, n_sel) = rand();

		for (int m_sel = 0; m_sel < m; m_sel++)
			for (int n_sel = 0; n_sel < n; n_sel++)
				matCSR3_1.SetElement(m_sel, n_sel, mat_base(m_sel, n_sel));

		for (int m_sel = 0; m_sel < m; m_sel++)
			for (int n_sel = 0; n_sel < n; n_sel++)
				assert(mat_base(m_sel, n_sel) = matCSR3_1.GetElement(m_sel, n_sel));
		
		for (int m_sel = 0; m_sel < m; m_sel++)
			for (int n_sel = 0; n_sel < n; n_sel++)
				assert(mat_base(m_sel, n_sel) = matCSR3_1.Element(m_sel, n_sel));

		for (int m_sel = 0; m_sel < m; m_sel++)
			for (int n_sel = 0; n_sel < n; n_sel++)
				assert(mat_base(m_sel, n_sel) = matCSR3_1(m_sel, n_sel));


	}

}