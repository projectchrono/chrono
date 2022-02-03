#include "cbtPolarDecomposition.h"
#include "cbtMinMax.h"

namespace
{
cbtScalar abs_column_sum(const cbtMatrix3x3& a, int i)
{
	return cbtFabs(a[0][i]) + cbtFabs(a[1][i]) + cbtFabs(a[2][i]);
}

cbtScalar abs_row_sum(const cbtMatrix3x3& a, int i)
{
	return cbtFabs(a[i][0]) + cbtFabs(a[i][1]) + cbtFabs(a[i][2]);
}

cbtScalar p1_norm(const cbtMatrix3x3& a)
{
	const cbtScalar sum0 = abs_column_sum(a, 0);
	const cbtScalar sum1 = abs_column_sum(a, 1);
	const cbtScalar sum2 = abs_column_sum(a, 2);
	return cbtMax(cbtMax(sum0, sum1), sum2);
}

cbtScalar pinf_norm(const cbtMatrix3x3& a)
{
	const cbtScalar sum0 = abs_row_sum(a, 0);
	const cbtScalar sum1 = abs_row_sum(a, 1);
	const cbtScalar sum2 = abs_row_sum(a, 2);
	return cbtMax(cbtMax(sum0, sum1), sum2);
}
}  // namespace

cbtPolarDecomposition::cbtPolarDecomposition(cbtScalar tolerance, unsigned int maxIterations)
	: m_tolerance(tolerance), m_maxIterations(maxIterations)
{
}

unsigned int cbtPolarDecomposition::decompose(const cbtMatrix3x3& a, cbtMatrix3x3& u, cbtMatrix3x3& h) const
{
	// Use the 'u' and 'h' matrices for intermediate calculations
	u = a;
	h = a.inverse();

	for (unsigned int i = 0; i < m_maxIterations; ++i)
	{
		const cbtScalar h_1 = p1_norm(h);
		const cbtScalar h_inf = pinf_norm(h);
		const cbtScalar u_1 = p1_norm(u);
		const cbtScalar u_inf = pinf_norm(u);

		const cbtScalar h_norm = h_1 * h_inf;
		const cbtScalar u_norm = u_1 * u_inf;

		// The matrix is effectively singular so we cannot invert it
		if (cbtFuzzyZero(h_norm) || cbtFuzzyZero(u_norm))
			break;

		const cbtScalar gamma = cbtPow(h_norm / u_norm, 0.25f);
		const cbtScalar inv_gamma = cbtScalar(1.0) / gamma;

		// Determine the delta to 'u'
		const cbtMatrix3x3 delta = (u * (gamma - cbtScalar(2.0)) + h.transpose() * inv_gamma) * cbtScalar(0.5);

		// Update the matrices
		u += delta;
		h = u.inverse();

		// Check for convergence
		if (p1_norm(delta) <= m_tolerance * u_1)
		{
			h = u.transpose() * a;
			h = (h + h.transpose()) * 0.5;
			return i;
		}
	}

	// The algorithm has failed to converge to the specified tolerance, but we
	// want to make sure that the matrices returned are in the right form.
	h = u.transpose() * a;
	h = (h + h.transpose()) * 0.5;

	return m_maxIterations;
}

unsigned int cbtPolarDecomposition::maxIterations() const
{
	return m_maxIterations;
}

unsigned int polarDecompose(const cbtMatrix3x3& a, cbtMatrix3x3& u, cbtMatrix3x3& h)
{
	static cbtPolarDecomposition polar;
	return polar.decompose(a, u, h);
}
