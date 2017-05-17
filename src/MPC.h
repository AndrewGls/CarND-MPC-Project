#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cassert>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC
{
public:
	MPC();

	virtual ~MPC();

	// Solve the model given an initial state and polynomial coefficients.
	// Return the first actuatotions.
	vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};


// Returns derivative of 3rd order polynomial p'(x) = coeffs[1] + 2*coeffs[2] * x  + 3*coeffs[3] * x^2
template <class VEC, class T>
T poly3_derivative(const VEC& coeffs, T x) {
	return coeffs[1] + 2 * coeffs[2] * x + 3 * coeffs[3] * x * x;
}


// Transforms points from Car space to Map space
template <class Vec>
void CarToMapSpace(Vec& m_x, Vec& m_y, const Vec& c_x, const Vec& c_y, double psi, double tx, double ty)
{
	// Counter-clockwise transformation (from car to map space)
	// |x'| |cos(a) -sin(a) tx| |x|
	// |y'|=|sin(a)  cos(a) ty|*|y|
	// |1 | | 0        0     0| |1|

	assert(c_x.size() == c_y.size());

	m_x.resize(c_x.size());
	m_y.resize(c_y.size());

	const auto cosa = cos(psi);
	const auto sina = sin(psi);

	for (int i = 0; i < c_x.size(); i++)
	{
		c_x[i] = m_x[i] * cosa - m_y[i] * sina + tx;
		c_y[i] = m_x[i] * sina + m_y[i] * cosa + ty;
	}
}


// Transforms points from Map space to Car space
template <class Vec>
void MapToCarSpace(Vec& c_x, Vec& c_y, const Vec& m_x, const Vec& m_y, double psi, double tx, double ty)
{
	// Inverse of counter-clockwise transformation (from car to map space)
	// x += tx
	// y += ty
	// |x'| | cos(a) sin(a)| |x|
	// |y'|=|-sin(a) cos(a)|*|y|

	assert(m_x.size() == m_y.size());

	c_x.resize(m_x.size());
	c_y.resize(m_y.size());

	const auto cosa = cos(psi);
	const auto sina = sin(psi);

	for (int i = 0; i < c_x.size(); i++)
	{
		const auto x = m_x[i] + tx;
		const auto y = m_y[i] + ty;
		c_x[i] =  x * cosa + y * sina;
		c_y[i] = -x * sina + y * cosa;
	}
}


#endif /* MPC_H */
