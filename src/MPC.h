#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cassert>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC
{
	static constexpr double max_delta = 0.436332;
	static const int num_states_in_latency;

public:
	MPC();
	~MPC() {}

	// Solve the model given an initial state and polynomial coefficients.
	// Return the first actuatotions.
	vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

	double steeringValue() const { return -steering_delta_ / max_delta; }
	double throttleValue() const { return a_; }

	std::vector<double> pred_path_x_;
	std::vector<double> pred_path_y_;

private:
	double steering_delta_;
	double a_;
};


namespace Utils
{

	// Returns derivative of 3rd order polynomial p'(x) = coeffs[1] + 2*coeffs[2] * x  + 3*coeffs[3] * x^2
	template <class VEC, class T>
	T poly3_derivative(const VEC& coeffs, T x) {
		return coeffs[1] + 2 * coeffs[2] * x + 3 * coeffs[3] * x * x;
	}


	// Transforms points from Map space to Car space
	template <class Vec>
	void MapToCarSpace(Vec& c_x, Vec& c_y, const Vec& m_x, const Vec& m_y, double psi, double tx, double ty)
	{
		// Inverse of counter-clockwise transformation from car to map space.
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
			const auto x = m_x[i] - tx;
			const auto y = m_y[i] - ty;
			c_x[i] = x * cosa + y * sina;
			c_y[i] = -x * sina + y * cosa;
		}
	}

} // end of Utils namespace

#endif /* MPC_H */
