#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
//#include "matplotlibcpp.h"

//namespace plt = matplotlibcpp;

using namespace Utils;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

//**************************************************************************
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.rfind("}]");
	if (found_null != string::npos) {
		return "";
	} else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

//**************************************************************************
// Evaluate a polynomial.
double polyeval(const Eigen::VectorXd& coeffs, double x)
{
	double result = 0.0;
	for (int i = 0; i < coeffs.size(); i++) {
		result += coeffs[i] * pow(x, i);
	}
	return result;
}

//**************************************************************************
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
	Eigen::MatrixXd A(xvals.size(), order + 1);

	for (int i = 0; i < xvals.size(); i++) {
		A(i, 0) = 1.0;
	}

	for (int j = 0; j < xvals.size(); j++) {
		for (int i = 0; i < order; i++) {
			A(j, i + 1) = A(j, i) * xvals(j);
		}
	}

	auto Q = A.householderQr();
	auto result = Q.solve(yvals);
	return result;
}

//**************************************************************************
int main()
{
	uWS::Hub h;

	// MPC is initialized here!
	MPC mpc;

	h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
	{
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		string sdata = string(data).substr(0, length);
		cout << sdata << endl;
		if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
		{
			string s = hasData(sdata);
			if (s != "")
			{
				auto j = json::parse(s);
				string event = j[0].get<string>();
				if (event == "telemetry")
				{
					// j[1] is the data JSON object, specified in the Map space.
					const vector<double> ms_ptsx = j[1]["ptsx"];  // Waypoints (x, y) in
					const vector<double> ms_ptsy = j[1]["ptsy"];  //  the Map space.
					const double ms_px  = j[1]["x"];  // position x of the car
					const double ms_py  = j[1]["y"];  // position y of the car
					const double ms_psi = j[1]["psi"]; // orientation of the car
					const double v   = j[1]["speed"];  // velocity of the car

					//
					// Data are transformed and processed in the Car space.
					//
					// Transform waypoints(x, y) to the Car space
					vector<double> ptsx;
					vector<double> ptsy;
					MapToCarSpace(ptsx, ptsy, ms_ptsx, ms_ptsy, ms_psi, ms_px, ms_py);


					//  `polyfit` to fit a third order polynomial to the (x, y) coordinates.
					// Fits a third order polynomial y(x) = f(x) = a0 + a1*x + a2*x^2 + a3*x^3 using waypoints (x, y) to
					// interpolate the reference trajectory as function f(x,y) in the space of the car.
					Eigen::VectorXd coeffs;
					{
						Eigen::VectorXd x (ptsx.size());
						Eigen::VectorXd y (ptsy.size());
						for (size_t i = 0; i < ptsx.size(); i++) {
							x(i) = ptsx[i];
							y(i) = ptsy[i];
						}
						coeffs = polyfit(x, y, 3);
					}

					// The cross track error is calculated by evaluating at polynomial at x, f(x) and subtracting y:
					// cte = polyeval(coeffs, px) - py,
					//	where px = 0 and py = 0 because this is the center of Car space.
					// Use polynomial p(x) = a0 + a1 * x + a2 * x^2 + a3 * x^3.
					double cte = polyeval(coeffs, 0);
					// Due to the sign starting at 0, the orientation error is -f'(x).
					// Derivative of polynomial f'(x) = coeffs[1] + 2*coeffs[2] * x  + 3*coeffs[3] * x^2
					//  at point (0,0) is coeffs[1].
					double epsi = -atan(coeffs[1]);

					/*
					* Calculates steeering angle and throttle using MPC.
					*
					* Both are in between [-1, 1].
					*
					*/
					double steer_value;
					double throttle_value;

					Eigen::VectorXd state(6);
					state << 0., 0., 0., v, cte, epsi; // { px, py, psi, v, cye, epsi }

					// Returns next state: x, y, psi, v, cte, epsi, delta, a
					vector<double> next_state = mpc.Solve(state, coeffs);
					// double mpc_x   = next_state[0];
					// double mpc_y   = next_state[1];
					// double mpc_psi = next_state[2];
					// double mpc_v   = next_state[3];
					// double mpc_cte = next_state[4];
					// double mpc_epsi = next_state[5];
					///double mpc_delta = next_state[6];
					///double mpc_a     = next_state[7];

					json msgJson;
					msgJson["steering_angle"] = mpc.steeringValue();
					msgJson["throttle"] = mpc.throttleValue();

					//Display the MPC predicted trajectory 
					//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
					// the points in the simulator are connected by a Green line
					msgJson["mpc_x"] = mpc.pred_path_x_;
					msgJson["mpc_y"] = mpc.pred_path_y_;

					//Display the waypoints/reference line
					//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
					// the points in the simulator are connected by a Yellow line
					msgJson["next_x"] = ptsx;
					msgJson["next_y"] = ptsy;


					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					std::cout << msg << std::endl;

					// Latency
					// The purpose is to mimic real driving conditions where
					// the car does actuate the commands instantly.
					//
					// Feel free to play around with this value but should be to drive
					// around the track with 100ms latency.
					//
					// NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
					// SUBMITTING.
					this_thread::sleep_for(chrono::milliseconds(100));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else
			{
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
	{
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		} else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws,
						   int code, char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	constexpr int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
