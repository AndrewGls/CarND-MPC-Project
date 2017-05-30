# Nonlinear Model Predictive Control (NMPC) with actuator latency
Self-Driving Car Engineer Nanodegree Program

---

## The Model

Model Predictive Control (MPC) drives the car around the track with additional latency 100ms between actuator commands. The NMPC allows the car to follow the trajectory along a line (path), provided by the simulator in the World (map) coordinate space, by using predicted(calculated) actuators like steering angle and acceleration (throttle/brake combined).

The vehicle model is implemented is a kinematic bicycle model that ignore tire forces, gravity, and mass. This simplification reduces the accuracy of the models, but it also makes them more tractable. At low and moderate speeds, kinematic models often approximate the actual vehicle dynamics. Kinematic model is implemented using the following equations:

   x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
   y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
   psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
   v_[t+1] = v[t] + a[t] * dt
   cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
   epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
   
where '(x,y)' is position of the vehicle; 'psi' is orientation of the vehicle; 'v' is velocity; 'delta' and 'a' are actuators like steering angle and aceleration (throttle/brake combined); 'Lf' measures the distance between the front of the vehicle and its center of gravity (the larger the vehicle, the slower the turn rate); 'cte' is  cross-track error (the difference between the line and the current vehicle position y in the space of the vehicle); 'epsi' is the orientation error. The vehicle model is implemented in the 'FG_eval' class.

NMPC controller approximates the trajectory with 3rd order polynomial and predicted N states with N-1 actuator changes of the car using a prediction horizon T, which is a duration over which future predictions are made. T is the product of two other variables, N and dt, where N is the number of timesteps in the horizon and dt is how much time elapses between actuations.
The cross track error (CTE) heading error (EPSI) are calculated in the car coordinate space. To do this, I transform waypoints ptsx and ptsy into car coordinates, fit the polynomial and calculate the CTE as the value of the polynomial function at the point x = 0 and the EPSI is -arctan of the first derivative at the point x = 0. After that, MPC controller predicts N state vectors and N-1 actuator vectors for prediction horizon T, using optimization solver Ipopt (Interior Point OPTimizer, pronounced eye-pea-Opt), and returns a new actuator vector, which is the transfer between predicted t+1 and t+2 states.

MPC hyperparameters, used in cost error function:

1. weights to balance cte, epsi and distance to target speed, used during the cost error calculation
2. penalization coefficients to control smoothness steering, smoothness of acceleration, to minimize the use of steering and the use of acceleration.

The hyperparameters were found empirically. 

## Timestep Length and Frequency

The Timestep Length (prediction horizon T) and Frequency were chosen empirically. I use timestep length N = 10, timestep frequency dt = 0.1 sec and 'Numeric max_cpu_time' equal to 0.05 sec, which allow to drive with 58 mph. I had to use trade-off between N, dt and possibility to solve optimization problem as fast as possible with right constraints, used in const error function. I used information about 100 ms latency between sensors and processing too.

## Model Predictive Control with Latency

The Model Predictive Control handles a 100 millisecond latency which simulates latency between sensors and processing. I'm using dt = 100 ms latency and 'Numeric max_cpu_time' equal to 50 ms to handle the actuators.

---
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from [here](https://github.com/coin-or/Ipopt/releases).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).



## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
