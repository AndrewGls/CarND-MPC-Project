# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

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

## The Model

Model Predictive Control (MPC) drives the car around the track with additional latency 100ms between commands. The MPC allows the car to follow the trajectory along a line (path), provided by simulator in the World (map) coordinate space, by using predicted(calculated) actuators like steering angle and acceleration (throttle/brake combined). MPC controller approximates the trajectory with 3rd order polynomia and predictes N states with N-1 actuators changes of the car using a prediction horizon T, which is a duration over which future predictions are made. T is the product of two other variables, N and dt, where N is the number of timesteps in the horizon and dt is how much time elapses between actuations.
The cross track error (CTE) heading error (EPSI) are calculated in the car coordinate space. To do this, I transform waypoints ptsx and ptsy into car coordinates, fit the polynomial and calculate the CTE as the value of the polynomial function at the point x = 0 and the EPSI is -arctan of the first derivative at the point x = 0. After that, MPC controller predicts N state vectors and N-1 actuator vectors for prediction horizon T, using optimization solver Ipopt (Interior Point OPTimizer, pronounced eye-pea-Opt), and returns a new actuator vector, which is the transfer between predicted t+1 and t+2 states.

MPC hyperparameters, used in cost error function:

1. weights to balance cte, epsi and distance to target speed, used during the cost error calculation
2. penalization coefficients to controll smoothness steering, smoothness of acceleration, to minimize the use of steering and the use of acceleration.

The hyperparameters were found imperically. 

## Timestep Length and Frequency

The Timestep Length (prediction horizon T) and Frequency were chosen impericall. I use timestep length N = 10, timestep frequency dt = 0.1 sec and 'Numeric max_cpu_time' equal to 0.05 sec, which allow to drive with 58 mph. I had to use trade-off between N, dt and  prossibility to solve oprimization problem as fast as possible with right constrains, used in const error function. I used information about 100 ms latency between sensors and processing too.

## Model Predictive Control with Latency

The Model Predictive Control handles a 100 millisecond latency which simulate latency between sensors and processing. I'm using dt = 100 ms latency and 'Numeric max_cpu_time' equal to 50 ms to handle the actuations.

