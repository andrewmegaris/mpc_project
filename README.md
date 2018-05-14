# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Ruberic Points:


### The Model:

The model used is a Kinematic model.  This model assume basic inputs only.  No friction between the tires and the road.

* x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
* y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
* psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
* v[t] = v[t-1] + a[t-1] * dt
* cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
* epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt

  
- x, y : Position.
- psi : Heading direction.
- v : Velocity.
- cte : Cross-track error.
- epsi : Orientation error. 
- a : Acceleration (throttle).
- delta : Steering angle.

### Timestep Length and Elapsed Duration (N & dt)

The number of points: N and the time interval: dt are used to define the prediction. The number of points interacts with the performance.  Too few points will not give you the performance required to navigate. Too many points will greatly slow down the controller, and would not be able to recover from some actuations. After trying a range of N from 5, 8, 10, 15, and 20 and dt from 50, 75, 100, 200, and 500 milliseconds, I decided to leave them fixed at 10 and 100 milliseconds respectively.  These values seemed to perform and allow the easiest adjusting of other parameters.

### Polynomial Fitting and MPC Preprocessing

The transofmration of the waypoints into the car's cooridinate frame is done in the main.cpp file starting at line 102.  A third degree polynomial is fitted to the waypoints after the translation. These coefficients calculate the cte and epsi. The Solver uses these to create a reference trajectory.

### Model Predictive Control with Latency
 
To handle the actuation latency the state values are calculated with the delay interval included with the model.  This replaces the initial value.


