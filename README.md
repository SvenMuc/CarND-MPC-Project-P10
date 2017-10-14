# Project 10: CarND-Controls-MPC
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[//]: # (Image References)
[image_mpc_simulator]: ./images/mpc_simulator.png

## Introduction
The goal of this project is to implement a C++ Model Predictive Controller (MPC) to drive the vehicle around the track in Udacity's simulator. Additionally, a 100 ms latency between actuation commands on top of the connection latency have to be compensated by the controller.

## Simulator
The simulator returns x and y waypoints (`ptsx`and `ptsy`), the vehicle orientation `psi` in radians, the vhicles' global position `x`and `y` in meter, the steering angle `steering_angle` in radians, the throttle position `throttle` and the vehicle's speed `speed`in mph.

The yellow line represents the waypoints returned by the simulator and the green lane the polynomial fitted reference path of the MPC.

![MPC Simulator][image_mpc_simulator]

## MPC - Model Predictive Controller

### State Vector
The state vector is defined as followed [px, py, psi, v, cte, epsi].

| State | Description                                                                                          | Unit |
|:------|:-----------------------------------------------------------------------------------------------------|:----:|
| px    | Vehicle position x                                                                                   |  m   |
| py    | Vehicle position y                                                                                   |  m   |
| psi   | Vehicle orientation                                                                                  | rad  |
| v     | Velocity                                                                                             | mph  |
| cte   | The Cross Track Error is the delta between the predicted distance of the vehicle and the trajectory. |  m   |
| epsi  | Psi error is the delta between the predicted vehicle orientation and the trajectory orientation.     | rad  |

### Actuator Vector
The simulator provides two actuator attributes. The steering angle `psi` and the throttle and break in a single attribute `a`. A negative value decelerates and a positive value accelerates the vehicle.

| Actuator | Description                                            |  Range   |  Unit  |
|:---------|:-------------------------------------------------------|:--------:|:------:|
| psi      | Steering angle (Attention: the interface uses radians) | [-25,25] | degree |
| a        | throttle/brake                                         |  [-1,1]  |   %    |

### Kinematic Model
The kinematic model predicts the state vector on the next timestep considering the actual state and actuator values as followed. The model does not consider dynamics like tire forces, slip angle or slip ratio because Udacity's simulator does not support it.

```cpp
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
```

The errors are calculated as followed:
```cpp
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

### Cost Function
The cost function is defined by the sum of the following attributes over all timesteps `N`:

- Cross Track Error `cte`
- Orientation error `epsi`
- Magnitude of steering angle `delta`and throttle `a`. This limits the usage of the actuators.
- Change rate of steering angle `delta_change` and throttle `a_change`. This limits the change rate of the actuators between two timesteps.

In order to weight the individual costs I applied the following weights. The weight factors have been tuned manually in several test runs. The higher weights for the cross track and orientation errors keeps the vehicle in the center of the lane. A lower weight for the velocity enables MPC to slow down in e.g. curves.

```cpp
// optimization weights
const double cte_cost_weight = 3000;
const double epsi_cost_weight = 2800;
const double v_cost_weight = 1;
const double delta_cost_weight = 100;
const double a_cost_weight = 20;
const double delta_change_cost_weight = 100;
const double a_change_cost_weight = 10;
```

### Hyperparameters
To tune the "look ahead time" of the MPC, you can tune the number of timesteps `N` and the length of each timestep `dt`. The parameters have to be chosen wisely. Small `dt` values reduce the distance between two points in the reference polynomial but increase the processing time at the same time which might lead to an oscillating controller because the steering and throttle commands are send too late. The number of timesteps `N` defines the number of total points for the polynomial fit. A small number leads to a shorter line. Thus the controller reduces e.g. the speed before a curve too late.

Finally, I decided to use `N = 10`and `dt = 0.1` which leads to a total look ahead time of 1 second. This is a good compromise between processing time and lookahead distance for the chosen target speed of 100 mph.

### Latency Compensation
To compensate the latency of 100 ms the state vector [px, py, pis v] and the errors `cte`and `epsi`are predicted by a kinematic model 100 ms ahead (see code below) before the MPC solve function is called.

```cpp
double pred_px = 0.0 + v * dt; // Since psi is zero, cos(0) = 1, can leave out
const double pred_py = 0.0; // Since sin(0) = 0, y stays as 0 (y + v * 0 * dt)
double pred_psi = 0.0 + v * -delta / Lf * dt;
double pred_v = v + a * dt;
double pred_cte = cte + v * sin(epsi) * dt;
double pred_epsi = epsi + v * -delta / Lf * dt;
```

## Results
The MPC has been developed on a powerful Apple Mac Book Pro 2017 (3,1 GHz Intel Core i7, 16 GB RAM, Radeon Pro 560 with 4 GB). The MPC performance is demonstrated in the YouTube video.

YouTube Video

[![Link](https://img.youtube.com/vi/s-m8sX0rvnQ/0.jpg)](https://youtu.be/s-m8sX0rvnQ)


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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
