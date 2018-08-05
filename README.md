# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

**This file is updated based on the assignment submission.**

[//]: # (Image References)
[image1]: ./setup.png
[image2]: ./control.png

# The Model
The model predictive control (MPC) implementation in this assignment is based on the 
lecture notes and the MPC quizzes solution. The model is a simple kinematic controller
that takes the intended path computed using a 3rd degree polynomial fitting (reference
waypoints), state variables *{current x coordinate (x), current y coordinate(y), 
vehicle orientation (psi), velocity (v), cross-talk error(cte), orientation error (epsi)}* 
and actuator constraints *{steering_angle, thtrottle}* and computes the actuator values for 
next specified *N* number of future intervals, while optimizing the error between intented 
waypoints and model predicted points.

The majority of the work involved in fine-tuning
- the resulting horizon (*T*)
- time interval between each prediction (*dt*)
- number of prediction steps (*N = T/dt*) 

AND

the error parameters for the optimizer consideration.

The model also considers the latency between computation and the actuator effect on the vehicle.

Here is the overview of steps.

## 1. Define Model Hyperparameters.
- Select the horizon (*T*), which is the number of seconds into future of the moving vehicle.
- Select the interval (*dt*), smaller the better, however the computer should have enough power.
- Derive the number of steps (*N = T/dt*)   

I experimented with a range of combinations from [N=6; dt=0.2] to [N=16; dt=0.09]
settled on the following.

 | T (sec)| dt(sec) | N  |
 |:------:|:-------:|:--:|
 |    1   |   0.1   | 10 | 
 
The following are video links for some combinations.

 | T (sec)| dt(sec) | N  | Video Link                    |Comment                     |
 |:------:|:-------:|:--:|:-----------------------------:|:--------------------------:|
 |    1   |   0.1   | 10 | https://youtu.be/pZ_lenzEivo  | Stable                     |
 |   1.2  |   0.2   |  6 | https://youtu.be/kXn03VMzEIo  | Unstable; too large error. |
 |   1.44 |   0.09  | 16 | https://youtu.be/8H_8zPxhLhc  | Unstable; very slow.       |  
 
 The relevant code is in `MPP.cpp`.
 
 
 ## 2. Setup Model equations.
 This involves pedicting next state. There are some finer details of converting global coordinates
 to vehicle oriented coordinates and taking care latency of actuator effet.
 The relevant code is implemented in `main.cpp()` around lines [#97 - #131].
 
## 3. Define Cost function.
This is very experimental in nature. I used the following error term penalty values.

 |Num| Penalty Parameter |Vaulue |comment                                             |
 |:-:|:-----------------:|:-----:|:--------------------------------------------------:|
 |1  |  cte_mul          |4000.0 | interdependent with other parameters & speed ref_v.|
 |2  |  epsi_mul         |2000.0 | intererdepent with other parameters and ref_v.     |
 |3  |  delta_mul        |10.0   | intererdepent with other parameters and ref_v.     |
 |4  |  v_d_mul          |1.0    | intererdepent with other parameters and ref_v.     |
 |5  |  delta_d_mul      |100.0  | intererdepent with other parameters and ref_v.     |
 |6  |  delta_d_mul      |10.0   | intererdepent with other parameters and ref_v.     |

I took help from previous implementations as guideline.
However, it looks like each implementation varied drastically in terms cost penalty multiplier
values.  The relevant code is in `MPP.cpp: operator()`.
I selected the following reference values.
 
 |Num| Ref. Parameter    |Vaulue |comment                                         |
 |:-:|:-----------------:|:-----:|:----------------------------------------------:|
 |1  |  ref_v            |90.0   | MPH. At higher speeds, more fine-tuning needed.|
 |2  |  ref_cte          |0.0    | logical value.                                 |
 |3  |  ref_epsi         |0.0    | logical value.                                 |
 
 The relevant definitions are in `MPP.cpp`.
 
 
## 3. Define Actuator Constraints.
As suggested in lectures and project notes, I used the following values.

 |Num| Constraint        |Lower Bound|Upper Bound| Comment                           |
 |:-:|:-----------------:|:---------:|:---------:|----------------------------------:|
 |1  |  Steering Value   |-0.436332  | 0.436332  |Units are radians [-25 - +25 deg.] |
 |2  |  Throttle         |-1.0       | 1.0       |                                   |

The following lecture snapshot provides a good pictorial description of setup.
![alt text][image1]

## 4. Control Loop
First the pre-processed current state with required translation from global to vehicle 
coordinates and taking care of actuator latency, and polynomial coefficients are passed 
to solver, `mpc.Solve()`. The solver computes optimal steering_angle and throttle values
by using optimizer `Ipopt`, to minimize the cost function, while satsifying the actuator
constraints. The loops repeates until the similator exits.
The following lecture snapshot provides a good pictorial description of control loop.
![alt text][image2]

## 5. Reflection.
This exercise provided an appreciation for the complexity involved for a simple model,
which does not even consider the dynamic forces, such as tire slip, air friction etc.
There are two many levers to play with. In one prominent way this exercise does NOT
address the end of the path plan, though the lecture covered briefly regarding reaching
the destination.

## 6. Results
I chose 90.0 mph as reference speed. At at 70 mph reference speed the vehicle was even more stable.
Here is the link to the video.  
https://youtu.be/pZ_lenzEivo



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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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
