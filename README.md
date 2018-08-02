## CarND-Controls-MPC

The goals/steps of this project are the following:

* Set timesteps N and time elapses between actuations dt
* Fit the polynomial to the waypoints
* Calculate initial cross track error and orientation error values
* Define the components of the cost function (state, actuators, etc).
* Define the model constraints.
* Handels 100 ms latency.


[//]: # (Image References)
[image1]: ./images/mpc_setup.png
[image2]: ./images/kinetic_model.png
[image3]: ./images/figure_1.png
[image4]: ./images/mpc.gif

![alt text][image4]

### Dependencies

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

### Basic Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


### MPC Implementation

![alt text][image1]

MPC reframes the task of following a trajectory to an optimization problem. 
 
Self-driving car uses the center of the lane as its reference trajectory. The MPC simulates different actuator inputs, predicts the resulting trajectory and select the trajectory with minimum cost. 
 
The cost is the offset from the reference state.

MPC optimizes the actuator inputs at each timestep in time to minimize this cost. Once found the lowest cost trajectory, it uses the first set of actuation commands and throw the calculated trajectory away. It then use the new state to calculate a new optimal trajectory. Since it's constantly calculating inputs over the future horizon, it's also called **receding horizon control**.


#### 1. Set N and dt
I tuned the hyperparameters N, dt, and T for the MPC.

* **T:** the prediciton horizon is the duration over which future predictions are made. 
* **N:** the number of timesteps in the horizon. 
* **dt:** time elapses between actuations

*T* is the product of *N* and *dt*, *T = N x dt*.

The general guideline is to set *T* as large as possible, and *dt* as small as possible. Larger values of *dt* result in less frequent actuation, which makes it harder to accurately approximate a continuous reference trajectory.

I did this in lines 9 and 10 in `MPC.cpp`.

#### 2. Fit the polynomials to waypoints

I first transfromed the waypoints from maps' coordinates to vehicle's coordinates. I did this in 2 steps:

1. Shift waypoints to the vehicle's origin
2. Rotate the waypoints through homogenous transformation matrix

I did this in lines 120 to 133 in `main.cpp`.

The reference trajectory is passed to control block as 3rd order polynomial. I fit a polynomial to the transformed waypoints in vehicle's coordinates. I did this in line 136 in `main.cpp`, and the function `polyfit()` is implemented in lines 49 to 68 in `main.cpp`.

#### 3. Calculate cross track error and orientation error

Assum the vehicle is travelling a straight road and the longitudinal direction is teh same as the x-axis:

The cross track error (cte) is the difference between the difference between the line and the current vehicle position y. The reference line is the 3rd order polynomial *f(x)* and CTE at the current state is defined as:

```
cte = f(x) - y
```

The orientation error (eψ) is the desired orientation subtracted from the current orientation.

```
eψ = ψ - ψdes
```
 ​	
ψ is known as part of the state. I calculated ψdes (desired psi) as the tangential angle of the polynomil *f* evaluated at *x*, *arctan(f'(x))*. *f'* is the derivative of the polynomial.

I did this in lines 139 and 140 in `main.cpp`.

#### 4. Define the cost function

cte and eψ should be zero. 

```
cost = 0
for (int t=0, t < N, t++) {
	cost += pow(cte[t], 2)
	cost += pow(epsi[t], 2)
}
```

If vehicle comes to this idealized state in the trajector (i.e., cte and eψ both are zero), the vehicle might stop. Therefore I set a reference velocity to be 40 mph.

```
cost += pow(v[t] - ref_v, 2)
```

To penalize magnitude of the input and its change rate, I also included control inputs in the cost.

```
cost += pow(delta[t],2)
cost += pow(delta[t+1] - delta[t], 2) 
```

I also reduced the speed at turns:

```
cost += pow(v[t] * delta[t], 2)
```


I did this in lines 57 to 76 in `MPC.cpp`.


#### 5. Define model constraints

The vehicle model is defined as following:

![alt text][image2]

I defined this model by constraining the state at time t+1 by setting the values within `fg` to the difference of state at time t+1 and above formula. I did this in lines 118 to 125 in `MPC.cpp`. 

Except the initial state, I also set the corresponding `constraints_lowerbound` and the `constraints_upperbound` values to 0. This force the relationships described by the model to hold true. I did this in lines 204 to 223 in `MPC.cpp`.
 
#### 6. Handle Latency

In a real car, an actuation command won't execute instantly = there will be a delay ~100ms as the command propagates throught the system.

To overcome, I modeled this latency in the MPC system. I simulated using the vehicle model starting from the current state for the duration of the latency. The resulting state from the simulation is the new initial state for MPC.

I did this simulation in lines 278 to 292 in the function `Simulate()` of `MPC.cpp`, and lines 100 to 110 in `main.cpp`.

#### 7. Tuning MPC

I tuned the MPC by plotting the CTE, steering angle, and speed errors in the first 100 iterations:

![alt text][image3]

I multiplied different parts of the cost function with values > 1 to obtain smoother transitions.