[//]: # (Image References)

[image1]: ./img/last_curve_radar.png "Last Curve Radar"
[image2]: ./img/last_curve_laser.png "Last Curve Laser"
[image3]: ./img/last_curve_both.png "Last Curve Both Sensors"
[image4]: ./img/both_d1.png "Final Path Dataset 1"
[image5]: ./img/both_d2.png "Final Path Dataset 2"
[image6]: ./img/ekf_scheme.png "EKF scheme"

# Extended Kalman Filter Project 
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview

In this project an [Extendend Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) is used to estimate the state of a moving object of interest with noisy lidar and radar measurements. 
Accuracy is computed via [Root Mean Squared Error (RMSE)](https://en.wikipedia.org/wiki/Root-mean-square_deviation) metric and compared to thresholds presented in **Results** section.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

## Dependencies

* [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)
  * This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. 
  * For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `


## Main Simulation Protocol

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


**INPUT**: values provided by the simulator to the c++ program

    ["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

    ["estimate_x"] <= kalman filter estimated position x
    ["estimate_y"] <= kalman filter estimated position y
    ["rmse_x"]
    ["rmse_y"]
    ["rmse_vx"]
    ["rmse_vy"]

---

## Code Explanation

#### General Processing Flow

The algorithm used to compute Extended Kalman Filter is shown in the image below.

  ![alt text][image6]

#### Handling first measurements properly

In code block at lines 74-123 of [FusionEKF.cpp](./src/FusionEKF.cpp) there is the initialization of Fusion System. Here the `KalmanFilter` class object `ekf_` is initialized basing on first measurement. 

Initial state `x_` is set to first measurement values in cartesian coordinates. 

If the first measurement is coming from radar, then polar to cartesian conversion is performed.
  
#### Predicting and then updates

* Code block at lines 126-152 of [FusionEKF.cpp](./src/FusionEKF.cpp) represents the Extended Kalman Filter `predict` step, 
* Code block at lines 154-182 of [FusionEKF.cpp](./src/FusionEKF.cpp) represents the Extended Kalman Filter `update` step

#### Handling both radar and laser

In EKF update code block previously defined, there is handled the sensor type.

If a measurement come from **Radar**, 
* the Jacobian is computed using the function `CalculateJacobian` of [tools.cpp file](./src/tools.cpp) and set as H matrix of KalmanFilter object
* Measurement covariance matrix R is set to `R_radar_`

If a measurement come from **Laser**, 
* H matrix is set to `H_laser_`
* Measurement covariance matrix R is set to `R_laser_`

Code containing Extended Kalman Filter equations is contained in [kalman_filter.cpp file](./src/kalman_filter.cpp) whitin the functions:
*  `predict()` : Predict the state. **line 27** 
*  `update()` : Update the state by using Kalman Filter equations **line 36**
*  `updateEKF()` : Update the state by using Extended Kalman Filter equations **line 59**

#### Avoiding unnecessary computations
  - Unecessary computations are avoided throughout the project.
  - In particular at line 135-136-137 of [FusionEKF.cpp](./src/FusionEKF.cpp) when updating process covariance matrix Q

#### Helper Functions

In file [tools.cpp](./src/tools.cpp) there are two helper functions:

  - `normalize_radians`: this function takes an angle expressed in radians and normalize its value in range `[-pi, pi]`
  - `cartesianPolar`: this function takes in input a cartesian space state vector `[px, py, vx, vy]` containing position `(px, py)` and vector `(vx, vy)` and convert it to polar space state vector `(rho, phi, rho_dot)`

#### Codestyle

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) were used to validate code code.

Codestyle validation has been led via Cpplint.

[Cpplint](https://github.com/cpplint/cpplint) is a static code checker for C++ validating code using the Google's style guide. 

**Command to validate code**

```bash
cpplint --filter=-build/include_subdir,-runtime/int FusionEKF.cpp kalman_filter.cpp tools.cpp 
```

This command returns no error meaning that the code respects the guidelines.

The only warnings filtered were:

* `Use int16/int64/etc, rather than the C type long  [runtime/int]`
* `Include the directory when naming .h files  [build/include_subdir]`

But they were considered minor fix and skipped.


## Results

In order to evaluate the accuracy to the filter, the metric [Root Mean Squared Error (RMSE)](https://en.wikipedia.org/wiki/Root-mean-square_deviation) is used.

The code for its computation is contained in function `calculateRMSE` at lines 15-47 of [tools.cpp](./src/tools.cpp) 

Evaluation has been conducted on both Dataset1 and Dataset2 avaiable in the simulator.

The three run configuration can be specified at compile time by setting the variable `mode` of FusionEKF object. 

Supported values are defined in [FusionEKF.h](./src/FusionEKF.h) and set in FusionEKF constructor :

```cpp
#define BOTH_SENSORS 0
#define ONLY_RADAR 1
#define ONLY_LASER 2
```

### Baseline

The baseline for computation of accuracy metric is showed in the table below.

Param | Threshold
----  | -----
**RMSE x** | **`0.11`** 
**RMSE y** | **`0.11`**
**RMSE vx** | **`0.52`** 
**RMSE vy** | **`0.52`**

The results will be presented with a colored box meaning how close the value is to baseline according to the legend:

- Value above threshold : ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)
- Value below threshold : ![#00A800](https://placehold.it/15/00A800/000000?text=+)
- Value near to threshold : ![#1589F0](https://placehold.it/15/FFA500/000000?text=+)


### Only Laser

Param| Dataset 1 | Dataset 2
-----|----- | -----
RMSE x | `0.1473` ![#1589F0](https://placehold.it/15/FFA500/000000?text=+) | `0.1170` ![#1589F0](https://placehold.it/15/FFA500/000000?text=+)
RMSE y | `0.1153` ![#1589F0](https://placehold.it/15/FFA500/000000?text=+) | `0.1262` ![#1589F0](https://placehold.it/15/FFA500/000000?text=+)
RMSE vx | `0.6383` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+) | `0.6501` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)
RMSE vy | `0.5346` ![#1589F0](https://placehold.it/15/FFA500/000000?text=+) | `0.6108` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)


**Comment**

When using only laser to update Extended Kalman Filter, values obtained are near to threshold but neither of them is below its corresponding threhsold.

Laser sensor is good in estimating position but not good enough.

### Only Radar

Param| Dataset 1 | Dataset 2
-----|----- | -----
RMSE x | `0.2302` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)| `0.2693` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)
RMSE y | `0.3464` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)| `0.3848` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)
RMSE vx | `0.5835` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)| `0.6534` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)
RMSE vy | `0.8040` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)| `0.8638` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)


**Comment**

When using only radar to update Extended Kalman Filter, values obtained are far from given threshold.

Using only Radar sensor is far from being sufficient in completing given task.


### Both Sensors

Param| Dataset 1 | Dataset 2
-----|----- | -----
RMSE x | `0.0973` ![#00A800](https://placehold.it/15/00A800/000000?text=+) | `0.0726` ![#00A800](https://placehold.it/15/00A800/000000?text=+)
RMSE y | `0.0855` ![#00A800](https://placehold.it/15/00A800/000000?text=+) | `0.0967` ![#00A800](https://placehold.it/15/00A800/000000?text=+)
RMSE vx | `0.4513` ![#00A800](https://placehold.it/15/00A800/000000?text=+)| `0.4579` ![#00A800](https://placehold.it/15/00A800/000000?text=+)
RMSE vy | `0.4399` ![#00A800](https://placehold.it/15/00A800/000000?text=+) | `0.4966` ![#00A800](https://placehold.it/15/00A800/000000?text=+)


**Comment**

Using both sensors gives good result in completing this task.

It is good noting how two sensor not sufficient by themselves are successfully combined by Extended Kalman Filter. Remarkable!

Using both radar and laser sensors grant completing successfully task provided.



### Final Path Dataset 1

Result with both sensors activated on Dataset 1.

![alt text][image4]

### Final Path Dataset 2

Result with both sensors activated on Dataset 2.

![alt text][image5]

### Last Curve Detail

In order to give a more detailed look at weaknesses and strengths of the two sensor in this task an inspection of the last curve of dataset 1 is provided with the three configurations: only radar, only lidar, both sensors.

#### Radar

![alt text][image1]

Here we can see that only radar is sub-optimal in following the last curve of the dataset.

#### Laser

![alt text][image2]

Here we can see that laser is better than radar in following the last curve ground-truth.

#### Both

![alt text][image3]

In combination of the two sensors it is shown how EKF has learned to rely more on laser sensor during the last curve.

The tracking is better and it is inspired by result using only laser.

The result is **even improved** embodying also radar measurements.

## Known Issues and Open Points

* Generating more data, 
  
  It would be better to have more dataset in order to evaluate better the filter developed. A good dataset for this task could be a track where you have in some segments only one sensor at a time and then other segments with the two sensors combined in order to see how the filter react in such conditions

* Creating test classes with [gtest](https://github.com/google/googletest) automated testing framework.






