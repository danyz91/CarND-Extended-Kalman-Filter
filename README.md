[//]: # (Image References)

[image1]: ./img/last_curve_radar.png "Last Curve Radar"
[image2]: ./img/last_curve_laser.png "Last Curve Laser"
[image3]: ./img/last_curve_both.png "Last Curve Both Sensors"
[image4]: ./img/both_d1.png "Final Path Dataset 1"
[image5]: ./img/both_d2.png "Final Path Dataset 2"

# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

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

## Other Important Dependencies

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).



## Results



## Baseline

Param | Threshold
----  | -----
**RMSE x** | **`0.11`** 
**RMSE y** | **`0.11`**
**RMSE vx** | **`0.52`** 
**RMSE vy** | **`0.52`**

## Only Laser

Param| Dataset 1 | Dataset 2
-----|----- | -----
RMSE x | `0.1473` ![#1589F0](https://placehold.it/15/FFA500/000000?text=+) | `0.1170` ![#1589F0](https://placehold.it/15/FFA500/000000?text=+)
RMSE y | `0.1153` ![#1589F0](https://placehold.it/15/FFA500/000000?text=+) | `0.1262` ![#1589F0](https://placehold.it/15/FFA500/000000?text=+)
RMSE vx | `0.6383` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+) | `0.6501` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)
RMSE vy | `0.5346` ![#1589F0](https://placehold.it/15/FFA500/000000?text=+) | `0.6108` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)

## Only Radar

Param| Dataset 1 | Dataset 2
-----|----- | -----
RMSE x | `0.2302` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)| `0.2693` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)
RMSE y | `0.3464` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)| `0.3848` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)
RMSE vx | `0.5835` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)| `0.6534` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)
RMSE vy | `0.8040` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)| `0.8638` ![#f03c15](https://placehold.it/15/f03c15/000000?text=+)

## Both Sensors

Param| Dataset 1 | Dataset 2
-----|----- | -----
RMSE x | `0.0973` ![#00A800](https://placehold.it/15/00A800/000000?text=+) | `0.0726` ![#00A800](https://placehold.it/15/00A800/000000?text=+)
RMSE y | `0.0855` ![#00A800](https://placehold.it/15/00A800/000000?text=+) | `0.0967` ![#00A800](https://placehold.it/15/00A800/000000?text=+)
RMSE vx | `0.4513` ![#00A800](https://placehold.it/15/00A800/000000?text=+)| `0.4579` ![#00A800](https://placehold.it/15/00A800/000000?text=+)
RMSE vy | `0.4399` ![#00A800](https://placehold.it/15/00A800/000000?text=+) | `0.4966` ![#00A800](https://placehold.it/15/00A800/000000?text=+)

## Cpplint
```bash
cpplint --filter=-build/include_subdir,-runtime/int FusionEKF.cpp kalman_filter.cpp tools.cpp 
```

* `Use int16/int64/etc, rather than the C type long  [runtime/int] [4]`
* `Include the directory when naming .h files  [build/include_subdir] [4]`


## Final Path Dataset 1
![alt text][image4]

## Final Path Dataset 2
![alt text][image5]

## Last Curve Detail

### Radar

![alt text][image1]

### Laser

![alt text][image2]

### Both

![alt text][image3]







