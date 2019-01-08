# CarND-Path-Planning
## James DiDonato
### Udacity Self Driving Car Engineer Nanodegree - January 2019


This project was completed as part of my enrollment in the  Self Driving Car Engineer Nanodegree from Udacity.

## Requirements
In this project the goal is to safely navigate a vehicle autonomously around a virtual highway with other traffic. The other traffic is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

[//]: # (Image References)

[image1]: ./ReportImages/run1.png "Run1"
[image2]: ./ReportImages/run2.png "Run2"
[image3]: ./ReportImages/FSM.png "FSM"

## Reflection

#### Summary

I was able to succesfully complete the project, here are some photos illustrating two succesfully runs of longer than 10 minutes of driving without incident in an efficient manner:

![alt text][image1]

![alt text][image2]

#### Behavioural Planning

A Finite State Machine (FSM) with three states is used to manage the decision making: Lane Change Left, Lane Change Right, and Keep Lane. After starting out in the Keep Lane state (the vehicle needs to get up to speed before making lane changes), a cost is calculated for each of the three states and the finite state machine transitions to the state with the lowest cost, esentially deciding whether to switch lanes or stay in the same lane. If a decision to switch lanes has been made, the desired lane is passed to the trajectory generator that executes the lane change. Here is the FSM depicted visually:

![alt text][image3]


The three cost functions that make up the state transition function are described:

**Out of Center Lane Cost**
- A cost is assigned to a state if it will move the vehicle outside of the center lane. This is done to motivate the ego vehicle to lane change back into the center lane, partly because the center lane is the safest highway lane to drive in and in some jurisdictions it is illegal to "hang out" in the left lane on the highway. Therefore making a return to the center lane vital to abiding by the law and driving safely.

**Rear End Collision Cost**
- A cost is assigned for the potential of colliding with the closest in lane vehicle. This is done to motivate the vehicle to pass slow cars. The cost increases linearly as the ego vehicle approaches the potential threat and is calibrated to force a lane change at a safe distance, in sensible manner. If the ego vehicle manages to get too close, the cost will be zero to avoid colliding with the vehicle while making the lane change ( because the trajectory planner does not cover this edge case ).

**Open Space**
- This cost is designed to discourage the ego vehicle from causing a collision during a lane change. Adjacent lanes are checked for the existance of a vehicle within a gap spanning from behind ego (to ensure we don't cut anyone off) and sufficiently in front of ego (enough to justify a lane change), about 50m. A cost is computed based on the presence of a vehicle in this gap and increases depending on how close it is to the 'back' of said gap. These costs are only calculated for the Lane Changing states, as there is no threat to a collision while changing lanes if we just want to stay in our lane.

Significant time was spent tuning the weights of each cost functions to ensure that the behaviours were safe and efficient. 

#### Trajectory Generation

The trajectory is generated using a combination of the previous path, the map waypoints, and the spline.h library.  Each iteration, a spline is generated using several points starting with the two starting points of the current path, and then three waypoints at 40,60,and 90m down the road. It is important to note that the three points down the road are located in the target lane, which is generated as a result of the FSM Behaviour Planner. The spline is generated using these 5 points and provides a smooth trajectory for the vehicle to follow. The longitudinal increment between each spline point is generated based on the reference velocity. The higher the velocity, the further each point is spaced out.


The trajectory itself consists of 60 points in total, of which 30 are re-used from the previous un-used path. By re-using the path we are able to control the vehicle more precisely which is especially important when travelling around the corners at higher speeds. Also, it makes the program more efficient, saving the time required to compute the new points. A downside of using points from the previous path is that the adaptive cruise control algorithm that prevents a rear-end collision is slower to respond and exhibhits hysteresis when slowing down and speeding up. The total number of points also influences the trajectory. If the number of points is too long, then the ego vehicle would take very long to change lanes and be at risk for colliding with other cars. Too short and the the lane changes are quick and abrupt, casuing the jerk and acceleration values to exceed their allowable thresholds. 60 points was a happy medium.
   

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Simulator.
In order to run this program, you will need to download the Udacity Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
