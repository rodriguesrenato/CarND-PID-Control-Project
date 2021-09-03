# CarND-Controls-PID

![](images/pid_30mph_c3.gif)

This project implements a PID controller for the simulated car drives close as possible to the center of the lane and perform a full lap around the lake trak of the Udacity Term2 Simulator.

A video of the car performing a full lap can be found [here](https://youtu.be/_DgkelTIHSw).

This is the 8th project of the Udacity Self-Driving Car Engineer Nanodegree Program.

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


## Results

These are the parameters for each successful test:

- **Test 1** - `Throttle`: 0.3, `speed`: 34mph, `PID params`: [0.11,0.0,2.5]
- **Test 2** - `Throttle`: 0.3, `speed`: 34mph, `PID params`: [0.12,0.006,3.5]
- **Test 3** - `Throttle`: 0.4, `speed`: 40mph, `PID params`: [0.12,0.01,3.5]

Before start tuning the params, I build a debug function to print all the variables, which helped me understand effects of each params on each error and the final output of the pid controller. This is an example of a line of the printed output on the terminal:

```shell
   CTE: 0.464   angle/steer_angle:   -2.492|   -0.110 Total Error:   -0.110 | Err:    0.464,  360.325,    0.021 | K * Err:   -0.049,   -0.000,   -0.062
```

On the `Err` and `K * Err` sections, the respective values follow this sequence: `proportional`,`integral`,`derivative` terms of the PID controller.

The start point is to find a good candidate for the `Kp` value. Values around `0.1` results in acceptable oscillatory behaviour around the center of the lane to be used as a start point to tune next parameters.

-  using higher Kp, the car oscillates more around the center of the lane, straight or curved path.
- using a lower Kp, it reduces the oscillations, but reduces the response on curves. 

The following parameter to be tuned was the `kd`. Multiple tests were done starting from `kd=1.0` and higher `kd` values performed better than lower ones.  

- High `Kd` reflects in quicker corrections, but with a cost of higher jerk.
- Values around `2.0` to `4.0` were acceptable, and the `kd` has to be tuned along with `Kp` to find the best combination for a target speed.
- `kd` should be higher for higher target speed to keep the car in the lane area on a curve.

Finally, the `Ki` parameter is going to be tuned. For the integral term of the controller, the update step of the PID controller was implemented in two versions:

- Version 1 - The standard way : 

```c++
i_error_ += cte;
```

- Version 2 - Integral windup with zero-crossing reset [[1]](https://en.wikipedia.org/wiki/Integral_windup) [[2]](https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf) and cte buffer with fixed size:

```c++
if (cte > 0 && p_error_ < 0 || cte < 0 && p_error_ > 0) {
    cte_int_.clear();
  }
  cte_int_.push_back(cte);
  if (cte_int_.size() > integral_buffer_size) {
    cte_int_.erase(cte_int_.begin());
  }
  i_error_ = 0;
  for (auto i : cte_int_) {
    i_error_ += i;
  }
```

The **version 1** is the classic solution and workes fine except when the speed is low (error accumulates faster to large values) and when the car takes more time to get back to center, especially on curves going with high speed, resulting in a high accumulated error that takes too long to stabilizes.

The **version 2** was a combination of a solution mentioned for this problem, called **integral windup**, with a buffer of the previous errors. This approach partially solves the problems mentioned previously. The buffer of previous errors with fixed size helps to avoid large accumulated errors. The buffer is cleared on zero crossing cte to reset the accumulated error and start a new cycle from the set point.

The version chosen was **version 2**, which gives much better results. As the accumulated error can get higher than the `proportional` and the `derivative` errors, it was tuned starting from `0.001` and ending with values between `0.006` and `0.01`.

The `PD` controller worked well for this simulation. When compared to `PID` controller, the `PD` has lower oscillations around the center of the lane, but most of the time the car takes much more time to get closer to the center of the lane, sometimes it drives better but at cost of driver close ( cte near zero, but not crossing) to the center of lane.

The `PID` controller worked better than `PD` in some situations, especially on straight sections of the track.

There are also two different configurations for the PID controller for different speeds: `Phase::LowSpeed` and `Phase::HighSpeed`. When testing this controller at higher speeds, it was tested with different PID configurations to improve the response due to the response time begin different according to speed.