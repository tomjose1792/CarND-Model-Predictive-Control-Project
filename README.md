# CarND-Model-Predictive-Control-Project



[//]: # (Image References)

[image1]: ./simulation/Simulation.png "Simulator screen during the run"

### Project Goal and Implementation:

The project goal is to navigate a vehicle on a track in a Udacity-provided simulator, which actually provides telemetry and track waypoint data, by controlling steering and acceleration values. A 100 ms latency is also added to counter the lag in implementing the control values. The project code, makes use of the IPOPT and CPPAD libraries to calculate an optimal trajectory and associated actuation control values in order to minimize the error with a third-degree polynomial fit to the waypoints taken. The optimization considers only a predifned set of waypoints, and predicts a trajectory for that duration based upon a Kinematic model of the vehicle and a cost function based mostly on the vehicle's cross-track error (the distance from the track waypoints and the current position) and orientation angle error.
 
#### Note: Project code files in 'src' folder

For comprehensive instructions on how to install and run project, please, refer to the following repo, which was used as a skeleton for this project: https://github.com/udacity/CarND-MPC-Project 

#### Basic Build instructions
    $ mkdir build && cd build
    $ cmake .. && make
    $ ./mpc

Or use the `$ bash build_and_run.sh` provided to execute the above commands.

### Project Implementation:
--

#### The model:

A kinematic model is used with the following set of equations:
  
      x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
      y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
      psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
      v_[t] = v[t-1] + a[t-1] * dt
      cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
      epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt

which gives the state of the model,

'x','y' (postion of car),'psi' (orientation angle),'v' (velocity), 'cte' (cross track error), 'epsi' (orientation error).

The actuator outputs are 'a'-acceleration(throttle) & 'delta' - steering angle.

The model takes the state and actuations from the previous timestep (t-1) to calculate the state for the current timestep (t) based on the model equations.


#### Timestep Length (N) and Elapsed Duration between timesteps(dt):

The values were selected by trial and error methods. I found that that with too many points like points more than the value of N=10 tended the MPC controller to run slow and later in the track it led the car to go out of track often. Also, tried the values of 200, 300 & 500 ms for dt value. But found that at some point in the track it led to erratic behaviour. Finally, settled with 10 for N and 100 ms for dt.

#### Polynomial Fitting and MPC Preprocessing:
The waypoints received from the simulator are transformed to the vehicle coordinate system. A 3rd degree polynomial is fitted to the transformed points. 

#### Model Predictive Control with Latency:

A latency of 100ms is implemented as given by Udacity. I did vary the latency values from 50 to 100 ms and found how it effected the output. A latency is implemented to account for the delay in implementation of control values as the state for which the state the model calculated the control values would have changed to a new timestep because of the delay. Therefore calculations are carried out for a timestep of 100 ms ahead.

### Simulation:

The simulator screen showing the autonomous drive through a MPC controller.

![alt text][image1]

#### Video
A short video of the run of the car around the track can be found here.https://github.com/tomjose1792/CarND-Model-Predictive-Control-Project/blob/master/simulation/simulation_video.mov 




