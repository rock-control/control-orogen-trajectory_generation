# The Trajectory Generation Task Library

The components in this task library use Reflexxes to generate smooth motion commands based on the current state of the system, the target state and motion constraints. Reflexxes ensures that the generated motion command never violates the given motion constraints (e.g. position limits, max. velocity, acceleration or jerk). Generally, there are two possible ways to use this task library: 

* As a low-level (safety) interpolator, which ensures that the commands sent to your actuators are always smooth and dynamically feasible
* As a motion profile generator, which creates reference trajectories that obey the given motion constraints

For details on the implementation of Reflexxes, please check the documentation on their [website](http://www.reflexxes.ws/). Please note that there are two different Reflexxes libraries, that provide different features:

* Reflexxes TypeII: Fully open source (LGPL V3.0). It allows arbitrary initial and target states of motion, as well as limitation of velocity and acceleration
* Reflexxes TypeIV: Restricted to scientific usage (individual license). It additionally allows introduction of upper and lower position bounds, as well as jerk limitation. 

Please check against which version of Reflexxes you are linking. By default, TypeII will be used. In order to switch to the Reflexxes TypeIV implementation, please add the following to your `overrides.yml` in the autoproj folder of your Rock installation:

  ```
  - control/reflexxes:
    dfkigit: dfki-control/reflexxes_type_iv.git
    branch: master
    patches:
       -  $AUTOPROJ_SOURCE_DIR/remotes/dfki.control/patches/reflexxes_type_iv.patch
  ```

The Rock componenents within this task library provide the following features:

* Set new motion constraints (min./max. position, max. velocity, max. acceleration and max. jerk) by configuration or at runtime
* Set arbitrary target position/velocity for all joints at runtime (with arbitrary frequency)
* Position and velocity-based implementation
* Joints and Cartesian space implementation
