## The Trajectory Generation Task Library

The components in this task library uses the Reflexxes Motion Libraries (RML) to generate smooth motion commands based on the current state of the system, the target state and motion constraints. Reflexxes ensures that the generated motion command never violates the given motion constraints (e.g. position limits, max. velocity, acceleration or jerk). Generally, there are two possible ways to use this task library: 

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
* Synchronize the motion of all joints
* Position and velocity-based implementation
* Joints and Cartesian space implementation

## Limitations and Remarks

* Note that RML is meant to be used ONLY for reactive motions with quickly changing, but discrete target points. Examples are sensor-based (e.g. Visual Servoing) or point-to-point motions. RML is not meant to be used for interpolating full trajectories
* The quality of the trajectory depends on the accuracy of this component's period. Real-time systems may significantly improve performance. Furthermore, the cycle time property has to match the period of the component, otherwise the generated motion will be too fast or slow. 
* The current state of the robot will NOT be considered at runtime, simply because (a) RML is not meant to be used this way and (b) it is the job of your robot's joint controllers to be able to follow the given reference. If the reference trajectory is too challenging for your robot controllers, make the motion constraints more conservative. However, if you use e.g. a compliant system and hold the robot so that it is unable to follow the reference trajectory, the components in this task library will currently not realize this difference between reference and actual state.
