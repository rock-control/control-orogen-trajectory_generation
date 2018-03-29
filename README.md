## The Trajectory Generation Task Library

The components in this task library use the Reflexxes Motion Libraries (RML) to generate smooth motion commands based on the current state of the system, the target state and motion constraints. Reflexxes ensures that the generated motion command never violates the given motion constraints (e.g. position limits, max. velocity, acceleration or jerk). Generally, there are two possible ways to use this task library:

* As a low-level (safety) interpolator, which ensures that the commands sent to your actuators are always smooth and dynamically feasible
* As a motion profile generator, which creates reference trajectories that obey the given motion constraints

For details on the implementation of Reflexxes, please check the documentation on their [website](http://www.reflexxes.ws/). Please note that there are two different Reflexxes libraries, that provide different features:

* Reflexxes Type II: Fully open source (LGPL V3.0). It allows arbitrary initial and target states of motion, as well as limitation of velocity and acceleration
* Reflexxes Type IV: Restricted to scientific usage (individual license). It additionally allows introduction of upper and lower position bounds, as well as jerk limitation.

Please check against which version of Reflexxes you are linking. By default, Type II will be used. In order to switch to the Reflexxes Type IV implementation, please add the following to your `overrides.yml` in the autoproj folder of your Rock installation (note that you will need access to the [DFKI RIC Gitlab Server](https://git.hb.dfki.de) for this to work):

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
* Joint and Cartesian space implementation

## Examples

Check the [scripts folder](https://github.com/rock-control/control-orogen-trajectory_generation/tree/master/scripts) for examples on each of the components. There are four different implementations:
1. `RMLPositionTask`: Position based implementation in joint space
  * Inputs:
    * Current joint state
    * Target position and (optionally) target velocity for each joint (target port). If velocity is unset, zero target velocity is assumed.
    * (Optionally) Target position / velocity for each joint, plus new motion constraints. Note that changing the motion constraints online might lead to an unresolvable situation, in which case RML will throw an error
  * Outputs:
    * Smooth motion command (position/speed/acceleration)

2. `RMLVelocityTask`: Velocity based implementation in joint space
  * Inputs:
    * Current joint state
    * Target velocity for each joint (target port)
    * (Optionally) Target velocity for each joint, plus new motion constraints. Note that changing the motion constraints online might lead to an unresolvable situation, in which case RML will throw an error
  * Outputs:
    * Smooth motion command. Will be speed/acceleration if `convert_to_position` is set to false or position/speed/acceleration if `convert_to_position` is set to true
    * The output velocity will be set to zero if no new target value arrived for more than `no_reference_timeout` seconds. You can disable the timeout by setting `no_reference_timeout` to infinity.

3. `RMLCartesianPositionTask`: Position based implementation in Cartesian space
  * Inputs:
    * Current Cartesian state
    * Target Cartesian position/orientation and (optionally) target translations/rotational velocity (target port). Note that in the current implementation, the orientation is converted to ZYX-euler angles (wrt. rotated coordinate system). Euler angles are prone to stability problems near singular configurations. These problems can be avoided by limiting the target orientation accordingly.
  * Outputs:
    * Smooth motion command (position/speed)

4. `RMLCartesianVelocityTask`: Velocity based implementation in Cartesian space
  * Inputs:
    * Current Cartesian state
    * Target target translational/angular velocity (target port)
  * Outputs:
    * Smooth motion command. Will be speed/acceleration if `convert_to_position` is set to false or position/speed/acceleration if `convert_to_position` is set to true
    * The output velocity will be set to zero if no new target value arrived for more than `no_reference_timeout` seconds. You can disable the timeout by setting `no_reference_timeout` to infinity.

Each component is based on the `RMLTask` task context. An example configuration looks as follows (for the RMLPositionTask):

  ```
  --- name:default
  # Cycle Time is seconds. IMPORTANT: This value has to match the period of the component. Default is 0.01 which matches the default period.
  cycle_time: 0.01

  # Motion constraints that define the properties of the output trajectory (command-port). These include the maximum/minimum position,
  # maximum maximum speed, maximum acceleration and maximum jerk (derivative of acceleration).
  motion_constraints:
    names: ["Joint1", "Joint2"]
    elements: [{max: {position: 1.0, speed: 0.3, acceleration: 0.5}, min: {position: -1.0}, max_jerk: 1.0},
               {max: {position: 1.5, speed: 0.6, acceleration: 1.0}, min: {position: -1.0}, max_jerk: 1.0}]

  # Behaviour on the position limits (only reflexxes TypeIV!!!). Can be one of POSITIONAL_LIMITS_IGNORE, POSITIONAL_LIMITS_ERROR_MSG_ONLY
  # and POSITIONAL_LIMITS_ACTIVELY_PREVENT. See reflexxes/RMLFlags.h for details.
  positional_limits_behavior: :POSITIONAL_LIMITS_IGNORE

  # Synchronozation behavior for the different joints. Can be one of PHASE_SYNCHRONIZATION_IF_POSSIBLE, ONLY_TIME_SYNCHRONIZATION,
  # ONLY_PHASE_SYNCHRONIZATION and NO_SYNCHRONIZATION. See reflexxes/RMLFlags.h for details.
  synchronization_behavior: :PHASE_SYNCHRONIZATION_IF_POSSIBLE

  ```

## Limitations and Remarks

* Note that RML is meant to be used ONLY for reactive motions with quickly changing, but discrete target points. Examples are sensor-based (e.g. Visual Servoing) or point-to-point motions. RML is not meant to be used for interpolating full trajectories
* The quality of the trajectory depends on the accuracy of this component's period. Real-time systems may significantly improve performance. Furthermore, the cycle time property has to match the period of the component, otherwise the generated motion will be too fast or slow.
* The current state of the robot will NOT be considered at runtime, simply because (a) RML is not meant to be used this way and (b) it is the job of your robot's joint controllers to be able to follow the given reference. If the reference trajectory is too challenging for your robot controllers, make the motion constraints more conservative. However, if you use e.g. a compliant system and hold the robot so that it is unable to follow the reference trajectory, the components in this task library will currently not realize the (possibly increasing) difference between reference and actual state.
* The RML parameters `MinimumSynchronizationTime` and `OverrideValue` are currently not used by the components in this task library
* In the RMLCartesianPosition implementation, the orientation is internally converted to euler angles, which is prone to stability problems near singularities.
