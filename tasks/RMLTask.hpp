/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLTASK_TASK_HPP

#include "trajectory_generation/RMLTaskBase.hpp"
#include <base/commands/Joints.hpp>
#include "trajectory_generationTypes.hpp"
#include <reflexxes/ReflexxesAPI.h>

/* TODOs (D.M, 2016/06/28):
 *
 * - Introduce a "stop" functionality, which leads to a controlled stop of the robot (while repecting the motion constraints). This can be useful
 *   if, e.g. the robot shall be stopped by some external sensor event, without performing a 'hard' stop
 * - Introduce a "reset" functionality, which sets the current interpolator state to the actual state. This can be used after the robot has stopped
 *   to bring the system to a safe initial state
 * - Add the possibility to control compliant joints. Here the problem is that the interpolator state is quite often not the same as the actual
 *   robot state (because of a position deviation due to an external force applied to the robot). This could be handles by adding a "maximum allowed
 *   deviation" between actual and interpolator state. However, the continuity of the output signal has to be ensured at all times!
 */

namespace trajectory_generation{

/** This task generates a feasible, time-stamped trajectory to given a target (position/velocity, depending on the subclass used).
 *  "Feasible" means here that the output trajectory (command port) will respect the motion constraints defined by the
 *  motion_constraints-property, that is maximum/minimum position (only Reflexxes TypeIV), maximum speed, maximum
 *  acceleration and maximum jerk (derivative of acceleration). The motion constraints structure is define in trajectory_generationTypes.hpp.
 */
class RMLTask : public RMLTaskBase
{
    friend class RMLTaskBase;
protected:

    /** Motion constraints that define the properties of the output trajectory (command-port). These include the maximum/minimum position,
      * maximum maximum speed, maximum acceleration and maximum jerk (derivative of acceleration).*/
    trajectory_generation::MotionConstraints motion_constraints;
    /** Interface to the Online Trajectory Generation algorithms of the Reflexxes Motion Libraries*/
    ReflexxesAPI* rml_api;
    /** Input parameters for the OTG algorithm (target, constraints, flags, ...).*/
    RMLInputParameters *rml_input_parameters;
    /** Output parameters of the OTG algorithm (new states, errors, ...).*/
    RMLOutputParameters *rml_output_parameters;
    /** Input flags for the OTG algorithm.*/
    RMLFlags* rml_flags;
    /** Has the rml algorithm been initialized with the current joint state*/
    ReflexxesResultValue rml_result_value;
    /** RMLInputParameters do not work with orogen, so use own type*/
    ReflexxesInputParameters input_parameters;
    /** RMLOutputParameters do not work with orogen, so use own type*/
    ReflexxesOutputParameters output_parameters;
    /** Timestamp if updateHook();*/
    base::Time timestamp;
    /** Cycle time for interpolation*/
    double cycle_time;

    /** Update the RML input parameters with the current motion constraints*/
    virtual void setMotionConstraints(const MotionConstraints &constraints, RMLInputParameters* new_input_parameters) = 0;
    /** Read the current state from port and update the RML input parameters accordingly. Return the flow status.*/
    virtual RTT::FlowStatus updateCurrentState(RMLInputParameters& new_input_parameters) = 0;
    /** Read a new target from port and update the RML input parameters accordingly. Return the flow status.*/
    virtual RTT::FlowStatus updateTarget(RMLInputParameters& new_input_parameters) = 0;    
    /** Perform online trajectory generation (call the the RML algorithm) and update the RML output parameters. Return the result value.*/
    virtual ReflexxesResultValue performOTG(const RMLInputParameters& new_input_parameters, const RMLFlags& flags, RMLOutputParameters* new_output_parameters) = 0;
    /** Update and write the current trajectory sample to port*/
    virtual void writeSample(const RMLOutputParameters& new_output_paramameters);
    /** Call echo() method for rml input and output parameters*/
    virtual void printParams() = 0;

    /** Handle result of the OTG algorithm. Handle errors.*/
    void handleResultValue(ReflexxesResultValue result_value);
    /** Convert from RMLInputParameters to orogen type*/
    virtual const ReflexxesInputParameters& fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out);
    /** Convert from RMLOutputParameters to orogen type*/
    virtual const ReflexxesOutputParameters& fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out);

public:
    RMLTask(std::string const& name = "trajectory_generation::RMLTask");
    RMLTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~RMLTask();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

