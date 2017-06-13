/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLTASK_TASK_HPP

#include "trajectory_generation/RMLTaskBase.hpp"
#include "trajectory_generationTypes.hpp"
#include <reflexxes/ReflexxesAPI.h>
#include <base/samples/RigidBodyState.hpp>

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

/** This task generates a feasible, time-stamped trajectory to a given a target (position/velocity, depending on the subclass used).
 *  "Feasible" means here that the output trajectory (command port) will respect the motion constraints defined by the
 *  motion_constraints-property, that is maximum/minimum position (only Reflexxes TypeIV), maximum speed, maximum
 *  acceleration and maximum jerk (derivative of acceleration). The motion constraints structure is define in trajectory_generationTypes.hpp.
 */
class RMLTask : public RMLTaskBase
{
    friend class RMLTaskBase;
protected:
    MotionConstraints motion_constraints;        /** Motion constraints that define the properties of the output trajectory*/
    ReflexxesAPI* rml_api;                       /** Interface to the Online Trajectory Generation algorithms of the Reflexxes Motion Libraries*/
    RMLInputParameters *rml_input_parameters;    /** Input parameters for the OTG algorithm (target, constraints, flags, ...).*/
    RMLOutputParameters *rml_output_parameters;  /** Output parameters of the OTG algorithm (new states, errors, ...).*/
    RMLFlags* rml_flags;                         /** Input flags for the RML algorithm.*/
    ReflexxesResultValue rml_result_value;       /** Current result value of RML, will be RML_NOT_INITIALIZED in the beginning*/
    ReflexxesInputParameters input_parameters;   /** RMLInputParameters do not work with orogen, so use own type*/
    ReflexxesOutputParameters output_parameters; /** RMLOutputParameters do not work with orogen, so use own type*/
    base::Time timestamp;                        /** Timestamp if updateHook();*/
    double cycle_time;                           /** Cycle time for interpolation*/

    /** Update the motion constraints of a particular element*/
    virtual void updateMotionConstraints(const MotionConstraint& constraint,
                                         const size_t idx,
                                         RMLInputParameters* new_input_parameters) = 0;

    /** Read the current state from port and return position and flow status*/
    virtual RTT::FlowStatus updateCurrentState(RMLInputParameters* new_input_parameters) = 0;

    /** Update the RML input parameters with the new target */
    virtual RTT::FlowStatus updateTarget(RMLInputParameters* new_input_parameters) = 0;

    /** Perform one step of online trajectory generation (call the RML algorithm with the given parameters). Return the RML result value*/
    virtual ReflexxesResultValue performOTG(RMLInputParameters* new_input_parameters,
                                            RMLOutputParameters* new_output_parameters,
                                            RMLFlags *rml_flags) = 0;

    /** Write the generated trajectory to port*/
    virtual void writeCommand(const RMLOutputParameters& new_output_parameters) = 0;

    /** Call echo() method for rml input and output parameters*/
    virtual void printParams(const RMLInputParameters& in, const RMLOutputParameters& out) = 0;

    /** Convert from RMLInputParameters to orogen type*/
    virtual const ReflexxesInputParameters& convertRMLInputParams(const RMLInputParameters &in, ReflexxesInputParameters& out) = 0;

    /** Convert from RMLOutputParameters to orogen type*/
    virtual const ReflexxesOutputParameters& convertRMLOutputParams(const RMLOutputParameters &in, ReflexxesOutputParameters& out) = 0;

    /** Handle result of the OTG algorithm. Handle errors.*/
    void handleResultValue(ReflexxesResultValue result_value);

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

