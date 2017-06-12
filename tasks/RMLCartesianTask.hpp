/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLCARTESIANTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLCARTESIANTASK_TASK_HPP

#include "trajectory_generation/RMLCartesianTaskBase.hpp"
#include <base/samples/RigidBodyState.hpp>

namespace trajectory_generation{

class RMLCartesianTask : public RMLCartesianTaskBase
{
    friend class RMLCartesianTaskBase;

protected:
    base::samples::RigidBodyState cartesian_state; /** From input port: Current Cartesian state. Will only be used for initializing RML */
    base::samples::RigidBodyState current_sample;  /** From input port: Current Cartesian interpolator status (position/speed/acceleration)*/
    base::samples::RigidBodyState target;          /** From input port: Target Cartesian position or speed.  */
    base::samples::RigidBodyState command;         /** To output port: Commanded Cartesian position or speed.  */

    /** Read the current state from port. If available, update the RML input parameters. Also return the flow state of the port. */
    virtual RTT::FlowStatus updateCurrentState(const std::vector<std::string> &names,
                                               RMLInputParameters* new_input_parameters);

    /** Read a new target from port. If available, update the RML input parameters. Also return the flow state of the port. */
    virtual RTT::FlowStatus updateTarget(const MotionConstraints& default_constraints,
                                         RMLInputParameters* new_input_parameters);

    /** Perform one step of online trajectory generation (call the RML algorithm with the given parameters). Return the RML result value*/
    virtual ReflexxesResultValue performOTG(RMLInputParameters* new_input_parameters,
                                            RMLOutputParameters* new_output_parameters,
                                            RMLFlags *rml_flag) = 0;

    /** Write the generated trajectory to port*/
    virtual void writeCommand(const RMLOutputParameters& new_output_parameters) = 0;

    /** Call echo() method for rml input and output parameters*/
    virtual void printParams() = 0;

    /** Update the target of a particular joint*/
    virtual void updateTarget(const base::samples::RigidBodyState& cmd,
                              RMLInputParameters* new_input_parameters) = 0;



public:
    RMLCartesianTask(std::string const& name = "trajectory_generation::RMLCartesianTask");
    RMLCartesianTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~RMLCartesianTask();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

