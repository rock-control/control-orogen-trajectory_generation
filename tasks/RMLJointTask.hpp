/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLJOINTTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLJOINTTASK_TASK_HPP

#include "trajectory_generation/RMLJointTaskBase.hpp"

namespace trajectory_generation{

class RMLJointTask : public RMLJointTaskBase
{
    friend class RMLJointTaskBase;

    RTT::FlowStatus getCurrentState(const std::vector<std::string> &names, base::samples::Joints &current_state);

protected:
    base::samples::Joints joint_state;    /** From input port: Current joint state. Will only be used for initializing RML */
    base::samples::Joints current_sample; /** From input port: Current joint interpolator status (position/speed/acceleration)*/
    ConstrainedJointsCmd target;          /** From input port: Target joint position or speed.  */
    base::commands::Joints command;       /** From input port: Target joint position or speed.  */

    /** Read the current state from port. If available, update the RML input parameters. Also return the flow state*/
    virtual RTT::FlowStatus updateCurrentState(const std::vector<std::string> &names,
                                               RMLInputParameters* new_input_parameters);

    /** Read a new target from port. If available, update the RML input parameters. Also return the flow state*/
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

    virtual void setJointState(const base::JointState &state,
                               const size_t idx,
                               RMLInputParameters* new_input_parameters) = 0;

    virtual void setJointTarget(const base::JointState &cmd,
                                const size_t idx,
                                RMLInputParameters* new_input_parameters) = 0;

    virtual void setMotionConstraints(const MotionConstraint& constraint,
                                      const size_t idx,
                                      RMLInputParameters* new_input_parameters) = 0;

public:
    RMLJointTask(std::string const& name = "trajectory_generation::RMLJointTask");
    RMLJointTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~RMLJointTask();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

