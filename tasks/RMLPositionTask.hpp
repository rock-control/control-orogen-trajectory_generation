/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLPOSITIONTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLPOSITIONTASK_TASK_HPP

#include "trajectory_generation/RMLPositionTaskBase.hpp"

namespace trajectory_generation{

class RMLPositionTask : public RMLPositionTaskBase
{
    friend class RMLPositionTaskBase;

protected:
    /** Update the motion constraints of a particular joint*/
    virtual void updateMotionConstraints(const MotionConstraint& constraint,
                                         const size_t idx,
                                         RMLInputParameters* new_input_parameters);

    /** Perform one step of online trajectory generation (call the RML algorithm with the given parameters). Return the RML result value*/
    virtual ReflexxesResultValue performOTG(RMLInputParameters* new_input_parameters,
                                            RMLOutputParameters* new_output_parameters,
                                            RMLFlags *rml_flag);

    /** Write the generated trajectory to port*/
    virtual void writeCommand(const RMLOutputParameters& new_output_parameters);

    /** Call echo() method for rml input and output parameters*/
    virtual void printParams();

    /** Update the current state of a particular joint*/
    virtual void updateCurrentState(const base::JointState &state,
                                    const size_t idx,
                                    RMLInputParameters* new_input_parameters);

    /** Update the target of a particular joint*/
    virtual void updateTarget(const base::JointState &cmd,
                              const size_t idx,
                              RMLInputParameters* new_input_parameters);

    /** Convert from RMLInputParameters to orogen type*/
    virtual const ReflexxesInputParameters& fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out);

    /** Convert from RMLOutputParameters to orogen type*/
    virtual const ReflexxesOutputParameters& fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out);

public:
    RMLPositionTask(std::string const& name = "trajectory_generation::RMLPositionTask");
    RMLPositionTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~RMLPositionTask();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

