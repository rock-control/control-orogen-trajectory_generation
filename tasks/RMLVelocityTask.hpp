/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLVELOCITYTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLVELOCITYTASK_TASK_HPP

#include "trajectory_generation/RMLVelocityTaskBase.hpp"

namespace trajectory_generation{

class RMLVelocityTask : public RMLVelocityTaskBase
{
    friend class RMLVelocityTaskBase;
protected:
    double no_reference_timeout;
    base::Time time_of_last_reference;
    bool convert_to_position;

    /** Perform one step of online trajectory generation (call the RML algorithm with the given parameters). Return the RML result value*/
    virtual ReflexxesResultValue performOTG(RMLInputParameters* new_input_parameters,
                                            RMLOutputParameters* new_output_parameters,
                                            RMLFlags *rml_flag);

    /** Write the generated trajectory to port*/
    virtual void writeCommand(const RMLOutputParameters& new_output_parameters);

    /** Call echo() method for rml input and output parameters*/
    virtual void printParams();

    /** Update the target of a particular joint*/
    virtual void updateTarget(const base::JointState &cmd,
                              const size_t idx,
                              RMLInputParameters* new_input_parameters);

    /** Convert from RMLOutputParameters to orogen type*/
    virtual const ReflexxesOutputParameters& fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out);

public:
    RMLVelocityTask(std::string const& name = "trajectory_generation::RMLVelocityTask");
    RMLVelocityTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~RMLVelocityTask();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

