/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLCARTESIANPOSITIONTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLCARTESIANPOSITIONTASK_TASK_HPP

#include "trajectory_generation/RMLCartesianPositionTaskBase.hpp"

namespace trajectory_generation{

/** Position based implementation of a Cartesian RML task. The given target has to have valid position entries. Velocity entries are optional.*/
class RMLCartesianPositionTask : public RMLCartesianPositionTaskBase
{
    friend class RMLCartesianPositionTaskBase;
protected:
    /** Perform one step of online trajectory generation (call the RML algorithm with the given parameters). Return the RML result value*/
    virtual ReflexxesResultValue performOTG(RMLInputParameters* new_input_parameters,
                                            RMLOutputParameters* new_output_parameters,
                                            RMLFlags *rml_flag);

    /** Write the generated trajectory to port*/
    virtual void writeCommand(const RMLOutputParameters& new_output_parameters);

    /** Call echo() method for rml input and output parameters*/
    virtual void printParams();

    /** Update the target of a particular joint*/
    virtual void updateTarget(const base::samples::RigidBodyState& cmd,
                              RMLInputParameters* new_input_parameters);

    /** Convert from RMLInputParameters to orogen type*/
    virtual const ReflexxesInputParameters& fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out);

    /** Convert from RMLOutputParameters to orogen type*/
    virtual const ReflexxesOutputParameters& fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out);


public:
    RMLCartesianPositionTask(std::string const& name = "trajectory_generation::RMLCartesianPositionTask");
    RMLCartesianPositionTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~RMLCartesianPositionTask();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

