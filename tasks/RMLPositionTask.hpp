/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLPOSITIONTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLPOSITIONTASK_TASK_HPP

#include "trajectory_generation/RMLPositionTaskBase.hpp"

namespace trajectory_generation{

class RMLPositionTask : public RMLPositionTaskBase
{
    friend class RMLPositionTaskBase;
protected:

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

    /** Update the RML input parameters with the current motion constraints*/
    virtual void setMotionConstraints(const MotionConstraints &constraints, RMLInputParameters* new_input_parameters);
    /** Read the current state from port and update the RML input parameters accordingly. Return the flow status.*/
    virtual RTT::FlowStatus updateCurrentState(RMLInputParameters& new_input_parameters);
    /** Read a new target from port and update the RML input parameters accordingly. Return the flow status.*/
    virtual RTT::FlowStatus updateTarget(RMLInputParameters& new_input_parameters);
    /** Perform online trajectory generation (call the the RML algorithm) and update the RML output parameters. Return the result value.*/
    virtual ReflexxesResultValue performOTG(const RMLInputParameters& new_input_parameters, const RMLFlags& flags, RMLOutputParameters* new_output_parameters);
    /** Update and write the current trajectory sample to port*/
    virtual void writeSample(const RMLOutputParameters& new_output_paramameters);
    /** Call echo() method for rml input and output parameters*/
    virtual void printParams();
    /** Convert from RMLInputParameters to orogen type*/
    virtual const ReflexxesInputParameters& fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out);
    /** Convert from RMLOutputParameters to orogen type*/
    virtual const ReflexxesOutputParameters& fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out);
    /** Handle result of the OTG algorithm. Handle errors.*/
    void handleResultValue(ReflexxesResultValue result_value);

};
}

#endif

