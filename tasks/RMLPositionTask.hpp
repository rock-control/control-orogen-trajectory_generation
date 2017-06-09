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

    /** Perform one step of online trajectory generation (call the RML algorithm with the given parameters). Return the RML result value*/
    virtual ReflexxesResultValue performOTG(const CurrentStateVector& current_state, const TargetVector& target);
    /** Call echo() method for rml input and output parameters*/
    virtual void printParams();

};
}

#endif

