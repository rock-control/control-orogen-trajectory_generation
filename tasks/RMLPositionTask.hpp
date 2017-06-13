/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLPOSITIONTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLPOSITIONTASK_TASK_HPP

#include "trajectory_generation/RMLPositionTaskBase.hpp"

namespace trajectory_generation{

class RMLPositionTask : public RMLPositionTaskBase
{
    friend class RMLPositionTaskBase;
public:
    RMLPositionTask(std::string const& name = "trajectory_generation::RMLPositionTask");
    RMLPositionTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~RMLPositionTask(){}
    bool configureHook(){return RMLPositionTaskBase::configureHook();}
    bool startHook(){return RMLPositionTaskBase::startHook();}
    void updateHook(){RMLPositionTaskBase::updateHook();}
    void errorHook(){RMLPositionTaskBase::errorHook();}
    void stopHook(){RMLPositionTaskBase::stopHook();}
    void cleanupHook(){RMLPositionTaskBase::cleanupHook();}
};
}

#endif

