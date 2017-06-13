/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLJOINTPOSITIONTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLJOINTPOSITIONTASK_TASK_HPP

#include "trajectory_generation/RMLJointPositionTaskBase.hpp"

namespace trajectory_generation{
class RMLJointPositionTask : public RMLJointPositionTaskBase
{
    friend class RMLJointPositionTaskBase;

public:
    RMLJointPositionTask(std::string const& name = "trajectory_generation::RMLJointPositionTask") : RMLJointPositionTaskBase(name){}
    RMLJointPositionTask(std::string const& name, RTT::ExecutionEngine* engine) : RMLJointPositionTaskBase(name, engine){}
    ~RMLJointPositionTask(){}
    bool configureHook(){return RMLJointPositionTaskBase::configureHook();}
    bool startHook(){return RMLJointPositionTaskBase::startHook();}
    void updateHook(){RMLJointPositionTaskBase::updateHook();}
    void errorHook(){RMLJointPositionTaskBase::errorHook();}
    void stopHook(){RMLJointPositionTaskBase::stopHook();}
    void cleanupHook(){RMLJointPositionTaskBase::cleanupHook();}
};
}

#endif

