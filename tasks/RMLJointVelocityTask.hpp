/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLJOINTVELOCITYTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLJOINTVELOCITYTASK_TASK_HPP

#include "trajectory_generation/RMLJointVelocityTaskBase.hpp"

namespace trajectory_generation{
class RMLJointVelocityTask : public RMLJointVelocityTaskBase
{
    friend class RMLJointVelocityTaskBase;

public:
    RMLJointVelocityTask(std::string const& name = "trajectory_generation::RMLJointVelocityTask") : RMLJointVelocityTaskBase(name){}
    RMLJointVelocityTask(std::string const& name, RTT::ExecutionEngine* engine) : RMLJointVelocityTaskBase(name, engine){}
    ~RMLJointVelocityTask(){}
    bool configureHook(){return RMLJointVelocityTaskBase::configureHook();}
    bool startHook(){return RMLJointVelocityTaskBase::startHook();}
    void updateHook(){RMLJointVelocityTaskBase::updateHook();}
    void errorHook(){RMLJointVelocityTaskBase::errorHook();}
    void stopHook(){RMLJointVelocityTaskBase::stopHook();}
    void cleanupHook(){RMLJointVelocityTaskBase::cleanupHook();}
};
}

#endif

