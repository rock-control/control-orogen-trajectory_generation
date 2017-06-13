/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLVELOCITYTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLVELOCITYTASK_TASK_HPP

#include "trajectory_generation/RMLVelocityTaskBase.hpp"

namespace trajectory_generation{

class RMLVelocityTask : public RMLVelocityTaskBase
{
    friend class RMLVelocityTaskBase;
public:
    RMLVelocityTask(std::string const& name = "trajectory_generation::RMLVelocityTask");
    RMLVelocityTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~RMLVelocityTask(){}
    bool configureHook();
    bool startHook(){return RMLVelocityTaskBase::startHook();}
    void updateHook(){RMLVelocityTaskBase::updateHook();}
    void errorHook(){RMLVelocityTaskBase::errorHook();}
    void stopHook(){RMLVelocityTaskBase::stopHook();}
    void cleanupHook(){RMLVelocityTaskBase::cleanupHook();}
};
}

#endif

