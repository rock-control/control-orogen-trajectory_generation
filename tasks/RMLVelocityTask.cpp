/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLVelocityTask.hpp"
#include <base-logging/Logging.hpp>

using namespace trajectory_generation;

RMLVelocityTask::RMLVelocityTask(std::string const& name = "trajectory_generation::RMLVelocityTask") : RMLVelocityTaskBase(name){
    LOG_WARN("RMLVelocityTask is deprecated. Use RMLJointVelocityTask instead!");
}

RMLVelocityTask::RMLVelocityTask(std::string const& name, RTT::ExecutionEngine* engine) : RMLVelocityTaskBase(name, engine){
    LOG_WARN("RMLVelocityTask is deprecated. Use RMLJointVelocityTask instead!");
}
