/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLPositionTask.hpp"
#include <base-logging/Logging.hpp>

using namespace trajectory_generation;

RMLPositionTask::RMLPositionTask(std::string const& name = "trajectory_generation::RMLPositionTask") : RMLPositionTaskBase(name){
    LOG_WARN("RMLPositionTask is deprecated. Use RMLJointPositionTask instead!");
}

RMLPositionTask::RMLPositionTask(std::string const& name, RTT::ExecutionEngine* engine) : RMLPositionTaskBase(name, engine){
    LOG_WARN("RMLPositionTask is deprecated. Use RMLJointPositionTask instead!");
}
