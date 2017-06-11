/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLCartesianTask.hpp"

using namespace trajectory_generation;

RMLCartesianTask::RMLCartesianTask(std::string const& name)
    : RMLCartesianTaskBase(name)
{
}

RMLCartesianTask::RMLCartesianTask(std::string const& name, RTT::ExecutionEngine* engine)
    : RMLCartesianTaskBase(name, engine)
{
}

RMLCartesianTask::~RMLCartesianTask()
{
}
bool RMLCartesianTask::configureHook()
{
    if (! RMLCartesianTaskBase::configureHook())
        return false;
    return true;
}
bool RMLCartesianTask::startHook()
{
    if (! RMLCartesianTaskBase::startHook())
        return false;
    return true;
}
void RMLCartesianTask::updateHook()
{
    RMLCartesianTaskBase::updateHook();
}
void RMLCartesianTask::errorHook()
{
    RMLCartesianTaskBase::errorHook();
}
void RMLCartesianTask::stopHook()
{
    RMLCartesianTaskBase::stopHook();
}
void RMLCartesianTask::cleanupHook()
{
    RMLCartesianTaskBase::cleanupHook();
}
