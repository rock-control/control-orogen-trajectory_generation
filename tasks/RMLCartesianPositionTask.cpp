/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLCartesianPositionTask.hpp"

using namespace trajectory_generation;

RMLCartesianPositionTask::RMLCartesianPositionTask(std::string const& name)
    : RMLCartesianPositionTaskBase(name)
{
}

RMLCartesianPositionTask::RMLCartesianPositionTask(std::string const& name, RTT::ExecutionEngine* engine)
    : RMLCartesianPositionTaskBase(name, engine)
{
}

RMLCartesianPositionTask::~RMLCartesianPositionTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See RMLCartesianPositionTask.hpp for more detailed
// documentation about them.

bool RMLCartesianPositionTask::configureHook()
{
    if (! RMLCartesianPositionTaskBase::configureHook())
        return false;
    return true;
}
bool RMLCartesianPositionTask::startHook()
{
    if (! RMLCartesianPositionTaskBase::startHook())
        return false;
    return true;
}
void RMLCartesianPositionTask::updateHook()
{
    RMLCartesianPositionTaskBase::updateHook();
}
void RMLCartesianPositionTask::errorHook()
{
    RMLCartesianPositionTaskBase::errorHook();
}
void RMLCartesianPositionTask::stopHook()
{
    RMLCartesianPositionTaskBase::stopHook();
}
void RMLCartesianPositionTask::cleanupHook()
{
    RMLCartesianPositionTaskBase::cleanupHook();
}
