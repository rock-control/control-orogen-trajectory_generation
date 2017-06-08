/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLCartesianVelocityTask.hpp"

using namespace trajectory_generation;

RMLCartesianVelocityTask::RMLCartesianVelocityTask(std::string const& name)
    : RMLCartesianVelocityTaskBase(name)
{
}

RMLCartesianVelocityTask::RMLCartesianVelocityTask(std::string const& name, RTT::ExecutionEngine* engine)
    : RMLCartesianVelocityTaskBase(name, engine)
{
}

RMLCartesianVelocityTask::~RMLCartesianVelocityTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See RMLCartesianVelocityTask.hpp for more detailed
// documentation about them.

bool RMLCartesianVelocityTask::configureHook()
{
    if (! RMLCartesianVelocityTaskBase::configureHook())
        return false;
    return true;
}
bool RMLCartesianVelocityTask::startHook()
{
    if (! RMLCartesianVelocityTaskBase::startHook())
        return false;
    return true;
}
void RMLCartesianVelocityTask::updateHook()
{
    RMLCartesianVelocityTaskBase::updateHook();
}
void RMLCartesianVelocityTask::errorHook()
{
    RMLCartesianVelocityTaskBase::errorHook();
}
void RMLCartesianVelocityTask::stopHook()
{
    RMLCartesianVelocityTaskBase::stopHook();
}
void RMLCartesianVelocityTask::cleanupHook()
{
    RMLCartesianVelocityTaskBase::cleanupHook();
}
