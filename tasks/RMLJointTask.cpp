/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLJointTask.hpp"

using namespace trajectory_generation;

RMLJointTask::RMLJointTask(std::string const& name)
    : RMLJointTaskBase(name)
{
}

RMLJointTask::RMLJointTask(std::string const& name, RTT::ExecutionEngine* engine)
    : RMLJointTaskBase(name, engine)
{
}

RMLJointTask::~RMLJointTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See RMLJointTask.hpp for more detailed
// documentation about them.

bool RMLJointTask::configureHook()
{
    if (! RMLJointTaskBase::configureHook())
        return false;
    return true;
}
bool RMLJointTask::startHook()
{
    if (! RMLJointTaskBase::startHook())
        return false;
    return true;
}
void RMLJointTask::updateHook()
{
    RMLJointTaskBase::updateHook();
}
void RMLJointTask::errorHook()
{
    RMLJointTaskBase::errorHook();
}
void RMLJointTask::stopHook()
{
    RMLJointTaskBase::stopHook();
}
void RMLJointTask::cleanupHook()
{
    RMLJointTaskBase::cleanupHook();
}
