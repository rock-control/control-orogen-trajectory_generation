/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLCartesianTask.hpp"

using namespace trajectory_generation;

RMLCartesianTask::RMLCartesianTask(std::string const& name)
    : RMLCartesianTaskBase(name){
}

RMLCartesianTask::RMLCartesianTask(std::string const& name, RTT::ExecutionEngine* engine)
    : RMLCartesianTaskBase(name, engine){
}

RMLCartesianTask::~RMLCartesianTask(){
}

bool RMLCartesianTask::configureHook(){
    if (! RMLCartesianTaskBase::configureHook())
        return false;
    return true;
}

bool RMLCartesianTask::startHook(){
    if (! RMLCartesianTaskBase::startHook())
        return false;
    return true;
}

void RMLCartesianTask::updateHook(){
    RMLCartesianTaskBase::updateHook();
}

void RMLCartesianTask::errorHook(){
    RMLCartesianTaskBase::errorHook();
}

void RMLCartesianTask::stopHook(){
    RMLCartesianTaskBase::stopHook();
}

void RMLCartesianTask::cleanupHook(){
    RMLCartesianTaskBase::cleanupHook();
}

RTT::FlowStatus RMLCartesianTask::updateCurrentState(const std::vector<std::string> &names,
                                           RMLInputParameters* new_input_parameters){

}

RTT::FlowStatus RMLCartesianTask::updateTarget(const MotionConstraints& default_constraints,
                                     RMLInputParameters* new_input_parameters){

}
