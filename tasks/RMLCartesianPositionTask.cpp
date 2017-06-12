/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLCartesianPositionTask.hpp"

using namespace trajectory_generation;

RMLCartesianPositionTask::RMLCartesianPositionTask(std::string const& name)
    : RMLCartesianPositionTaskBase(name){
}

RMLCartesianPositionTask::RMLCartesianPositionTask(std::string const& name, RTT::ExecutionEngine* engine)
    : RMLCartesianPositionTaskBase(name, engine){
}

RMLCartesianPositionTask::~RMLCartesianPositionTask(){
}

bool RMLCartesianPositionTask::configureHook(){
    if (! RMLCartesianPositionTaskBase::configureHook())
        return false;
    return true;
}

bool RMLCartesianPositionTask::startHook(){
    if (! RMLCartesianPositionTaskBase::startHook())
        return false;
    return true;
}

void RMLCartesianPositionTask::updateHook(){
    RMLCartesianPositionTaskBase::updateHook();
}

void RMLCartesianPositionTask::errorHook(){
    RMLCartesianPositionTaskBase::errorHook();
}

void RMLCartesianPositionTask::stopHook(){
    RMLCartesianPositionTaskBase::stopHook();
}

void RMLCartesianPositionTask::cleanupHook(){
    RMLCartesianPositionTaskBase::cleanupHook();
}

ReflexxesResultValue RMLCartesianPositionTask::performOTG(RMLInputParameters* new_input_parameters,
                                                          RMLOutputParameters* new_output_parameters,
                                                          RMLFlags *rml_flag){

}

void RMLCartesianPositionTask::writeCommand(const RMLOutputParameters& new_output_parameters){

}

void RMLCartesianPositionTask::printParams(){

}

void RMLCartesianPositionTask::updateTarget(const base::samples::RigidBodyState& cmd,
                                            RMLInputParameters* new_input_parameters){

}

const ReflexxesInputParameters& RMLCartesianPositionTask::fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out){
    RMLTask::fromRMLTypes(in, out);
    memcpy(out.max_velocity_vector.data(), ((RMLPositionInputParameters&)in).MaxVelocityVector->VecData, sizeof(double) * in.GetNumberOfDOFs());
    memcpy(out.target_position_vector.data(), ((RMLPositionInputParameters&)in).TargetPositionVector->VecData, sizeof(double) * in.GetNumberOfDOFs());
    return out;
}

const ReflexxesOutputParameters& RMLCartesianPositionTask::fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out){
    RMLTask::fromRMLTypes(in, out);
#ifdef USING_REFLEXXES_TYPE_IV
    out.trajectory_exceeds_target_position = ((RMLPositionOutputParameters&)in).TrajectoryExceedsTargetPosition;
#endif
    return out;
}
