/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLCartesianVelocityTask.hpp"
#include <base-logging/Logging.hpp>

using namespace trajectory_generation;

RMLCartesianVelocityTask::RMLCartesianVelocityTask(std::string const& name)
    : RMLCartesianVelocityTaskBase(name){
}

RMLCartesianVelocityTask::RMLCartesianVelocityTask(std::string const& name, RTT::ExecutionEngine* engine)
    : RMLCartesianVelocityTaskBase(name, engine){
}

RMLCartesianVelocityTask::~RMLCartesianVelocityTask(){
}

bool RMLCartesianVelocityTask::configureHook(){
    if (! RMLCartesianVelocityTaskBase::configureHook())
        return false;
    return true;
}

bool RMLCartesianVelocityTask::startHook(){
    if (! RMLCartesianVelocityTaskBase::startHook())
        return false;
    return true;
}

void RMLCartesianVelocityTask::updateHook(){
    RMLCartesianVelocityTaskBase::updateHook();
}

void RMLCartesianVelocityTask::errorHook(){
    RMLCartesianVelocityTaskBase::errorHook();
}

void RMLCartesianVelocityTask::stopHook(){
    RMLCartesianVelocityTaskBase::stopHook();
}

void RMLCartesianVelocityTask::cleanupHook(){
    RMLCartesianVelocityTaskBase::cleanupHook();
}

ReflexxesResultValue RMLCartesianVelocityTask::performOTG(RMLInputParameters* new_input_parameters,
                                                          RMLOutputParameters* new_output_parameters,
                                                          RMLFlags *rml_flags){
    checkVelocityTimeout(time_of_last_reference, no_reference_timeout);

    int result = rml_api->RMLVelocity( *(RMLVelocityInputParameters*)new_input_parameters,
                                       (RMLVelocityOutputParameters*)new_output_parameters,
                                       *(RMLVelocityFlags*)rml_flags );

    *new_input_parameters->CurrentPositionVector     = *new_output_parameters->NewPositionVector;
    *new_input_parameters->CurrentVelocityVector     = *new_output_parameters->NewVelocityVector;
    *new_input_parameters->CurrentAccelerationVector = *new_output_parameters->NewAccelerationVector;

    return (ReflexxesResultValue)result;
}

void RMLCartesianVelocityTask::writeCommand(const RMLOutputParameters& new_output_parameters){

}

void RMLCartesianVelocityTask::printParams(){

}

void RMLCartesianVelocityTask::updateTarget(const base::samples::RigidBodyState& cmd,
                                            RMLInputParameters* new_input_parameters){

}

const ReflexxesOutputParameters& RMLCartesianVelocityTask::fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out){

    RMLTask::fromRMLTypes(in, out);
    memcpy(out.position_values_at_target_velocity.data(),
           ((RMLVelocityOutputParameters&)in).PositionValuesAtTargetVelocity->VecData,
           sizeof(double) * in.GetNumberOfDOFs());
    return out;
}
