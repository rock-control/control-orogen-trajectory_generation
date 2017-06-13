/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLCartesianVelocityTask.hpp"
#include <base-logging/Logging.hpp>
#include "Conversions.hpp"

using namespace trajectory_generation;

bool RMLCartesianVelocityTask::configureHook(){

    rml_flags = new RMLVelocityFlags();
    rml_input_parameters = new RMLVelocityInputParameters(_motion_constraints.get().size());
    rml_output_parameters = new RMLVelocityOutputParameters(_motion_constraints.get().size());

    no_reference_timeout = _no_reference_timeout.get();
    if(base::isNaN(no_reference_timeout))
        no_reference_timeout = base::infinity<double>();
    convert_to_position = _convert_to_position.get();

    if (! RMLCartesianVelocityTaskBase::configureHook())
        return false;
    return true;
}

void RMLCartesianVelocityTask::updateMotionConstraints(const MotionConstraint& constraint,
                                                       const size_t idx,
                                                       RMLInputParameters* new_input_parameters){

    toRMLTypes(constraint, idx, *(RMLVelocityInputParameters*)new_input_parameters);
}

RTT::FlowStatus RMLCartesianVelocityTask::updateCurrentState(RMLInputParameters* new_input_parameters){
    RTT::FlowStatus fs = _cartesian_state.readNewest(cartesian_state);
    if(fs == RTT::NewData && rml_result_value == RML_NOT_INITIALIZED){
        toRMLTypes(cartesian_state, *new_input_parameters);
        current_sample = cartesian_state;
    }
    if(fs != RTT::NoData)
        _current_sample.write(current_sample);
    return fs;
}

RTT::FlowStatus RMLCartesianVelocityTask::updateTarget(RMLInputParameters* new_input_parameters){
    RTT::FlowStatus fs = _target.readNewest(target);
    if(fs == RTT::NewData)
        toRMLTypes(target, *(RMLVelocityInputParameters*)new_input_parameters);
    return fs;
}

ReflexxesResultValue RMLCartesianVelocityTask::performOTG(RMLInputParameters* new_input_parameters,
                                                          RMLOutputParameters* new_output_parameters,
                                                          RMLFlags *rml_flags){

    int result = rml_api->RMLVelocity(*(RMLVelocityInputParameters*)new_input_parameters,
                                       (RMLVelocityOutputParameters*)new_output_parameters,
                                      *(RMLVelocityFlags*)rml_flags );

    // Always feed back the new state as the current state. This means that the current robot position
    // is completely ignored. However, on a real robot, using the current position as input in RML will NOT work!
    *new_input_parameters->CurrentPositionVector     = *new_output_parameters->NewPositionVector;
    *new_input_parameters->CurrentVelocityVector     = *new_output_parameters->NewVelocityVector;
    *new_input_parameters->CurrentAccelerationVector = *new_output_parameters->NewAccelerationVector;

    return (ReflexxesResultValue)result;
}

void RMLCartesianVelocityTask::writeCommand(const RMLOutputParameters& new_output_parameters){
    if(convert_to_position)
        fromRMLTypes((RMLPositionOutputParameters&)new_output_parameters, command);
    else
        fromRMLTypes((RMLVelocityOutputParameters&)new_output_parameters, command);
    fromRMLTypes((RMLPositionOutputParameters&)new_output_parameters, current_sample);
    current_sample.time = command.time = base::Time::now();
    command.sourceFrame = target.sourceFrame;
    command.targetFrame = target.targetFrame;
    _command.write(command);
}

void RMLCartesianVelocityTask::printParams(const RMLInputParameters& in, const RMLOutputParameters& out){
    ((RMLVelocityInputParameters&  )in).Echo();
    ((RMLVelocityOutputParameters& )out).Echo();
}

const ReflexxesInputParameters& RMLCartesianVelocityTask::convertRMLInputParams(const RMLInputParameters &in, ReflexxesInputParameters& out){
    fromRMLTypes((RMLVelocityInputParameters&)in, out);
    return out;
}

const ReflexxesOutputParameters& RMLCartesianVelocityTask::convertRMLOutputParams(const RMLOutputParameters &in, ReflexxesOutputParameters& out){
    fromRMLTypes((RMLVelocityOutputParameters&)in, out);
    return out;
}
