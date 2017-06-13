/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLCartesianPositionTask.hpp"
#include <base-logging/Logging.hpp>
#include "Conversions.hpp"

using namespace trajectory_generation;

bool RMLCartesianPositionTask::configureHook(){

    rml_flags = new RMLPositionFlags();
    rml_input_parameters = new RMLPositionInputParameters(_motion_constraints.get().size());
    rml_output_parameters = new RMLPositionOutputParameters(_motion_constraints.get().size());

    if (! RMLCartesianPositionTaskBase::configureHook())
        return false;
    return true;
}


void RMLCartesianPositionTask::updateMotionConstraints(const MotionConstraint& constraint,
                                                       const size_t idx,
                                                       RMLInputParameters* new_input_parameters){
    toRMLTypes(constraint, idx, *(RMLPositionInputParameters*)new_input_parameters);
}

RTT::FlowStatus RMLCartesianPositionTask::updateCurrentState(RMLInputParameters* new_input_parameters){
    RTT::FlowStatus fs = _cartesian_state.readNewest(cartesian_state);
    if(fs == RTT::NewData && rml_result_value == RML_NOT_INITIALIZED){
        toRMLTypes(cartesian_state, *new_input_parameters);
        current_sample = cartesian_state;
    }
    if(fs != RTT::NoData)
        _current_sample.write(current_sample);
    return fs;
}

RTT::FlowStatus RMLCartesianPositionTask::updateTarget(RMLInputParameters* new_input_parameters){
    RTT::FlowStatus fs = _target.readNewest(target);
    if(fs == RTT::NewData)
        toRMLTypes(target, *(RMLPositionInputParameters*)new_input_parameters);
    return fs;
}

ReflexxesResultValue RMLCartesianPositionTask::performOTG(RMLInputParameters* new_input_parameters,
                                                          RMLOutputParameters* new_output_parameters,
                                                          RMLFlags *rml_flags){

    int result = rml_api->RMLPosition( *(RMLPositionInputParameters*)new_input_parameters,
                                       (RMLPositionOutputParameters*)new_output_parameters,
                                       *(RMLPositionFlags*)rml_flags );

    // Always feed back the new state as the current state. This means that the current robot position
    // is completely ignored. However, on a real robot, using the current position as input in RML will NOT work!
    *new_input_parameters->CurrentPositionVector     = *new_output_parameters->NewPositionVector;
    *new_input_parameters->CurrentVelocityVector     = *new_output_parameters->NewVelocityVector;
    *new_input_parameters->CurrentAccelerationVector = *new_output_parameters->NewAccelerationVector;

    return (ReflexxesResultValue)result;
}

void RMLCartesianPositionTask::writeCommand(const RMLOutputParameters& new_output_parameters){
    fromRMLTypes((RMLPositionOutputParameters&)new_output_parameters, command);
    fromRMLTypes((RMLPositionOutputParameters&)new_output_parameters, current_sample);
    current_sample.time = command.time = base::Time::now();
    command.sourceFrame = target.sourceFrame;
    command.targetFrame = target.targetFrame;
    _command.write(command);
}

void RMLCartesianPositionTask::printParams(const RMLInputParameters& in, const RMLOutputParameters& out){
    ((RMLPositionInputParameters&  )in).Echo();
    ((RMLPositionOutputParameters& )out).Echo();
}

const ReflexxesInputParameters& RMLCartesianPositionTask::convertRMLInputParams(const RMLInputParameters &in, ReflexxesInputParameters& out){
    fromRMLTypes((RMLPositionInputParameters&)in, out);
    return out;
}

const ReflexxesOutputParameters& RMLCartesianPositionTask::convertRMLOutputParams(const RMLOutputParameters &in, ReflexxesOutputParameters& out){
    fromRMLTypes((RMLPositionOutputParameters&)in, out);
    return out;
}
