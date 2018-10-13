/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLCartesianPositionTask.hpp"
#include <base-logging/Logging.hpp>
#include "Conversions.hpp"

using namespace trajectory_generation;

bool RMLCartesianPositionTask::configureHook(){
    if(_motion_constraints.get().size() != 6){
        LOG_ERROR("Size of motion constraint must be 6, but is %i", _motion_constraints.get().size());
        return false;
    }

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
    motionConstraint2RmlTypes(constraint, idx, *(RMLPositionInputParameters*)new_input_parameters);
}

bool RMLCartesianPositionTask::updateCurrentState(RMLInputParameters* new_input_parameters){
    RTT::FlowStatus fs = _cartesian_state.readNewest(cartesian_state);
    if(fs == RTT::NewData && !has_current_state){
        cartesianState2RmlTypes(cartesian_state, *new_input_parameters);
        current_sample.sourceFrame = cartesian_state.sourceFrame;
        current_sample.targetFrame = cartesian_state.targetFrame;
        rmlTypes2CartesianState(*new_input_parameters, current_sample);
        has_current_state = true;
    }
    if(fs != RTT::NoData){
        current_sample.time = base::Time::now();
        _current_sample.write(current_sample);
    }
    return has_current_state;
}

bool RMLCartesianPositionTask::updateTarget(RMLInputParameters* new_input_parameters){
    RTT::FlowStatus fs = _target.readNewest(target);
    if(fs == RTT::NewData){
        has_target = true;
        target2RmlTypes(target, *(RMLPositionInputParameters*)new_input_parameters);
#ifdef USING_REFLEXXES_TYPE_IV
        // Crop at limits if POSITIONAL_LIMITS_ACTIVELY_PREVENT is selected, otherwise RML will throw a positional limits error
        if(rml_flags->PositionalLimitsBehavior == POSITIONAL_LIMITS_ACTIVELY_PREVENT)
            cropTargetAtPositionLimits(*(RMLPositionInputParameters*)new_input_parameters);
#endif
    }
    return has_target;
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
    rmlTypes2Command((RMLPositionOutputParameters&)new_output_parameters, command);
    rmlTypes2CartesianState(*rml_input_parameters, current_sample);
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
    rmlTypes2InputParams((RMLPositionInputParameters&)in, out);
    return out;
}

const ReflexxesOutputParameters& RMLCartesianPositionTask::convertRMLOutputParams(const RMLOutputParameters &in, ReflexxesOutputParameters& out){
    rmlTypes2OutputParams((RMLPositionOutputParameters&)in, out);
    return out;
}
