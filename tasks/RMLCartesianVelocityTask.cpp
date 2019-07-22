/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLCartesianVelocityTask.hpp"
#include <base-logging/Logging.hpp>
#include "Conversions.hpp"

using namespace trajectory_generation;

bool RMLCartesianVelocityTask::configureHook(){
    if(_motion_constraints.get().size() != 6){
        LOG_ERROR("Size of motion constraint must be 6, but is %i", _motion_constraints.get().size());
        return false;
    }

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
    motionConstraint2RmlTypes(constraint, idx, *(RMLVelocityInputParameters*)new_input_parameters);
}

bool RMLCartesianVelocityTask::updateCurrentState(RMLInputParameters* new_input_parameters){
    RTT::FlowStatus fs = _cartesian_state.readNewest(cartesian_state);
    if(fs == RTT::NewData && !has_current_state){
        cartesianState2RmlTypes(cartesian_state, *new_input_parameters);
        current_sample.source_frame = cartesian_state.source_frame;
        current_sample.target_frame = cartesian_state.target_frame;
        current_sample = cartesian_state;
        has_current_state = true;
    }
    if(fs != RTT::NoData){
        _current_sample.write(current_sample);
        _current_sample.write(current_sample);
    }
    return has_current_state;
}

bool RMLCartesianVelocityTask::updateTarget(RMLInputParameters* new_input_parameters){
    RTT::FlowStatus fs = _target.readNewest(target);
    if(fs == RTT::NewData){
        has_target = true;
        target2RmlTypes(target, *(RMLVelocityInputParameters*)new_input_parameters);
#ifdef USING_REFLEXXES_TYPE_IV
        // Workaround: If an element is close to a position limit and the target velocity is pointing in direction of the limit, the sychronization time is computed by
        // reflexxes as if the constrained joint could move freely in the direction of the limit. This leads to incorrect synchronization time for all other elements.
        // Set the target velocity to zero in this case!
        if(rml_flags->PositionalLimitsBehavior == POSITIONAL_LIMITS_ACTIVELY_PREVENT)
            fixRmlSynchronizationBug(cycle_time, *(RMLVelocityInputParameters*)new_input_parameters);
#endif
    }
    return has_target;
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
        rmlTypes2Command((RMLPositionOutputParameters&)new_output_parameters, command);
    else
        rmlTypes2Command((RMLVelocityOutputParameters&)new_output_parameters, command);

    rmlTypes2Command((RMLPositionOutputParameters&)new_output_parameters, current_sample);
    current_sample.time = command.time = base::Time::now();
    command.source_frame  = target.source_frame;
    command.target_frame  = target.target_frame;
    _command.write(command);
}

void RMLCartesianVelocityTask::printParams(const RMLInputParameters& in, const RMLOutputParameters& out){
    ((RMLVelocityInputParameters&  )in).Echo();
    ((RMLVelocityOutputParameters& )out).Echo();
}

const ReflexxesInputParameters& RMLCartesianVelocityTask::convertRMLInputParams(const RMLInputParameters &in, ReflexxesInputParameters& out){
    rmlTypes2InputParams((RMLVelocityInputParameters&)in, out);
    return out;
}

const ReflexxesOutputParameters& RMLCartesianVelocityTask::convertRMLOutputParams(const RMLOutputParameters &in, ReflexxesOutputParameters& out){
    rmlTypes2OutputParams((RMLVelocityOutputParameters&)in, out);
    return out;
}
