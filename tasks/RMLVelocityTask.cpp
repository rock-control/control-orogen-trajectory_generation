/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLVelocityTask.hpp"
#include <base-logging/Logging.hpp>
#include "Conversions.hpp"

using namespace trajectory_generation;

bool RMLVelocityTask::configureHook(){
    rml_flags = new RMLVelocityFlags();
    rml_input_parameters = new RMLVelocityInputParameters(_motion_constraints.get().size());
    rml_output_parameters = new RMLVelocityOutputParameters(_motion_constraints.get().size());

    no_reference_timeout = _no_reference_timeout.get();
    if(base::isNaN(no_reference_timeout))
        no_reference_timeout = base::infinity<double>();
    convert_to_position = _convert_to_position.get();

    if (! RMLVelocityTaskBase::configureHook())
        return false;
    return true;
}

void RMLVelocityTask::updateMotionConstraints(const MotionConstraint& constraint,
                                              const size_t idx,
                                              RMLInputParameters* new_input_parameters){
    motionConstraint2RmlTypes(constraint, idx, *(RMLVelocityInputParameters*)new_input_parameters);
}

RTT::FlowStatus RMLVelocityTask::updateCurrentState(RMLInputParameters* new_input_parameters){
    RTT::FlowStatus fs = _joint_state.readNewest(joint_state);
    if(fs == RTT::NewData && rml_result_value == RML_NOT_INITIALIZED){
        jointState2RmlTypes(joint_state, motion_constraints.names, *new_input_parameters);
        current_sample.names = motion_constraints.names;
        rmlTypes2JointState(*new_input_parameters, current_sample);
    }
    if(fs != RTT::NoData){
        current_sample.time = base::Time::now();
        _current_sample.write(current_sample);
    }
    return fs;
}

RTT::FlowStatus RMLVelocityTask::updateTarget(RMLInputParameters* new_input_parameters){
    RTT::FlowStatus fs_target = _target.readNewest(target);
    RTT::FlowStatus fs_constr_target = _constrained_target.readNewest(target);

    if(fs_constr_target != RTT::NoData && fs_target != RTT::NoData)
        throw std::runtime_error("There is data on both, the target AND the constrained_target port. You should use only one of the two ports!");

    RTT::FlowStatus fs = RTT::NoData;
    if(fs_target != RTT::NoData)
        fs = fs_target;
    else if(fs_constr_target != RTT::NoData)
        fs = fs_constr_target;

    if(fs == RTT::NewData){
        target.validate();
        target2RmlTypes(target, motion_constraints, *(RMLVelocityInputParameters*)new_input_parameters);
#ifdef USING_REFLEXXES_TYPE_IV
        // Workaround: If an element is close to a position limit and the target velocity is pointing in direction of the limit, the sychronization time is computed by
        // reflexxes as if the constrained joint could move freely in the direction of the limit. This leads to incorrect synchronization time for all other elements.
        // Set the target velocity to zero in this case!
        if(rml_flags->PositionalLimitsBehavior == POSITIONAL_LIMITS_ACTIVELY_PREVENT)
            fixRmlSynchronizationBug(cycle_time, *(RMLVelocityInputParameters*)new_input_parameters);
#endif
    }


    return fs;
}

ReflexxesResultValue RMLVelocityTask::performOTG(RMLInputParameters* new_input_parameters, RMLOutputParameters* new_output_parameters, RMLFlags *rml_flags){

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

void RMLVelocityTask::writeCommand(const RMLOutputParameters& new_output_parameters){
    if(convert_to_position)
        rmlTypes2Command((RMLPositionOutputParameters&)new_output_parameters, command);
    else
        rmlTypes2Command((RMLVelocityOutputParameters&)new_output_parameters, command);
    rmlTypes2JointState(*rml_input_parameters, current_sample);
    current_sample.time = command.time = base::Time::now();
    command.names = motion_constraints.names;
    _command.write(command);
}

void RMLVelocityTask::printParams(const RMLInputParameters& in, const RMLOutputParameters& out){
    ((RMLVelocityInputParameters&  )in).Echo();
    ((RMLVelocityOutputParameters& )out).Echo();
}

const ReflexxesInputParameters& RMLVelocityTask::convertRMLInputParams(const RMLInputParameters &in, ReflexxesInputParameters& out){
    rmlTypes2InputParams((RMLVelocityInputParameters&)in, out);
    return out;
}

const ReflexxesOutputParameters& RMLVelocityTask::convertRMLOutputParams(const RMLOutputParameters &in, ReflexxesOutputParameters& out){
    rmlTypes2OutputParams((RMLVelocityOutputParameters&)in, out);
    return out;
}
