/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLPositionTask.hpp"
#include <base-logging/Logging.hpp>
#include "Conversions.hpp"

using namespace trajectory_generation;

bool RMLPositionTask::configureHook(){

    rml_flags = new RMLPositionFlags();
    rml_input_parameters = new RMLPositionInputParameters(_motion_constraints.get().size());
    rml_output_parameters = new RMLPositionOutputParameters(_motion_constraints.get().size());

    if (! RMLPositionTaskBase::configureHook())
        return false;
    return true;
}

void RMLPositionTask::updateMotionConstraints(const MotionConstraint& constraint,
                                              const size_t idx,
                                              RMLInputParameters* new_input_parameters){

    toRMLTypes(constraint, idx, *(RMLPositionInputParameters*)new_input_parameters);
}

RTT::FlowStatus RMLPositionTask::updateCurrentState(RMLInputParameters* new_input_parameters){

    RTT::FlowStatus fs = _joint_state.readNewest(joint_state);
    if(fs == RTT::NewData){
        toRMLTypes(joint_state, motion_constraints.names, *new_input_parameters);
        current_sample = joint_state;
    }
    if(fs != RTT::NoData)
        _current_sample.write(current_sample);
    return fs;
}

RTT::FlowStatus RMLPositionTask::updateTarget(RMLInputParameters* new_input_parameters){
    RTT::FlowStatus fs_target = _target.readNewest(target);
    RTT::FlowStatus fs_constr_target = _constrained_target.readNewest(target);

    if(fs_constr_target != RTT::NoData && fs_target != RTT::NoData)
        throw std::runtime_error("There is data on both, the target AND the constrained_target port. You should use only one of the two ports!");

    RTT::FlowStatus fs = RTT::NoData;
    if(fs_target != RTT::NoData)
        fs = fs_target;
    else if(fs_constr_target != RTT::NoData)
        fs = fs_constr_target;

    if(fs == RTT::NewData)
        toRMLTypes(target, *(RMLPositionInputParameters*)new_input_parameters);

    return fs;
}

ReflexxesResultValue RMLPositionTask::performOTG(RMLInputParameters* new_input_parameters,
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

void RMLPositionTask::writeCommand(const RMLOutputParameters& new_output_parameters){
    fromRMLTypes((RMLPositionOutputParameters&)new_output_parameters, command);
    fromRMLTypes((RMLPositionOutputParameters&)new_output_parameters, current_sample);
    current_sample.time = base::Time::now();
    command.time = base::Time::now();
    command.names = motion_constraints.names;
    _command.write(command);
}

void RMLPositionTask::printParams(const RMLInputParameters& in, const RMLOutputParameters& out){
    ((RMLPositionInputParameters&  )in).Echo();
    ((RMLPositionOutputParameters& )out).Echo();
}

const ReflexxesInputParameters& RMLPositionTask::convertRMLInputParams(const RMLInputParameters &in, ReflexxesInputParameters& out){
    fromRMLTypes((RMLPositionInputParameters&)in, out);
    return out;
}

const ReflexxesOutputParameters& RMLPositionTask::convertRMLOutputParams(const RMLOutputParameters &in, ReflexxesOutputParameters& out){
    fromRMLTypes((RMLPositionOutputParameters&)in, out);
    return out;
}
