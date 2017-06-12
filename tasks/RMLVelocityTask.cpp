/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLVelocityTask.hpp"
#include <base-logging/Logging.hpp>

using namespace trajectory_generation;

RMLVelocityTask::RMLVelocityTask(std::string const& name)
    : RMLVelocityTaskBase(name){
    rml_flags = new RMLVelocityFlags();
}

RMLVelocityTask::RMLVelocityTask(std::string const& name, RTT::ExecutionEngine* engine)
    : RMLVelocityTaskBase(name, engine){
    rml_flags = new RMLVelocityFlags();
}

RMLVelocityTask::~RMLVelocityTask(){
    delete rml_flags;
}

bool RMLVelocityTask::configureHook(){

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

bool RMLVelocityTask::startHook(){
    if (! RMLVelocityTaskBase::startHook())
        return false;
    return true;
}

void RMLVelocityTask::updateHook(){
    RMLVelocityTaskBase::updateHook();
}

void RMLVelocityTask::errorHook(){
    RMLVelocityTaskBase::errorHook();
}

void RMLVelocityTask::stopHook(){
    RMLVelocityTaskBase::stopHook();
}

void RMLVelocityTask::cleanupHook(){
    RMLVelocityTaskBase::cleanupHook();

    delete rml_input_parameters;
    delete rml_output_parameters;
}

ReflexxesResultValue RMLVelocityTask::performOTG(RMLInputParameters* new_input_parameters,
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

void RMLVelocityTask::writeCommand(const RMLOutputParameters& new_output_parameters){

    for(size_t i = 0; i < command.size(); i++){
        current_sample[i].position = new_output_parameters.NewPositionVector->VecData[i];
        command[i].speed           = current_sample[i].speed        = new_output_parameters.NewVelocityVector->VecData[i];
        command[i].acceleration    = current_sample[i].acceleration = new_output_parameters.NewAccelerationVector->VecData[i];
    }

    if( convert_to_position )
        for(size_t i = 0; i < command.size(); i++)
            command[i].position = new_output_parameters.NewPositionVector->VecData[i];

    command.time = base::Time::now();
    _command.write(command);
}

void RMLVelocityTask::printParams(){
    ((RMLVelocityInputParameters*)rml_input_parameters)->Echo();
    ((RMLVelocityOutputParameters*)rml_output_parameters)->Echo();
}

void RMLVelocityTask::updateTarget(const base::JointState &cmd,
                                   const size_t idx,
                                   RMLInputParameters* new_input_parameters){

    if(!cmd.hasSpeed()){
        LOG_ERROR("Target speed of element %i is invalid: %f", idx, cmd.speed);
        throw std::invalid_argument("Invalid target velocity");
    }

    new_input_parameters->TargetVelocityVector->VecData[idx] = cmd.speed;
    new_input_parameters->SelectionVector->VecData[idx]      = true;

    // Reset velocity watchdog
    time_of_last_reference = base::Time::now();

    // If a joint is at a position limit, the target velocity is non-zero and pointing in direction of the limit, the sychronization time is
    // computed by RML as if the constrained joint could move freely. This might lead to incorrect synchronization time for all other joints.
    // Workaround: Set the target velocity to zero if (and only if) POSITIONAL_LIMITS_ACTIVELY_PREVENT is selected
#ifdef USING_REFLEXXES_TYPE_IV
    if(rml_flags->PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT){
        double cur_pos = new_input_parameters->CurrentPositionVector->VecData[idx];
        double target_vel = new_input_parameters->TargetVelocityVector->VecData[idx];
        double max_pos = new_input_parameters->MaxPositionVector->VecData[idx];
        double min_pos = new_input_parameters->MinPositionVector->VecData[idx];

        if( (target_vel*cycle_time + cur_pos > max_pos) || (target_vel*cycle_time + cur_pos < min_pos) )
            new_input_parameters->TargetVelocityVector->VecData[idx] = 0;
    }
#endif

}

const ReflexxesOutputParameters& RMLVelocityTask::fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out){

    RMLTask::fromRMLTypes(in, out);
    memcpy(out.position_values_at_target_velocity.data(),
           ((RMLVelocityOutputParameters&)in).PositionValuesAtTargetVelocity->VecData,
           sizeof(double) * in.GetNumberOfDOFs());
    return out;
}
