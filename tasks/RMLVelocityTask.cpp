/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLVelocityTask.hpp"
#include <base-logging/Logging.hpp>

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

    updateMotionConstraints(constraint, idx, (RMLVelocityInputParameters*)new_input_parameters);
}

ReflexxesResultValue RMLVelocityTask::performOTG(RMLInputParameters* new_input_parameters,
                                                 RMLOutputParameters* new_output_parameters,
                                                 RMLFlags *rml_flags){

    checkVelocityTimeout(time_of_last_reference, no_reference_timeout);

    return performOTG((RMLVelocityInputParameters*)new_input_parameters,
                      (RMLVelocityOutputParameters*)new_output_parameters,
                      (RMLVelocityFlags*)rml_flags);
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

void RMLVelocityTask::printParams(const RMLInputParameters& in, const RMLOutputParameters& out){
    ((RMLVelocityInputParameters&  )in).Echo();
    ((RMLVelocityOutputParameters& )out).Echo();
}

void RMLVelocityTask::updateTarget(const TargetVector& target_vector,
                                    RMLInputParameters* new_input_parameters){
    // Reset velocity watchdog
    time_of_last_reference = base::Time::now();

    updateTarget(target_vector, (RMLVelocityInputParameters*)new_input_parameters);
}

/*
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

}*/

const ReflexxesInputParameters& RMLVelocityTask::fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out){
    out.fromRMLTypes((RMLVelocityInputParameters&)in);
    return out;
}

const ReflexxesOutputParameters& RMLVelocityTask::fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out){
    out.fromRMLTypes((RMLVelocityOutputParameters&)in);
    return out;
}
