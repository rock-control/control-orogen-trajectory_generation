/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLCartesianVelocityTask.hpp"
#include <base-logging/Logging.hpp>

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

    updateMotionConstraints(constraint, idx, (RMLVelocityInputParameters*)new_input_parameters);
}

ReflexxesResultValue RMLCartesianVelocityTask::performOTG(RMLInputParameters* new_input_parameters,
                                                          RMLOutputParameters* new_output_parameters,
                                                          RMLFlags *rml_flags){

    checkVelocityTimeout(time_of_last_reference, no_reference_timeout);

    return performOTG((RMLVelocityInputParameters*)new_input_parameters,
                      (RMLVelocityOutputParameters*)new_output_parameters,
                      (RMLVelocityFlags*)rml_flags);
}

void RMLCartesianVelocityTask::writeCommand(const RMLOutputParameters& new_output_parameters){

    base::Vector3d pos, euler, vel, ang_vel;
    memcpy(pos.data(),     new_output_parameters.NewPositionVector->VecData,   sizeof(double)*3);
    memcpy(euler.data(),   new_output_parameters.NewPositionVector->VecData+3, sizeof(double)*3);
    memcpy(vel.data(),     new_output_parameters.NewVelocityVector->VecData,   sizeof(double)*3);
    memcpy(ang_vel.data(), new_output_parameters.NewVelocityVector->VecData+3, sizeof(double)*3);

    current_sample.position    = pos;
    current_sample.orientation = fromEuler(euler);
    command.velocity           = current_sample.velocity         = vel;
    command.angular_velocity   = current_sample.angular_velocity = ang_vel;

    if( convert_to_position ){
        command.position    = pos;
        command.orientation = fromEuler(euler);
    }

    command.time = base::Time::now();
    command.sourceFrame = target.sourceFrame;
    command.targetFrame = target.targetFrame;
    _command.write(command);
}

void RMLCartesianVelocityTask::printParams(const RMLInputParameters& in, const RMLOutputParameters& out){
    ((RMLVelocityInputParameters&  )in).Echo();
    ((RMLVelocityOutputParameters& )out).Echo();
}

void RMLCartesianVelocityTask::updateTarget(const TargetVector& target_vector,
                                            RMLInputParameters* new_input_parameters){
    updateTarget(target_vector, (RMLVelocityInputParameters*)new_input_parameters);
}

/*
void RMLCartesianVelocityTask::updateTarget(const base::samples::RigidBodyState& cmd,
                                            RMLInputParameters* new_input_parameters){

    if(!cmd.hasValidVelocity() || !cmd.hasValidAngularVelocity()){
        LOG_ERROR("Target velocity and/or angular velocity is invalid");
        throw std::invalid_argument("Invalid target");
    }

    // Reset velocity watchdog
    time_of_last_reference = base::Time::now();

    memset(new_input_parameters->SelectionVector->VecData,        true,                        sizeof(bool)  *6);
    memcpy(new_input_parameters->TargetVelocityVector->VecData,   cmd.velocity.data(),         sizeof(double)*3);
    memcpy(new_input_parameters->TargetVelocityVector->VecData+3, cmd.angular_velocity.data(), sizeof(double)*3);

    for(int i = 0; i < 6; i++){

        // If a joint is at a position limit, the target velocity is non-zero and pointing in direction of the limit, the sychronization time is
        // computed by RML as if the constrained joint could move freely. This might lead to incorrect synchronization time for all other joints.
        // Workaround: Set the target velocity to zero if (and only if) POSITIONAL_LIMITS_ACTIVELY_PREVENT is selected
#ifdef USING_REFLEXXES_TYPE_IV
        if(rml_flags->PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT){
            double cur_pos = new_input_parameters->CurrentPositionVector->VecData[i];
            double target_vel = new_input_parameters->TargetVelocityVector->VecData[i];
            double max_pos = new_input_parameters->MaxPositionVector->VecData[i];
            double min_pos = new_input_parameters->MinPositionVector->VecData[i];

            if( (target_vel*cycle_time + cur_pos > max_pos) || (target_vel*cycle_time + cur_pos < min_pos) )
                new_input_parameters->TargetVelocityVector->VecData[i] = 0;
        }
#endif
    }
}*/

const ReflexxesInputParameters& RMLCartesianVelocityTask::fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out){
    out.fromRMLTypes((RMLVelocityInputParameters&)in);
    return out;
}

const ReflexxesOutputParameters& RMLCartesianVelocityTask::fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out){
    out.fromRMLTypes((RMLVelocityOutputParameters&)in);
    return out;
}
