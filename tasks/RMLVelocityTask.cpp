/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLVelocityTask.hpp"
#include <base/Logging.hpp>

using namespace trajectory_generation;

RMLVelocityTask::RMLVelocityTask(std::string const& name)
    : RMLVelocityTaskBase(name)
{
}

RMLVelocityTask::RMLVelocityTask(std::string const& name, RTT::ExecutionEngine* engine)
    : RMLVelocityTaskBase(name, engine)
{
}

RMLVelocityTask::~RMLVelocityTask()
{
}

bool RMLVelocityTask::configureHook()
{
    if (! RMLVelocityTaskBase::configureHook())
        return false;

    rml_input_parameters = new RMLVelocityInputParameters(motion_constraints.size());
    rml_output_parameters = new RMLVelocityOutputParameters(motion_constraints.size());

    for(size_t i = 0; i < motion_constraints.size(); i++)
        setMotionConstraints(motion_constraints[i], i);

    rml_flags = new RMLVelocityFlags();
    rml_flags->SynchronizationBehavior = _synchronization_behavior.get();
#ifdef USING_REFLEXXES_TYPE_IV
    rml_flags->PositionalLimitsBehavior = _positional_limits_behavior.get();
#endif

    no_reference_timeout = _no_reference_timeout.get();
    convert_to_position = _convert_to_position.get();

    return true;
}

bool RMLVelocityTask::startHook()
{
    if (! RMLVelocityTaskBase::startHook())
        return false;
    return true;
}

void RMLVelocityTask::updateHook()
{
    RMLVelocityTaskBase::updateHook();
}

void RMLVelocityTask::errorHook()
{
    RMLVelocityTaskBase::errorHook();
}

void RMLVelocityTask::stopHook()
{
    RMLVelocityTaskBase::stopHook();
}

void RMLVelocityTask::cleanupHook()
{
    RMLVelocityTaskBase::cleanupHook();
}

void RMLVelocityTask::checkVelocityTimeout()
{
    double t_diff = (base::Time::now() - time_of_last_reference).toSeconds();

    if(!time_of_last_reference.isNull() && t_diff > no_reference_timeout)
    {
        rml_input_parameters->TargetVelocityVector->Set(0.0);
        time_of_last_reference.microseconds = 0;
        LOG_WARN("%s: Timeout: No new reference velocity arrived for %f seconds. Setting target velocity to zero.", this->getName().c_str(), t_diff);
    }
}

ReflexxesResultValue RMLVelocityTask::performOTG(base::commands::Joints &current_command)
{
    checkVelocityTimeout();

    int result = rml_api->RMLVelocity( *(RMLVelocityInputParameters*)rml_input_parameters,
                                       (RMLVelocityOutputParameters*)rml_output_parameters,
                                       *(RMLVelocityFlags*)rml_flags );

    *rml_input_parameters->CurrentVelocityVector     = *rml_output_parameters->NewVelocityVector;
    *rml_input_parameters->CurrentAccelerationVector = *rml_output_parameters->NewAccelerationVector;

    current_command.resize(motion_constraints.size());
    current_command.names = motion_constraints.names;

    for(size_t i = 0; i < motion_constraints.size(); i++){
        current_sample[i].position = rml_output_parameters->NewPositionVector->VecData[i];
        current_command[i].speed = current_sample[i].speed = rml_output_parameters->NewVelocityVector->VecData[i];
        current_command[i].acceleration = current_sample[i].acceleration = rml_output_parameters->NewAccelerationVector->VecData[i];
    }

    if( convert_to_position ){
        for(size_t i = 0; i < motion_constraints.size(); i++){
            if(current_command[i].hasPosition())
                current_command[i].position += cycle_time * current_command[i].speed;
            else
                current_command[i].position = rml_input_parameters->CurrentPositionVector->VecData[i];
        }
    }

    return (ReflexxesResultValue)result;
}

void  RMLVelocityTask::setJointState(const base::JointState& state, const size_t idx)
{
    double position = state.position;

#ifdef USING_REFLEXXES_TYPE_IV

    // Make sure the current position is within limits
    if(rml_flags->PositionalLimitsBehavior == POSITIONAL_LIMITS_ACTIVELY_PREVENT){

        if(state.position >= motion_constraints[idx].max.position)
            position = motion_constraints[idx].max.position - 1e-5;
        if(state.position <= motion_constraints[idx].min.position)
            position = motion_constraints[idx].min.position + 1e-5;
    }

#endif

    rml_input_parameters->CurrentPositionVector->VecData[idx] = position;
}

void RMLVelocityTask::setTarget(const base::JointState& cmd, const size_t idx)
{
    if(!cmd.hasSpeed()){
        LOG_ERROR("Target speed of element %i is invalid: %f", idx, cmd.speed);
        throw std::invalid_argument("Invalid target velocity");
    }

    rml_input_parameters->TargetVelocityVector->VecData[idx] = cmd.speed;

    // Reset velocity watchdog:
    time_of_last_reference = base::Time::now();

    // Bug fix: If a joint is at its limits and the target velocity is non-zero and pointing in direction
    // of the limit, the sychronization time is computed by reflexxes as if the constrained joint could move
    // freely in the direction of the joint limit. This might lead to incorrect synchronization time for all other
    // joints. This code provides a workaround by setting the target velocity of a joint to zero if it is at a
    // joint position limit and the target velocity points in the direction of the limit.
#ifdef USING_REFLEXXES_TYPE_IV
    if(rml_flags->PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT){

        double cur_pos = rml_input_parameters->CurrentPositionVector->VecData[idx];
        double target_vel = rml_input_parameters->TargetVelocityVector->VecData[idx];
        double max_pos = rml_input_parameters->MaxPositionVector->VecData[idx];
        double min_pos = rml_input_parameters->MinPositionVector->VecData[idx];

        if( (target_vel*cycle_time + cur_pos > max_pos) || (target_vel*cycle_time + cur_pos < min_pos) )
            rml_input_parameters->TargetVelocityVector->VecData[idx] = 0;
    }
#endif
}

void RMLVelocityTask::setMotionConstraints(const trajectory_generation::JointMotionConstraints& constraints, const size_t idx){

    // Check if constraints are ok, e.g. max.speed > 0 etc
    constraints.validate();

#ifdef USING_REFLEXXES_TYPE_IV
    rml_input_parameters->MaxPositionVector->VecData[idx] = constraints.max.position;
    rml_input_parameters->MinPositionVector->VecData[idx] = constraints.min.position;
#endif
    rml_input_parameters->MaxAccelerationVector->VecData[idx] = constraints.max.acceleration;
    rml_input_parameters->MaxJerkVector->VecData[idx] = constraints.max_jerk;
}

const ReflexxesOutputParameters& RMLVelocityTask::fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out){

    RMLTask::fromRMLTypes(in, out);
    memcpy(out.position_values_at_target_velocity.data(),
           ((RMLVelocityOutputParameters&)in).PositionValuesAtTargetVelocity->VecData,
           sizeof(double) * in.GetNumberOfDOFs());
    return out;
}

void RMLVelocityTask::printParams(){
    ((RMLVelocityInputParameters*)rml_input_parameters)->Echo();
    ((RMLVelocityOutputParameters*)rml_output_parameters)->Echo();
}
