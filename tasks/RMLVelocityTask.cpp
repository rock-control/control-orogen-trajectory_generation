/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLVelocityTask.hpp"

using namespace trajectory_generation;

RMLVelocityTask::RMLVelocityTask(std::string const& name)
    : RMLVelocityTaskBase(name){
}

RMLVelocityTask::RMLVelocityTask(std::string const& name, RTT::ExecutionEngine* engine)
    : RMLVelocityTaskBase(name, engine){
}

RMLVelocityTask::~RMLVelocityTask(){
}

bool RMLVelocityTask::configureHook(){
    if (! RMLVelocityTaskBase::configureHook())
        return false;

    rml_input_parameters = new RMLVelocityInputParameters(motion_constraints.size());
    rml_output_parameters = new RMLVelocityOutputParameters(motion_constraints.size());
    rml_flags = new RMLVelocityFlags();

    for(size_t i = 0; i < motion_constraints.size(); i++)
        setMotionConstraints(motion_constraints[i], i);

#ifdef USING_REFLEXXES_TYPE_IV
    rml_flags->PositionalLimitsBehavior = _positional_limits_behavior.get();
#endif
    rml_flags->SynchronizationBehavior = _synchronization_behavior.get();

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
}

ReflexxesResultValue RMLVelocityTask::performOTG(base::commands::Joints &current_command){

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

    return (ReflexxesResultValue)result;
}

void  RMLVelocityTask::setJointState(const base::JointState& state, const size_t idx){

    double position = state.position;

#ifdef USING_REFLEXXES_TYPE_IV

    // bug fix for reflexxes: If the given position is out of limits, the algorithm stops working
    if(rml_flags->PositionalLimitsBehavior == POSITIONAL_LIMITS_ACTIVELY_PREVENT){

        if(state.position >= motion_constraints[idx].max.position)
            position = motion_constraints[idx].max.position - 1e-5;
        if(state.position <= motion_constraints[idx].min.position)
            position = motion_constraints[idx].min.position + 1e-5;
    }

#endif

    rml_input_parameters->CurrentPositionVector->VecData[idx] = position;
}

void RMLVelocityTask::setTarget(const base::JointState& cmd, const size_t idx){
    rml_input_parameters->TargetVelocityVector->VecData[idx] = cmd.speed;
}

void RMLVelocityTask::setMotionConstraints(const trajectory_generation::JointMotionConstraints& constraints, const size_t idx){

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
