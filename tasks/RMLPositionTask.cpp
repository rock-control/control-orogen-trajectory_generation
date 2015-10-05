/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLPositionTask.hpp"
#include <base/Logging.hpp>

using namespace trajectory_generation;

RMLPositionTask::RMLPositionTask(std::string const& name)
    : RMLPositionTaskBase(name)
{
}

RMLPositionTask::RMLPositionTask(std::string const& name, RTT::ExecutionEngine* engine)
    : RMLPositionTaskBase(name, engine)
{
}

RMLPositionTask::~RMLPositionTask()
{
}

bool RMLPositionTask::configureHook()
{

    if (! RMLPositionTaskBase::configureHook())
        return false;

    rml_input_parameters = new RMLPositionInputParameters(motion_constraints.size());
    rml_output_parameters = new RMLPositionOutputParameters(motion_constraints.size());

    for(size_t i = 0; i < motion_constraints.size(); i++)
        setMotionConstraints(motion_constraints[i], i);

    rml_flags = new RMLPositionFlags();
    rml_flags->SynchronizationBehavior = _synchronization_behavior.get();
#ifdef USING_REFLEXXES_TYPE_IV
    rml_flags->PositionalLimitsBehavior = _positional_limits_behavior.get();
#endif

    return true;
}
bool RMLPositionTask::startHook()
{
    if (! RMLPositionTaskBase::startHook())
        return false;
    return true;
}

void RMLPositionTask::updateHook()
{
    RMLPositionTaskBase::updateHook();
}

void RMLPositionTask::errorHook()
{
    RMLPositionTaskBase::errorHook();
}

void RMLPositionTask::stopHook()
{
    RMLPositionTaskBase::stopHook();
}

void RMLPositionTask::cleanupHook()
{
    RMLPositionTaskBase::cleanupHook();
}

ReflexxesResultValue RMLPositionTask::performOTG(base::commands::Joints &current_command)
{
    int result = rml_api->RMLPosition( *(RMLPositionInputParameters*)rml_input_parameters,
                                        (RMLPositionOutputParameters*)rml_output_parameters,
                                       *(RMLPositionFlags*)rml_flags );

    // Always feed back the new state as the current state:
    *rml_input_parameters->CurrentPositionVector     = *rml_output_parameters->NewPositionVector;
    *rml_input_parameters->CurrentVelocityVector     = *rml_output_parameters->NewVelocityVector;
    *rml_input_parameters->CurrentAccelerationVector = *rml_output_parameters->NewAccelerationVector;

    current_command.resize(motion_constraints.size());
    current_command.names = motion_constraints.names;

    for(size_t i = 0; i < motion_constraints.size(); i++){
        current_command[i].position = current_sample[i].position = rml_output_parameters->NewPositionVector->VecData[i];
        current_command[i].speed = current_sample[i].speed = rml_output_parameters->NewVelocityVector->VecData[i];
        current_command[i].acceleration = current_sample[i].acceleration = rml_output_parameters->NewAccelerationVector->VecData[i];
    }

    return (ReflexxesResultValue)result;
}

void  RMLPositionTask::setJointState(const base::JointState& state, const size_t idx)
{
    // Don't do anything here, since in position-based RML, the new position vector will always be fed back as current position
}

void RMLPositionTask::setTarget(const base::JointState& cmd, const size_t idx)
{
    if(!cmd.hasPosition()){
        LOG_ERROR("Target position of element %i is invalid: %f", idx, cmd.position);
        throw std::invalid_argument("Invalid target position");
    }

    double pos = cmd.position;

#ifdef USING_REFLEXXES_TYPE_IV  // Crop at limits if and only if POSITIONAL_LIMITS_ACTIVELY_PREVENT is selected
    if(rml_flags->PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT){
        double max = rml_input_parameters->MaxPositionVector->VecData[idx];
        double min = rml_input_parameters->MinPositionVector->VecData[idx];
        pos = std::max(std::min(max, pos), min);
    }
#endif

    ((RMLPositionInputParameters*)rml_input_parameters)->TargetPositionVector->VecData[idx] = pos;

    if(cmd.hasSpeed())
        rml_input_parameters->TargetVelocityVector->VecData[idx] = cmd.speed;
    else
        rml_input_parameters->TargetVelocityVector->VecData[idx] = 0;
}

void RMLPositionTask::setMotionConstraints(const trajectory_generation::JointMotionConstraints& constraints, const size_t idx)
{
    // Check if constraints are ok, e.g. max.speed > 0 etc
    constraints.validate();

#ifdef USING_REFLEXXES_TYPE_IV
    rml_input_parameters->MaxPositionVector->VecData[idx] = constraints.max.position;
    rml_input_parameters->MinPositionVector->VecData[idx] = constraints.min.position;
#endif
    ((RMLPositionInputParameters*)rml_input_parameters)->MaxVelocityVector->VecData[idx] = constraints.max.speed;
    rml_input_parameters->MaxAccelerationVector->VecData[idx] = constraints.max.acceleration;
    rml_input_parameters->MaxJerkVector->VecData[idx] = constraints.max_jerk;
}

const ReflexxesInputParameters& RMLPositionTask::fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out){
    RMLTask::fromRMLTypes(in, out);
    memcpy(out.max_velocity_vector.data(), ((RMLPositionInputParameters& )in).MaxVelocityVector->VecData, sizeof(double) * in.GetNumberOfDOFs());
    memcpy(out.target_position_vector.data(), ((RMLPositionInputParameters& )in).TargetPositionVector->VecData, sizeof(double) * in.GetNumberOfDOFs());
    return out;
}

const ReflexxesOutputParameters& RMLPositionTask::fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out)
{
    RMLTask::fromRMLTypes(in, out);
#ifdef USING_REFLEXXES_TYPE_IV
    out.trajectory_exceeds_target_position = ((RMLPositionOutputParameters&)in).TrajectoryExceedsTargetPosition;
#endif
    return out;
}

void RMLPositionTask::printParams()
{
    ((RMLPositionInputParameters*)rml_input_parameters)->Echo();
    ((RMLPositionOutputParameters*)rml_output_parameters)->Echo();
}
