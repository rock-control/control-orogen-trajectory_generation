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
    timestep = 0;
    return true;
}

void RMLPositionTask::updateHook()
{
    RMLPositionTaskBase::updateHook();

    base::JointsTrajectory new_traj;
    if(_trajectory.readNewest(new_traj) == RTT::NewData){
        if(state() != RUNNING)
            state(RUNNING);

        trajectory = new_traj;
        timestep = 0;
        computeSpeeds(joint_state, motion_constraints, trajectory);
    }

    if(timestep < trajectory.getTimeSteps()){
        if(state() == REACHED || state() == RUNNING){
            trajectory.getJointsAtTimeStep(timestep, target);
            handleNewTarget(target);
            timestep++;
        }
    }
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

void RMLPositionTask::computeSpeeds(const base::samples::Joints &joint_state,
                                    const trajectory_generation::JointsMotionConstraints& motion_constraints,
                                    base::JointsTrajectory& trajectory)
{
    double prev, next, cur;
    for(size_t timestep = 0; timestep < trajectory.getTimeSteps(); timestep++){

        double max_speed = 0;
        for(size_t joint_idx = 0; joint_idx < trajectory.size(); joint_idx++){

            const base::JointState &state = joint_state.getElementByName(trajectory.names[joint_idx]);

            cur = trajectory[joint_idx][timestep].position;

            if(timestep == 0) // First trajectory point
                prev = state.position;
            else
                prev = trajectory[joint_idx][timestep-1].position;

            if(timestep == trajectory.getTimeSteps()-1) // Last trajectory point
                next = prev;
            else
                next = trajectory[joint_idx][timestep+1].position;

            double speed = (next - prev)/2;

            //Check for inflection points. Target velocity should always be zero at these points
            if( (next > cur && prev > cur) ||
                    (next < cur && prev < cur) ||
                    (next == cur) )
                speed = 0;

            if(fabs(speed) > max_speed)
                max_speed = fabs(speed);

            trajectory[joint_idx][timestep].speed = speed;
        }

        // Normalize speed to 1.0 and scale to target speed
        for(size_t joint_idx = 0; joint_idx < trajectory.size(); joint_idx++){

            const trajectory_generation::JointMotionConstraints constraints = motion_constraints.getElementByName(trajectory.names[joint_idx]);
            double scale = max_speed < 1e-10 ? 0 : constraints.max.speed / max_speed;

            trajectory[joint_idx][timestep].speed *= scale;
        }
    }
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
