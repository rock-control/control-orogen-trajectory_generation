/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLPositionTrajectoryTask.hpp"

using namespace trajectory_generation;

RMLPositionTrajectoryTask::RMLPositionTrajectoryTask(std::string const& name)
    : RMLPositionTrajectoryTaskBase(name)
{
}

RMLPositionTrajectoryTask::RMLPositionTrajectoryTask(std::string const& name, RTT::ExecutionEngine* engine)
    : RMLPositionTrajectoryTaskBase(name, engine)
{
}

RMLPositionTrajectoryTask::~RMLPositionTrajectoryTask()
{
}

bool RMLPositionTrajectoryTask::configureHook()
{
    if (! RMLPositionTrajectoryTaskBase::configureHook())
        return false;
    return true;
}
bool RMLPositionTrajectoryTask::startHook()
{
    if (! RMLPositionTrajectoryTaskBase::startHook())
        return false;
    timestep = 0;
    return true;
}
void RMLPositionTrajectoryTask::updateHook()
{
    RMLPositionTrajectoryTaskBase::updateHook();

    if(state() == NO_JOINT_STATE)
        return;

    ConstrainedJointsTrajectory new_traj;
    if(_trajectory.readNewest(new_traj) == RTT::NewData)
        handleNewTrajectory(new_traj);
    else{
        if(timestep < trajectory.getTimeSteps()){
            if(rml_result_value == RML_FINAL_STATE_REACHED)
                setNextPoint(timestep++);
        }
        else{
            if(rml_result_value == RML_FINAL_STATE_REACHED){
                if(state() != REACHED_FINAL_TRAJ_POINT)
                    state(REACHED_FINAL_TRAJ_POINT);

                // Reset motion constraints
                motion_constraints = _motion_constraints.get();
                for(size_t i = 0; i < motion_constraints.size(); i++)
                    setMotionConstraints(motion_constraints[i], i);
            }
        }
    }
}
void RMLPositionTrajectoryTask::errorHook()
{
    RMLPositionTrajectoryTaskBase::errorHook();
}
void RMLPositionTrajectoryTask::stopHook()
{
    RMLPositionTrajectoryTaskBase::stopHook();
}
void RMLPositionTrajectoryTask::cleanupHook()
{
    RMLPositionTrajectoryTaskBase::cleanupHook();
}

void RMLPositionTrajectoryTask::setNextPoint(size_t t){

    for(size_t i = 0; i < motion_constraints.size(); i++)
        setMotionConstraints(trajectory.motion_constraints[t][i], i);

    trajectory.getJointsAtTimeStep(t, target);
    handleNewTarget(target);
}

void RMLPositionTrajectoryTask::handleNewTrajectory(const trajectory_generation::ConstrainedJointsTrajectory& new_traj){

    if(new_traj.getTimeSteps() == 0)
        return;

    if(state() != RUNNING)
        state(RUNNING);

    trajectory = new_traj;
    timestep = 0;

    computeSpeeds(joint_state, trajectory);
    setNextPoint(timestep++);
}

void RMLPositionTrajectoryTask::computeSpeeds(const base::samples::Joints &joint_state,
                                              trajectory_generation::ConstrainedJointsTrajectory& trajectory)
{
    double prev, next, cur;

    trajectory.motion_constraints.resize(trajectory.getTimeSteps());

    for(size_t timestep = 0; timestep < trajectory.getTimeSteps(); timestep++){

        double max_speed = 0;
        trajectory.motion_constraints[timestep].resize(trajectory.size());
        trajectory.motion_constraints[timestep].names = trajectory.names;

        for(size_t joint_idx = 0; joint_idx < trajectory.size(); joint_idx++){

            const base::JointState &state = joint_state.getElementByName(trajectory.names[joint_idx]);
            const trajectory_generation::JointMotionConstraints& constraint = motion_constraints.getElementByName(trajectory.names[joint_idx]);

            trajectory.motion_constraints[timestep][joint_idx] = constraint;

            // The current point of the trajectory
            cur = trajectory[joint_idx][timestep].position;

            if(timestep == 0){
                // The previous point is set to the current interpolator position (if available) or to
                // the current joint state for the first timestep
                if(!command.empty()){
                    const base::JointState &cmd = command.getElementByName(trajectory.names[joint_idx]);
                    prev = cmd.position;
                }
                else
                    prev = state.position;
            }
            else
                prev = trajectory[joint_idx][timestep-1].position;

            // Next point is set to the previous point for the last timestep, so that
            // target speed will be zero
            if(timestep == trajectory.getTimeSteps()-1) // Last trajectory point
                next = prev;
            else
                next = trajectory[joint_idx][timestep+1].position;

            double speed;
            speed = (next - prev)/2;

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
            if(fabs(trajectory[joint_idx][timestep].speed) > 1e-10)
                trajectory.motion_constraints[timestep][joint_idx].max.speed = fabs(trajectory[joint_idx][timestep].speed);
        }
    }
}
