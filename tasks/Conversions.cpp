#include "Conversions.hpp"
#include <base-logging/Logging.hpp>

using namespace joint_control_base;

namespace trajectory_generation{

base::Vector3d quaternion2Euler(const base::Orientation& orientation){
    // We use yaw-pitch-roll (ZYX wrt. rotated coordinate system) convention here
    return orientation.toRotationMatrix().eulerAngles(2,1,0);
}

base::Orientation euler2Quaternion(const base::Vector3d& euler){
    base::Quaterniond q;
    // We use yaw-pitch-roll (ZYX wrt. rotated coordinate system) convention here
    q = Eigen::AngleAxisd(euler(0), base::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(euler(1), base::Vector3d::UnitY()) *
            Eigen::AngleAxisd(euler(2), base::Vector3d::UnitX());
    return q;
}

void rmlTypes2InputParams(const RMLInputParameters &in, ReflexxesInputParameters& out){
    uint n_dof = in.GetNumberOfDOFs();;
    memcpy(out.selection_vector.data(),                in.SelectionVector->VecData,           sizeof(bool)   * n_dof);
    memcpy(out.current_position_vector.data(),             in.CurrentPositionVector->VecData,     sizeof(double) * n_dof);
    memcpy(out.current_velocity_vector.data(),         in.CurrentVelocityVector->VecData,     sizeof(double) * n_dof);
    memcpy(out.current_acceleration_vector.data(), in.CurrentAccelerationVector->VecData, sizeof(double) * n_dof);
    memcpy(out.max_acceleration_vector.data(),             in.MaxAccelerationVector->VecData,     sizeof(double) * n_dof);
    memcpy(out.max_jerk_vector.data(),             in.MaxJerkVector->VecData,             sizeof(double) * n_dof);
    memcpy(out.target_velocity_vector.data(),          in.TargetVelocityVector->VecData,      sizeof(double) * n_dof);
    out.min_synchronization_time = in.MinimumSynchronizationTime;
#ifdef USING_REFLEXXES_TYPE_IV
    memcpy(out.max_position_vector.data(), in.MaxPositionVector->VecData, sizeof(double) * n_dof);
    memcpy(out.min_position_vector.data(), in.MinPositionVector->VecData, sizeof(double) * n_dof);
    out.override_value = in.OverrideValue;
#endif
}

void rmlTypes2InputParams(const RMLPositionInputParameters &in, ReflexxesInputParameters& out){
    uint n_dof = in.GetNumberOfDOFs();;
    rmlTypes2InputParams((RMLInputParameters&)in, out);
    memcpy(out.max_velocity_vector.data(),    in.MaxVelocityVector->VecData,    sizeof(double) * n_dof);
    memcpy(out.target_position_vector.data(), in.TargetPositionVector->VecData, sizeof(double) * n_dof);
}

void rmlTypes2InputParams(const RMLVelocityInputParameters &in, ReflexxesInputParameters& out){
    rmlTypes2InputParams((RMLInputParameters&)in, out);
}

void rmlTypes2OutputParams(const RMLOutputParameters &in, ReflexxesOutputParameters& out){
    uint n_dof = in.GetNumberOfDOFs();
    memcpy(out.new_position_vector.data(),     in.NewPositionVector->VecData,     sizeof(double) * n_dof);
    memcpy(out.new_velocity_vector.data(),     in.NewVelocityVector->VecData,     sizeof(double) * n_dof);
    memcpy(out.new_acceleration_vector.data(), in.NewAccelerationVector->VecData, sizeof(double) * n_dof);
    memcpy(out.execution_times.data(),         in.ExecutionTimes->VecData,        sizeof(double) * n_dof);
    out.a_new_calculation_was_performed = in.ANewCalculationWasPerformed;
    out.trajectory_is_phase_synchronized = in.TrajectoryIsPhaseSynchronized;
    out.dof_with_greatest_execution_time = in.DOFWithTheGreatestExecutionTime;
    out.synchronization_time = in.SynchronizationTime;
#ifdef USING_REFLEXXES_TYPE_IV
    out.override_filter_is_active = in.OverrideFilterIsActive;
    out.current_override_value = in.CurrentOverrideValue;
#endif
}

void rmlTypes2OutputParams(const RMLPositionOutputParameters &in, ReflexxesOutputParameters& out){
    rmlTypes2OutputParams((RMLOutputParameters&)in, out);
#ifdef USING_REFLEXXES_TYPE_IV
    out.trajectory_exceeds_target_position = in.TrajectoryExceedsTargetPosition;
#endif
}

void rmlTypes2OutputParams(const RMLVelocityOutputParameters &in, ReflexxesOutputParameters& out){
    uint n_dof = in.GetNumberOfDOFs();
    rmlTypes2OutputParams((RMLOutputParameters&)in, out);
    memcpy(out.position_values_at_target_velocity.data(), in.PositionValuesAtTargetVelocity->VecData, sizeof(double) * n_dof);
}

void jointState2RmlTypes(const base::samples::Joints& joint_state, const std::vector<std::string> &names, const RMLFlags& flags, RMLInputParameters& params){
    for(size_t i = 0; i < names.size(); i++){
        try{
            const base::JointState &state = joint_state.getElementByName(names[i]);
            if(!state.hasPosition()){
                LOG_ERROR("Element %s of joint state does not have a valid position entry", names[i].c_str());
                throw std::invalid_argument("Invalid joint state");
            }
#ifdef USING_REFLEXXES_TYPE_IV
            if(flags.PositionalLimitsBehavior == POSITIONAL_LIMITS_ACTIVELY_PREVENT ||
               flags.PositionalLimitsBehavior == POSITIONAL_LIMITS_ERROR_MSG_ONLY){
                if(state.position > params.MaxPositionVector->VecData[i] ||
                   state.position < params.MinPositionVector->VecData[i]){
                    LOG_ERROR("Position of joint %s is outside joint limits, Upper: %f, Lower: %f, Actual: %f",
                              names[i].c_str(), params.MaxPositionVector->VecData[i], params.MinPositionVector->VecData[i], state.position);
                    throw std::invalid_argument("Invalid joint state");
                }
            }
#endif
            params.CurrentPositionVector->VecData[i]     = state.position;
            params.CurrentVelocityVector->VecData[i]     = 0;
            params.CurrentAccelerationVector->VecData[i] = 0;
        }
        catch(std::runtime_error e){
            LOG_ERROR("Element %s has been configured in motion constraints, but is not available in joint state", names[i].c_str());
            throw e;
        }
    }
}

void rmlTypes2JointState(const RMLInputParameters& params, base::samples::Joints& joint_state){
    joint_state.resize(params.GetNumberOfDOFs());
    for(uint i = 0; i < params.GetNumberOfDOFs(); i++){
        joint_state[i].position     = params.CurrentPositionVector->VecData[i];
        joint_state[i].speed        = params.CurrentVelocityVector->VecData[i];
        joint_state[i].acceleration = params.CurrentAccelerationVector->VecData[i];
    }
}

void cartesianState2RmlTypes(const base::samples::RigidBodyStateSE3& cartesian_state, RMLInputParameters& params){
    if(!cartesian_state.hasValidPose()){
        LOG_ERROR("Cartesian state has invalid position and/or orientation.");
        throw std::invalid_argument("Invalid cartesian state");
    }
    base::Vector3d euler = quaternion2Euler(cartesian_state.pose.orientation);
    memcpy(params.CurrentPositionVector->VecData,   cartesian_state.pose.position.data(), sizeof(double)*3);
    memcpy(params.CurrentPositionVector->VecData+3, euler.data(),                    sizeof(double)*3);
    memset(params.CurrentVelocityVector->VecData,   0,                               sizeof(double)*6);
    memset(params.CurrentVelocityVector->VecData,   0,                               sizeof(double)*6);
}

void rmlTypes2CartesianState(const RMLInputParameters& params, base::samples::RigidBodyStateSE3& cartesian_state){
    base::Vector3d euler;
    memcpy(cartesian_state.pose.position.data(),         params.CurrentPositionVector->VecData,   sizeof(double)*3);
    memcpy(euler.data(),                            params.CurrentPositionVector->VecData+3, sizeof(double)*3);
    memcpy(cartesian_state.twist.linear.data(),         params.CurrentVelocityVector->VecData,   sizeof(double)*3);
    memcpy(cartesian_state.twist.angular.data(), params.CurrentVelocityVector->VecData+3, sizeof(double)*3);
    cartesian_state.pose.orientation = euler2Quaternion(euler);
}

void motionConstraint2RmlTypes(const MotionConstraint& constraint, const uint idx, RMLInputParameters& params){
    // Check if constraints are ok, e.g. speed > 0 etc.
    constraint.validateVelocityLimit();
    constraint.validateAccelerationLimit();
    constraint.validateJerkLimit();
#ifdef USING_REFLEXXES_TYPE_IV
    constraint.validatePositionLimits();
    params.MaxPositionVector->VecData[idx] = constraint.max.position;
    params.MinPositionVector->VecData[idx] = constraint.min.position;
#endif
    params.MaxAccelerationVector->VecData[idx] = constraint.max.acceleration;
    params.MaxJerkVector->VecData[idx] = constraint.max_jerk;
}

void motionConstraint2RmlTypes(const MotionConstraint& constraint, const uint idx, RMLPositionInputParameters& params){
    motionConstraint2RmlTypes(constraint, idx, (RMLInputParameters&)params);
    params.MaxVelocityVector->VecData[idx] = constraint.max.speed;
}

void motionConstraint2RmlTypes(const MotionConstraint& constraint, const uint idx, RMLVelocityInputParameters& params){
    motionConstraint2RmlTypes(constraint, idx, (RMLInputParameters&)params);
}

void rmlTypes2Command(const RMLPositionOutputParameters& params, base::commands::Joints& command){
    uint n_dof = params.GetNumberOfDOFs();
    command.resize(n_dof);
    for(size_t i = 0; i < n_dof; i++){
        command[i].position     = params.NewPositionVector->VecData[i];
        command[i].speed        = params.NewVelocityVector->VecData[i];
        command[i].acceleration = params.NewAccelerationVector->VecData[i];
    }
}

void rmlTypes2Command(const RMLPositionOutputParameters& params, base::samples::RigidBodyStateSE3& command){
    base::Vector3d euler;
    memcpy(command.pose.position.data(),         params.NewPositionVector->VecData,   sizeof(double)*3);
    memcpy(euler.data(),                    params.NewPositionVector->VecData+3, sizeof(double)*3);
    memcpy(command.twist.linear.data(),         params.NewVelocityVector->VecData,   sizeof(double)*3);
    memcpy(command.twist.angular.data(), params.NewVelocityVector->VecData+3, sizeof(double)*3);
    command.pose.orientation = euler2Quaternion(euler);
}

void rmlTypes2Command(const RMLVelocityOutputParameters& params, base::commands::Joints& command){
    uint n_dof = params.GetNumberOfDOFs();
    command.resize(n_dof);
    for(size_t i = 0; i < n_dof; i++){
        command[i].speed           = params.NewVelocityVector->VecData[i];
        command[i].acceleration    = params.NewAccelerationVector->VecData[i];
    }
}

void rmlTypes2Command(const RMLVelocityOutputParameters& params, base::samples::RigidBodyStateSE3& command){
    memcpy(command.twist.linear.data(),         params.NewVelocityVector->VecData,       sizeof(double)*3);
    memcpy(command.twist.angular.data(),        params.NewVelocityVector->VecData+3,     sizeof(double)*3);
    memcpy(command.acceleration.linear.data(),  params.NewAccelerationVector->VecData,   sizeof(double)*3);
    memcpy(command.acceleration.angular.data(), params.NewAccelerationVector->VecData+3, sizeof(double)*3);
}

void target2RmlTypes(const ConstrainedJointsCmd& target, const MotionConstraints& default_constraints, RMLPositionInputParameters& params){
    // Set selection vector to false. Select individual elements below
    memset(params.SelectionVector->VecData, false, params.GetNumberOfDOFs());
    for(size_t i = 0; i < target.size(); i++){
        try{
            size_t idx = default_constraints.mapNameToIndex(target.names[i]);
            target2RmlTypes(target[i].position, target[i].speed, idx, params);
            if(!target.motion_constraints.empty()){
                MotionConstraint constraint = target.motion_constraints[i];
                constraint.applyDefaultIfUnset(default_constraints[idx]);
                motionConstraint2RmlTypes(constraint, idx, params);
            }
        }
        catch(std::runtime_error e){
            LOG_ERROR("Joint '%s' is in target vector but has not been configured in motion constraints", target.names[i].c_str());
            throw e;
        }
    }
}

void target2RmlTypes(const ConstrainedJointsCmd& target, const MotionConstraints& default_constraints, RMLVelocityInputParameters& params){
    // Set selection vector to false. Select individual elements below
    memset(params.SelectionVector->VecData, false, params.GetNumberOfDOFs());
    for(size_t i = 0; i < target.size(); i++){
        try{
            size_t idx = default_constraints.mapNameToIndex(target.names[i]);
            target2RmlTypes(target[i].speed, idx, params);
            if(!target.motion_constraints.empty()){
                MotionConstraint constraint = target.motion_constraints[i];
                constraint.applyDefaultIfUnset(default_constraints[idx]);
                motionConstraint2RmlTypes(constraint, idx, params);
            }
        }
        catch(std::runtime_error e){
            LOG_ERROR("Joint '%s' is in target vector but has not been configured in motion constraints", target.names[i].c_str());
            throw e;
        }
    }
}

void target2RmlTypes(const base::samples::RigidBodyStateSE3& target, RMLPositionInputParameters& params){
    base::Vector3d euler = quaternion2Euler(target.pose.orientation);
    for(int i = 0; i < 3; i++)
        target2RmlTypes(target.pose.position(i), target.twist.linear(i), i, params);
    for(int i = 0; i < 3; i++)
        target2RmlTypes(euler(i), target.twist.angular(i), i+3, params);
}

void target2RmlTypes(const base::samples::RigidBodyStateSE3& target, RMLVelocityInputParameters& params){
    for(int i = 0; i < 3; i++)
        target2RmlTypes(target.twist.linear(i), i, params);
    for(int i = 0; i < 3; i++)
        target2RmlTypes(target.twist.angular(i), i+3, params);
}

void target2RmlTypes(const base::samples::RigidBodyState& target, RMLPositionInputParameters& params){
    base::Vector3d euler = quaternion2Euler(target.orientation);
    for(int i = 0; i < 3; i++)
        target2RmlTypes(target.position(i), target.velocity(i), i, params);
    for(int i = 0; i < 3; i++)
        target2RmlTypes(euler(i), target.angular_velocity(i), i+3, params);
}

void target2RmlTypes(const base::samples::RigidBodyState& target, RMLVelocityInputParameters& params){
    for(int i = 0; i < 3; i++)
        target2RmlTypes(target.velocity(i), i, params);
    for(int i = 0; i < 3; i++)
        target2RmlTypes(target.angular_velocity(i), i+3, params);
}

void target2RmlTypes(const double target_pos, const double target_vel, const uint idx, RMLPositionInputParameters& params){
    if(base::isNaN(target_pos)){
        LOG_ERROR("Element %i of target has no valid position!", idx);
        throw std::invalid_argument("Invalid target");
    }
    params.SelectionVector->VecData[idx]      = true;
    params.TargetPositionVector->VecData[idx] = target_pos;
    params.TargetVelocityVector->VecData[idx] = 0;
    if(!base::isNaN(target_vel))
        params.TargetVelocityVector->VecData[idx] = target_vel;
}

void target2RmlTypes(const double target_vel, const uint idx, RMLVelocityInputParameters& params){
    if(base::isNaN(target_vel)){
        LOG_ERROR("Element %i of target has no valid velocity!", idx);
        throw std::invalid_argument("Invalid target");
    }
    params.SelectionVector->VecData[idx]      = true;
    params.TargetVelocityVector->VecData[idx] = target_vel;
}

void cropTargetAtPositionLimits(RMLPositionInputParameters& params){
#ifdef USING_REFLEXXES_TYPE_IV
    for(uint i = 0; i < params.GetNumberOfDOFs(); i++){
        double pos = params.TargetPositionVector->VecData[i];
        double max = params.MaxPositionVector->VecData[i];
        double min = params.MinPositionVector->VecData[i];
        params.TargetPositionVector->VecData[i] = std::max(std::min(max, pos), min);
    }
#endif
}

void fixRmlSynchronizationBug(const double cycle_time, RMLVelocityInputParameters& params){
#ifdef USING_REFLEXXES_TYPE_IV
    for(uint i = 0; i < params.GetNumberOfDOFs(); i++){
        double vel     = params.TargetVelocityVector->VecData[i];
        double cur_pos = params.CurrentPositionVector->VecData[i];
        double max_pos = params.MaxPositionVector->VecData[i];
        double min_pos = params.MinPositionVector->VecData[i];
        if( (vel*cycle_time + cur_pos > max_pos) || (vel*cycle_time + cur_pos < min_pos) )
            params.TargetVelocityVector->VecData[i] = 0;
    }
#endif
}

}
