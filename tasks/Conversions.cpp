#include "Conversions.hpp"
#include <base-logging/Logging.hpp>

namespace trajectory_generation{

base::Vector3d toEuler(const base::Orientation& orientation){
    // We use yaw-pitch-roll (ZYX wrt. rotated coordinate system) convention here
    return orientation.toRotationMatrix().eulerAngles(2,1,0);
}

base::Orientation fromEuler(const base::Vector3d& euler){
    base::Quaterniond q;
    // We use yaw-pitch-roll (ZYX wrt. rotated coordinate system) convention here
    q = Eigen::AngleAxisd(euler(0), base::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler(1), base::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler(2), base::Vector3d::UnitZ());
    return q;
}

void fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out){
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

void fromRMLTypes(const RMLPositionInputParameters &in, ReflexxesInputParameters& out){
    uint n_dof = in.GetNumberOfDOFs();;
    fromRMLTypes((RMLInputParameters&)in, out);
    memcpy(out.max_velocity_vector.data(),    in.MaxVelocityVector->VecData,    sizeof(double) * n_dof);
    memcpy(out.target_position_vector.data(), in.TargetPositionVector->VecData, sizeof(double) * n_dof);
}

void fromRMLTypes(const RMLVelocityInputParameters &in, ReflexxesInputParameters& out){
    fromRMLTypes((RMLInputParameters&)in, out);
}

void fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out){
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

void fromRMLTypes(const RMLPositionOutputParameters &in, ReflexxesOutputParameters& out){
    fromRMLTypes((RMLOutputParameters&)in, out);
#ifdef USING_REFLEXXES_TYPE_IV
    out.trajectory_exceeds_target_position = in.TrajectoryExceedsTargetPosition;
#endif
}

void fromRMLTypes(const RMLVelocityOutputParameters &in, ReflexxesOutputParameters& out){
    uint n_dof = in.GetNumberOfDOFs();
    fromRMLTypes((RMLOutputParameters&)in, out);
    memcpy(out.position_values_at_target_velocity.data(), in.PositionValuesAtTargetVelocity->VecData, sizeof(double) * n_dof);
}

void toRMLTypes(const base::samples::Joints& joint_state, const std::vector<std::string> &names, RMLInputParameters& params){
    for(size_t i = 0; i < names.size(); i++){
        try{
            const base::JointState &state = joint_state.getElementByName(names[i]);
            if(!state.hasPosition()){
                LOG_ERROR("Element %s of joint state does not have a valid position entry", names[i].c_str());
                throw std::invalid_argument("Invalid joint state");
            }
            params.CurrentPositionVector->VecData[i]     = state.position;
            params.CurrentVelocityVector->VecData[i]     = 0;
            params.CurrentAccelerationVector->VecData[i] = 0;
        }
        catch(std::exception e){
            LOG_ERROR("Element %s has been configured in motion constraints, but is not available in joint state", names[i].c_str());
            throw e;
        }
    }
}

void toRMLTypes(const base::samples::RigidBodyState& cartesian_state, RMLInputParameters& params){
    if(!cartesian_state.hasValidPosition() || !cartesian_state.hasValidOrientation()){
        LOG_ERROR("Cartesian state has invalid position and/or orientation.");
        throw std::invalid_argument("Invalid cartesian state");
    }
    base::Vector3d euler = toEuler(cartesian_state.orientation);
    memcpy(params.CurrentPositionVector->VecData,   cartesian_state.position.data(), sizeof(double)*3);
    memcpy(params.CurrentPositionVector->VecData+3, euler.data(),                    sizeof(double)*3);
    memset(params.CurrentVelocityVector->VecData,   0,                               sizeof(double)*6);
    memset(params.CurrentVelocityVector->VecData,   0,                               sizeof(double)*6);
}

void toRMLTypes(const MotionConstraint& constraint, const uint idx, RMLInputParameters& params){
    constraint.validate(); // Check if constraints are ok, e.g. max.speed > 0 etc
#ifdef USING_REFLEXXES_TYPE_IV
    params.MaxPositionVector->VecData[idx] = constraint.max.position;
    params.MinPositionVector->VecData[idx] = constraint.min.position;
#endif
    params.MaxAccelerationVector->VecData[idx] = constraint.max.acceleration;
    params.MaxJerkVector->VecData[idx] = constraint.max_jerk;
}

void toRMLTypes(const MotionConstraint& constraint, const uint idx, RMLPositionInputParameters& params){
    toRMLTypes(constraint, idx, (RMLInputParameters&)params);
    params.MaxVelocityVector->VecData[idx] = constraint.max.speed;
}

void toRMLTypes(const MotionConstraint& constraint, const uint idx, RMLVelocityInputParameters& params){
    toRMLTypes(constraint, idx, (RMLInputParameters&)params);
}

void fromRMLTypes(const RMLPositionOutputParameters& params, base::commands::Joints& command){
    uint n_dof = params.GetNumberOfDOFs();
    command.resize(n_dof);
    for(size_t i = 0; i < n_dof; i++){
        command[i].position     = params.NewPositionVector->VecData[i];
        command[i].speed        = params.NewVelocityVector->VecData[i];
        command[i].acceleration = params.NewAccelerationVector->VecData[i];
    }
}

void fromRMLTypes(const RMLPositionOutputParameters& params, base::samples::RigidBodyState& command){
    base::Vector3d euler;
    memcpy(command.position.data(),         params.NewPositionVector->VecData,   sizeof(double)*3);
    memcpy(euler.data(),                    params.NewPositionVector->VecData+3, sizeof(double)*3);
    memcpy(command.velocity.data(),         params.NewVelocityVector->VecData,   sizeof(double)*3);
    memcpy(command.angular_velocity.data(), params.NewVelocityVector->VecData+3, sizeof(double)*3);
    command.orientation = fromEuler(euler);
}

void fromRMLTypes(const RMLVelocityOutputParameters& params, base::commands::Joints& command){
    uint n_dof = params.GetNumberOfDOFs();
    command.resize(n_dof);
    for(size_t i = 0; i < n_dof; i++){
        command[i].speed           = params.NewVelocityVector->VecData[i];
        command[i].acceleration    = params.NewAccelerationVector->VecData[i];
    }
}

void fromRMLTypes(const RMLVelocityOutputParameters& params, base::samples::RigidBodyState& command){
    memcpy(command.velocity.data(),         params.NewVelocityVector->VecData,   sizeof(double)*3);
    memcpy(command.angular_velocity.data(), params.NewVelocityVector->VecData+3, sizeof(double)*3);
}

void toRMLTypes(const ConstrainedJointsCmd& target, RMLPositionInputParameters& params){

}

void toRMLTypes(const ConstrainedJointsCmd& target, RMLVelocityInputParameters& params){

}

void toRMLTypes(const base::samples::RigidBodyState& target, RMLPositionInputParameters& params){

}

void toRMLTypes(const base::samples::RigidBodyState& target, RMLVelocityInputParameters& params){

}

}
