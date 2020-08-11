#include "Conversions.hpp"
#include <base-logging/Logging.hpp>
#include <unsupported/Eigen/src/MatrixFunctions/MatrixExponential.h>

using namespace joint_control_base;

namespace trajectory_generation{


base::Vector3d inverseSkew(const base::Matrix3d& mat){
    return base::Vector3d(mat(2,1),mat(0,2),mat(1,0));
}

base::Matrix3d makeSkew(const base::Vector3d& vect){
    base::Matrix3d mat;
    mat<<0,-vect[2],vect[1],
         vect[2],0,-vect[0],
         -vect[1],vect[0],0;
    return mat;
}

void logMap(const base::Matrix3d& rot_mat, base::Vector3d& u, double& phi){

     // If orientation is approx. Identity, the rotaton axis is undefined, return zero vector in this case
    if(rot_mat.diagonal().isApprox(base::Vector3d(1,1,1))){
        u = base::Vector3d::Zero();
        phi = 0;
    }
    else{
        phi = acos((rot_mat.trace()-1.0)/2.0); //Rotation angle

        // If trace is -1, phi is approx. Pi, use approximaton in this case
        if((rot_mat.trace()+1) < 1e-9){
            if((rot_mat(2,2)+1) > 1e-9)
                u = (1.0/(sqrt(2*(1+rot_mat(2,2))))) * base::Vector3d(rot_mat(0,2), rot_mat(1,2), 1+rot_mat(2,2));
            else if((rot_mat(1,1)+1) > 1e-9)
                u = 1/(sqrt(2*(1+rot_mat(1,1)))) * base::Vector3d(rot_mat(0,1), 1+rot_mat(1,1), rot_mat(2,1));
            else if((rot_mat(0,0)+1))
                u = 1/(sqrt(2*(1+rot_mat(0,0)))) * base::Vector3d(1+rot_mat(0,0), 1+rot_mat(1,0), rot_mat(2,0));
            else
                throw std::runtime_error("Unable to compute logarithmic map for given matrix");
        }
        else // "normal" case
            u = inverseSkew(rot_mat-rot_mat.transpose())/(2*sin(phi));
    }
}

base::Matrix3d expMap(const base::Vector3d& rot_vect){
    return makeSkew(rot_vect).exp();
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
    base::Vector3d u;
    double phi;
    logMap(cartesian_state.pose.orientation.toRotationMatrix(), u, phi);
    base::Vector3d rot_vect = phi*u;
    memcpy(params.CurrentPositionVector->VecData,   cartesian_state.pose.position.data(), sizeof(double)*3);
    memcpy(params.CurrentPositionVector->VecData+3, rot_vect.data(),                    sizeof(double)*3);
    memset(params.CurrentVelocityVector->VecData,   0,                               sizeof(double)*6);
    memset(params.CurrentVelocityVector->VecData,   0,                               sizeof(double)*6);
}

void rmlTypes2CartesianState(const RMLInputParameters& params, base::samples::RigidBodyStateSE3& cartesian_state){
    base::Vector3d rot_vect;
    memcpy(cartesian_state.pose.position.data(),         params.CurrentPositionVector->VecData,   sizeof(double)*3);
    memcpy(rot_vect.data(),                            params.CurrentPositionVector->VecData+3, sizeof(double)*3);
    cartesian_state.pose.orientation = Eigen::Quaterniond(expMap(rot_vect));
    memcpy(cartesian_state.twist.linear.data(),         params.CurrentVelocityVector->VecData,   sizeof(double)*3);
    memcpy(cartesian_state.twist.angular.data(), params.CurrentVelocityVector->VecData+3, sizeof(double)*3);
    memcpy(cartesian_state.acceleration.linear.data(),         params.CurrentAccelerationVector->VecData,   sizeof(double)*3);
    memcpy(cartesian_state.acceleration.angular.data(), params.CurrentAccelerationVector->VecData+3, sizeof(double)*3);
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
    base::Vector3d rot_vect;
    memcpy(command.pose.position.data(),         params.NewPositionVector->VecData,   sizeof(double)*3);
    memcpy(rot_vect.data(),                    params.NewPositionVector->VecData+3, sizeof(double)*3);
    command.pose.orientation = Eigen::Quaterniond(expMap(rot_vect));
    memcpy(command.twist.linear.data(),         params.NewVelocityVector->VecData,   sizeof(double)*3);
    memcpy(command.twist.angular.data(), params.NewVelocityVector->VecData+3, sizeof(double)*3);
    memcpy(command.acceleration.linear.data(),         params.NewAccelerationVector->VecData,   sizeof(double)*3);
    memcpy(command.acceleration.angular.data(), params.NewAccelerationVector->VecData+3, sizeof(double)*3);
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

void target2RmlTypes(const base::samples::RigidBodyState& target, RMLPositionInputParameters& params){
    base::Vector3d u;
    double phi;
    logMap(target.orientation.toRotationMatrix(), u, phi);
    base::Vector3d option_one, option_two, new_target, current_state;
    option_one = u*phi;
    option_two = -u*(2*M_PI-phi);
    current_state = base::Vector3d(params.CurrentPositionVector->VecData[3],
                                    params.CurrentPositionVector->VecData[4],
                                    params.CurrentPositionVector->VecData[5]);

     if((current_state - option_one).norm() < (current_state - option_two).norm())
        new_target = option_one;
    else
        new_target = option_two;
    for(int i = 0; i < 3; i++)
        target2RmlTypes(target.position(i), target.velocity(i), i, params);
    for(int i = 0; i < 3; i++)
        target2RmlTypes(new_target(i), target.angular_velocity(i), i+3, params);
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
