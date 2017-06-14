#ifndef CONVERSIONS_HPP
#define CONVERSIONS_HPP

#include "trajectory_generationTypes.hpp"
#include <base/samples/RigidBodyState.hpp>
#include <reflexxes/ReflexxesAPI.h>

namespace trajectory_generation{

base::Vector3d quaternion2Euler(const base::Orientation& orientation);
base::Orientation euler2Quaternion(const base::Vector3d& euler);

void rmlTypes2InputParams(const RMLInputParameters &in,         ReflexxesInputParameters& out);
void rmlTypes2InputParams(const RMLPositionInputParameters &in, ReflexxesInputParameters& out);
void rmlTypes2InputParams(const RMLVelocityInputParameters &in, ReflexxesInputParameters& out);

void rmlTypes2OutputParams(const RMLOutputParameters &in,         ReflexxesOutputParameters& out);
void rmlTypes2OutputParams(const RMLPositionOutputParameters &in, ReflexxesOutputParameters& out);
void rmlTypes2OutputParams(const RMLVelocityOutputParameters &in, ReflexxesOutputParameters& out);

void jointState2RmlTypes(const base::samples::Joints& joint_state, const std::vector<std::string> &names, RMLInputParameters& params);
void cartesianState2RmlTypes(const base::samples::RigidBodyState& cartesian_state, RMLInputParameters& params);

void motionConstraint2RmlTypes(const MotionConstraint& constraint, const uint idx, RMLInputParameters& params);
void motionConstraint2RmlTypes(const MotionConstraint& constraint, const uint idx, RMLPositionInputParameters& params);
void motionConstraint2RmlTypes(const MotionConstraint& constraint, const uint idx, RMLVelocityInputParameters& params);

void rmlTypes2Command(const RMLPositionOutputParameters& params, base::commands::Joints& command);
void rmlTypes2Command(const RMLPositionOutputParameters& params, base::samples::RigidBodyState& command);
void rmlTypes2Command(const RMLVelocityOutputParameters& params, base::commands::Joints& command);
void rmlTypes2Command(const RMLVelocityOutputParameters& params, base::samples::RigidBodyState& command);

void target2RmlTypes(const ConstrainedJointsCmd& target, const MotionConstraints& default_constraints, const RMLFlags& flags, RMLPositionInputParameters& params);
void target2RmlTypes(const ConstrainedJointsCmd& target, const MotionConstraints& default_constraints, const RMLFlags& flags, const double cycle_time, RMLVelocityInputParameters& params);
void target2RmlTypes(const base::samples::RigidBodyState& target, const RMLFlags& flags, RMLPositionInputParameters& params);
void target2RmlTypes(const base::samples::RigidBodyState& target, const RMLFlags& flags, const double cycle_time, RMLVelocityInputParameters& params);
void target2RmlTypes(const double target_pos, const double target_vel, const uint idx, const RMLFlags& flags, RMLPositionInputParameters& params);
void target2RmlTypes(const double target_vel, const uint idx, const RMLFlags& flags, const double cycle_time, RMLVelocityInputParameters& params);

void cropTargetAtPositionLimits(RMLPositionInputParameters& params);
void fixRmlSynchronizationBug(const double cycle_time, RMLVelocityInputParameters& params);

}

#endif
