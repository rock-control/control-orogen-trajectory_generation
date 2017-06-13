#ifndef CONVERSIONS_HPP
#define CONVERSIONS_HPP

#include "trajectory_generationTypes.hpp"
#include <base/samples/RigidBodyState.hpp>
#include <reflexxes/RMLPositionInputParameters.h>
#include <reflexxes/RMLPositionOutputParameters.h>
#include <reflexxes/RMLVelocityInputParameters.h>
#include <reflexxes/RMLVelocityOutputParameters.h>
#include <reflexxes/RMLFlags.h>

namespace trajectory_generation{

base::Vector3d toEuler(const base::Orientation& orientation);
base::Orientation fromEuler(const base::Vector3d& euler);

void fromRMLTypes(const RMLInputParameters &in,         ReflexxesInputParameters& out);
void fromRMLTypes(const RMLPositionInputParameters &in, ReflexxesInputParameters& out);
void fromRMLTypes(const RMLVelocityInputParameters &in, ReflexxesInputParameters& out);

void fromRMLTypes(const RMLOutputParameters &in,         ReflexxesOutputParameters& out);
void fromRMLTypes(const RMLPositionOutputParameters &in, ReflexxesOutputParameters& out);
void fromRMLTypes(const RMLVelocityOutputParameters &in, ReflexxesOutputParameters& out);

void toRMLTypes(const base::samples::Joints& joint_state,             const std::vector<std::string> &names, RMLInputParameters& params);
void toRMLTypes(const base::samples::RigidBodyState& cartesian_state, RMLInputParameters& params);

void toRMLTypes(const MotionConstraint& constraint, const uint idx, RMLInputParameters& params);
void toRMLTypes(const MotionConstraint& constraint, const uint idx, RMLPositionInputParameters& params);
void toRMLTypes(const MotionConstraint& constraint, const uint idx, RMLVelocityInputParameters& params);

void fromRMLTypes(const RMLPositionOutputParameters& params, base::commands::Joints& command);
void fromRMLTypes(const RMLPositionOutputParameters& params, base::samples::RigidBodyState& command);
void fromRMLTypes(const RMLVelocityOutputParameters& params, base::commands::Joints& command);
void fromRMLTypes(const RMLVelocityOutputParameters& params, base::samples::RigidBodyState& command);

void toRMLTypes(const ConstrainedJointsCmd& target, RMLPositionInputParameters& params);
void toRMLTypes(const ConstrainedJointsCmd& target, RMLVelocityInputParameters& params);
void toRMLTypes(const base::samples::RigidBodyState& target, RMLPositionInputParameters& params);
void toRMLTypes(const base::samples::RigidBodyState& target, RMLVelocityInputParameters& params);

}

#endif
