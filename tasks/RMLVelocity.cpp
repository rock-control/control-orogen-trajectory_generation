/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLVelocity.hpp"

using namespace trajectory_generation;

bool RMLVelocity::configureHook(){

    rml_flags = new RMLVelocityFlags();
    rml_input_parameters = new RMLVelocityInputParameters(_motion_constraints.get().size());
    rml_output_parameters = new RMLVelocityOutputParameters(_motion_constraints.get().size());

    no_reference_timeout = _no_reference_timeout.get();
    if(base::isNaN(no_reference_timeout))
        no_reference_timeout = base::infinity<double>();
    convert_to_position = _convert_to_position.get();

    if (! RMLVelocityBase::configureHook())
        return false;
    return true;
}

void RMLVelocity::updateMotionConstraints(const MotionConstraint& constraint,
                                          const size_t idx,
                                          RMLInputParameters* new_input_parameters){

    RMLVelocityInputParameters* params = (RMLVelocityInputParameters*)new_input_parameters;

    constraint.validate(); // Check if constraints are ok, e.g. max.speed > 0 etc

#ifdef USING_REFLEXXES_TYPE_IV
    params->MaxPositionVector->VecData[idx] = constraint.max.position;
    params->MinPositionVector->VecData[idx] = constraint.min.position;
#endif
    params->MaxAccelerationVector->VecData[idx] = constraint.max.acceleration;
    params->MaxJerkVector->VecData[idx] = constraint.max_jerk;
}

void RMLVelocity::updateTarget(const TargetData& target_vector,
                               RMLInputParameters* new_input_parameters){

    RMLVelocityInputParameters* params = (RMLVelocityInputParameters*)new_input_parameters;

    int n_dof = params->NumberOfDOFs;
    memcpy(params->TargetVelocityVector->VecData, target_vector.velocity.data(),         sizeof(double) * n_dof);
    memcpy(params->SelectionVector->VecData,      target_vector.selection_vector.data(), sizeof(bool) * n_dof);

#ifdef USING_REFLEXXES_TYPE_IV
    for(size_t i = 0; i < n_dof; i++){
        // If a joint is at a position limit, the target velocity is non-zero and pointing in direction of the limit, the sychronization time is
        // computed by RML as if the constrained joint could move freely. This might lead to incorrect synchronization time for all other joints.
        // Workaround: Set the target velocity to zero if (and only if) POSITIONAL_LIMITS_ACTIVELY_PREVENT is selected
        if(rml_flags->PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT){
            double cur_pos = params->CurrentPositionVector->VecData[i];
            double target_vel = params->TargetVelocityVector->VecData[i];
            double max_pos = params->MaxPositionVector->VecData[i];
            double min_pos = params->MinPositionVector->VecData[i];

            if( (target_vel*cycle_time + cur_pos > max_pos) || (target_vel*cycle_time + cur_pos < min_pos) )
                params->TargetVelocityVector->VecData[i] = 0;
        }
    }
#endif

    for(size_t i = 0; i < target_vector.constraints.size(); i++){
        MotionConstraint constraint = target_vector.constraints[i];  // Get new motion constraint for target i
        constraint.applyDefaultIfUnset(motion_constraints[i]);    // Use default entry if new constraints entries are unset
        updateMotionConstraints(constraint, i, params);
    }
}

ReflexxesResultValue RMLVelocity::performOTG(RMLInputParameters* new_input_parameters,
                                             RMLOutputParameters* new_output_parameters,
                                             RMLFlags *rml_flags){

    int result = rml_api->RMLVelocity(*(RMLVelocityInputParameters*)new_input_parameters,
                                       (RMLVelocityOutputParameters*)new_output_parameters,
                                      *(RMLVelocityFlags*)rml_flags );

    // Always feed back the new state as the current state. This means that the current robot position
    // is completely ignored. However, on a real robot, using the current position as input in RML will NOT work!
    *new_input_parameters->CurrentPositionVector     = *new_output_parameters->NewPositionVector;
    *new_input_parameters->CurrentVelocityVector     = *new_output_parameters->NewVelocityVector;
    *new_input_parameters->CurrentAccelerationVector = *new_output_parameters->NewAccelerationVector;

    return (ReflexxesResultValue)result;
}

void RMLVelocity::printParams(const RMLInputParameters& in, const RMLOutputParameters& out){
    ((RMLVelocityInputParameters&  )in).Echo();
    ((RMLVelocityOutputParameters& )out).Echo();
}

const ReflexxesInputParameters& RMLVelocity::fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out){
    out.fromRMLTypes((RMLVelocityInputParameters&)in);
    return out;
}

const ReflexxesOutputParameters& RMLVelocity::fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out){
    out.fromRMLTypes((RMLVelocityOutputParameters&)in);
    return out;
}

void RMLVelocity::checkVelocityTimeout(const base::Time time_last_reference, const double timeout){
    double t_diff = (base::Time::now() - time_last_reference).toSeconds();
    if(!time_last_reference.isNull() && t_diff > timeout){
        LOG_ERROR("Timeout: No new reference velocity arrived for %f seconds. Setting target velocity to zero.", t_diff);
        throw std::runtime_error("Velocity reference timeout");
    }
}
