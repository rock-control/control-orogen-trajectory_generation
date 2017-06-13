/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLPosition.hpp"

using namespace trajectory_generation;

bool RMLPosition::configureHook(){

    rml_flags = new RMLPositionFlags();
    rml_input_parameters = new RMLPositionInputParameters(_motion_constraints.get().size());
    rml_output_parameters = new RMLPositionOutputParameters(_motion_constraints.get().size());

    if (! RMLPositionBase::configureHook())
        return false;
    return true;
}

void RMLPosition::updateMotionConstraints(const MotionConstraint& constraint,
                                          const size_t idx,
                                          RMLInputParameters* new_input_parameters){

    RMLPositionInputParameters* params = (RMLPositionInputParameters*)new_input_parameters;


}

void RMLPosition::updateTarget(const TargetData& target_vector,
                               RMLInputParameters* new_input_parameters){

    RMLPositionInputParameters* params = (RMLPositionInputParameters*)new_input_parameters;

    uint n_dof = params->NumberOfDOFs;
    memcpy(params->TargetPositionVector->VecData, target_vector.position.data(),         sizeof(double) * n_dof);
    memcpy(params->TargetVelocityVector->VecData, target_vector.velocity.data(),         sizeof(double) * n_dof);
    memcpy(params->SelectionVector->VecData,      target_vector.selection_vector.data(), n_dof);

#ifdef USING_REFLEXXES_TYPE_IV
    for(uint i = 0; i < n_dof; i++){
        // Crop at limits if POSITIONAL_LIMITS_ACTIVELY_PREVENT is selected
        double pos =  params->TargetPositionVector->VecData[i];
        if(rml_flags->PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT){
            double max = params->MaxPositionVector->VecData[i] - 1e-10;
            double min = params->MinPositionVector->VecData[i] + 1e-10;
            params->TargetPositionVector->VecData[i] = std::max(std::min(max, pos), min);
        }
    }
#endif

    for(size_t i = 0; i < target_vector.constraints.size(); i++){
        MotionConstraint constraint = target_vector.constraints[i];  // Get new motion constraint for target i
        constraint.applyDefaultIfUnset(motion_constraints[i]);    // Use default entry if new constraints entries are unset
        updateMotionConstraints(constraint, i, params);
    }
}

ReflexxesResultValue RMLPosition::performOTG(RMLInputParameters* new_input_parameters,
                                             RMLOutputParameters* new_output_parameters,
                                             RMLFlags *rml_flags){

    int result = rml_api->RMLPosition( *(RMLPositionInputParameters*)new_input_parameters,
                                        (RMLPositionOutputParameters*)new_output_parameters,
                                       *(RMLPositionFlags*)rml_flags );

    // Always feed back the new state as the current state. This means that the current robot position
    // is completely ignored. However, on a real robot, using the current position as input in RML will NOT work!
    *new_input_parameters->CurrentPositionVector     = *new_output_parameters->NewPositionVector;
    *new_input_parameters->CurrentVelocityVector     = *new_output_parameters->NewVelocityVector;
    *new_input_parameters->CurrentAccelerationVector = *new_output_parameters->NewAccelerationVector;

    return (ReflexxesResultValue)result;
}

void RMLPosition::printParams(const RMLInputParameters& in, const RMLOutputParameters& out){
    ((RMLPositionInputParameters&  )in).Echo();
    ((RMLPositionOutputParameters& )out).Echo();
}

const ReflexxesInputParameters& RMLPosition::fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out){
    out.fromRMLTypes((RMLPositionInputParameters&)in);
    return out;
}

const ReflexxesOutputParameters& RMLPosition::fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out){
    out.fromRMLTypes((RMLPositionOutputParameters&)in);
    return out;
}
