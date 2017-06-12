/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLPositionTask.hpp"
#include <base-logging/Logging.hpp>

using namespace trajectory_generation;

bool RMLPositionTask::configureHook(){

    rml_flags = new RMLPositionFlags();
    rml_input_parameters = new RMLPositionInputParameters(_motion_constraints.get().size());
    rml_output_parameters = new RMLPositionOutputParameters(_motion_constraints.get().size());

    if (! RMLPositionTaskBase::configureHook())
        return false;
    return true;
}

void RMLPositionTask::updateMotionConstraints(const MotionConstraint& constraint,
                                              const size_t idx,
                                              RMLInputParameters* new_input_parameters){

    updateMotionConstraints(constraint, idx, (RMLPositionInputParameters*)new_input_parameters);
}

ReflexxesResultValue RMLPositionTask::performOTG(RMLInputParameters* new_input_parameters,
                                                 RMLOutputParameters* new_output_parameters,
                                                 RMLFlags *rml_flags){

    return performOTG((RMLPositionInputParameters*)new_input_parameters,
                      (RMLPositionOutputParameters*)new_output_parameters,
                      (RMLPositionFlags*)rml_flags);
}

void RMLPositionTask::writeCommand(const RMLOutputParameters& new_output_parameters){

    for(size_t i = 0; i < command.size(); i++){
        command[i].position     = current_sample[i].position     = new_output_parameters.NewPositionVector->VecData[i];
        command[i].speed        = current_sample[i].speed        = new_output_parameters.NewVelocityVector->VecData[i];
        command[i].acceleration = current_sample[i].acceleration = new_output_parameters.NewAccelerationVector->VecData[i];
    }
    command.time = base::Time::now();
    _command.write(command);
}

void RMLPositionTask::printParams(const RMLInputParameters& in, const RMLOutputParameters& out){
    ((RMLPositionInputParameters&  )in).Echo();
    ((RMLPositionOutputParameters& )out).Echo();
}

void RMLPositionTask::updateTarget(const TargetVector& target_vector,
                                            RMLInputParameters* new_input_parameters){
    updateTarget(target_vector, (RMLPositionInputParameters*)new_input_parameters);
}

/*
void RMLPositionTask::updateTarget(const base::JointState &cmd,
                                   const size_t idx,
                                   RMLInputParameters* new_input_parameters){

    if(!cmd.hasPosition()){
        LOG_ERROR("Target position of element %i is invalid: %f", idx, cmd.position);
        throw std::invalid_argument("Invalid target");
    }

    RMLPositionInputParameters* params = (RMLPositionInputParameters*)new_input_parameters;

    // Crop at limits if POSITIONAL_LIMITS_ACTIVELY_PREVENT is selected
    double pos = cmd.position;
#ifdef USING_REFLEXXES_TYPE_IV
    if(rml_flags->PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT){
        double max = params->MaxPositionVector->VecData[idx] - 1e-10;
        double min = params->MinPositionVector->VecData[idx] + 1e-10;
        pos = std::max(std::min(max, pos), min);
    }
#endif

    params->TargetPositionVector->VecData[idx] = pos;
    params->SelectionVector->VecData[idx]      = true;

    // Set target speed to zero only if not target speed is given
    params->TargetVelocityVector->VecData[idx] = 0;
    if(cmd.hasSpeed())
        params->TargetVelocityVector->VecData[idx] = cmd.speed;
}*/

const ReflexxesInputParameters& RMLPositionTask::fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out){
    out.fromRMLTypes((RMLPositionInputParameters&)in);
    return out;
}

const ReflexxesOutputParameters& RMLPositionTask::fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out){
    out.fromRMLTypes((RMLPositionOutputParameters&)in);
    return out;
}
