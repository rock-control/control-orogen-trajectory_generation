/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLCartesianPositionTask.hpp"
#include <base-logging/Logging.hpp>

using namespace trajectory_generation;

bool RMLCartesianPositionTask::configureHook(){

    rml_flags = new RMLPositionFlags();
    rml_input_parameters = new RMLPositionInputParameters(_motion_constraints.get().size());
    rml_output_parameters = new RMLPositionOutputParameters(_motion_constraints.get().size());

    if (! RMLCartesianPositionTaskBase::configureHook())
        return false;
    return true;
}

void RMLCartesianPositionTask::updateMotionConstraints(const MotionConstraint& constraint,
                                                       const size_t idx,
                                                       RMLInputParameters* new_input_parameters){

    updateMotionConstraints(constraint, idx, (RMLPositionInputParameters*)new_input_parameters);
}

ReflexxesResultValue RMLCartesianPositionTask::performOTG(RMLInputParameters* new_input_parameters,
                                                          RMLOutputParameters* new_output_parameters,
                                                          RMLFlags *rml_flags){

    return performOTG((RMLPositionInputParameters*)new_input_parameters,
                      (RMLPositionOutputParameters*)new_output_parameters,
                      (RMLPositionFlags*)rml_flags);
}

void RMLCartesianPositionTask::writeCommand(const RMLOutputParameters& new_output_parameters){

    base::Vector3d pos, euler, vel, ang_vel;
    memcpy(pos.data(),     new_output_parameters.NewPositionVector->VecData,   sizeof(double)*3);
    memcpy(euler.data(),   new_output_parameters.NewPositionVector->VecData+3, sizeof(double)*3);
    memcpy(vel.data(),     new_output_parameters.NewVelocityVector->VecData,   sizeof(double)*3);
    memcpy(ang_vel.data(), new_output_parameters.NewVelocityVector->VecData+3, sizeof(double)*3);

    command.position           = current_sample.position         = pos;
    command.orientation        = current_sample.orientation      = fromEuler(euler);
    command.velocity           = current_sample.velocity         = vel;
    command.angular_velocity   = current_sample.angular_velocity = ang_vel;

    command.time = base::Time::now();
    command.sourceFrame = target.sourceFrame;
    command.targetFrame = target.targetFrame;
    _command.write(command);
}

void RMLCartesianPositionTask::printParams(const RMLInputParameters& in, const RMLOutputParameters& out){
    ((RMLPositionInputParameters&  )in).Echo();
    ((RMLPositionOutputParameters& )out).Echo();
}

void RMLCartesianPositionTask::updateTarget(const TargetVector& target_vector,
                                            RMLInputParameters* new_input_parameters){
    updateTarget(target_vector, (RMLPositionInputParameters*)new_input_parameters);
}

/*
void RMLCartesianPositionTask::updateTarget(const base::samples::RigidBodyState& cmd,
                                            RMLInputParameters* new_input_parameters){

    if(!cmd.hasValidPosition() || !cmd.hasValidOrientation()){
        LOG_ERROR("Target position and/or orientation is invalid");
        throw std::invalid_argument("Invalid target");
    }

    RMLPositionInputParameters* params = (RMLPositionInputParameters*)new_input_parameters;

    base::Vector3d euler = toEuler(cmd.orientation);

    memset(params->SelectionVector->VecData,        true,                        sizeof(bool)  *6);
    memcpy(params->TargetPositionVector->VecData,   cmd.position.data(),         sizeof(double)*3);
    memcpy(params->TargetPositionVector->VecData+3, euler.data(),                sizeof(double)*3);
    if(cmd.hasValidVelocity() && cmd.hasValidAngularVelocity()){
        memcpy(params->TargetVelocityVector->VecData,   cmd.velocity.data(),         sizeof(double)*3);
        memcpy(params->TargetVelocityVector->VecData+3, cmd.angular_velocity.data(), sizeof(double)*3);
    }

#ifdef USING_REFLEXXES_TYPE_IV
    for(int i = 0; i < 6; i++){
        // Crop at limits if POSITIONAL_LIMITS_ACTIVELY_PREVENT is selected
        double pos = params->TargetPositionVector->VecData[i];
        if(rml_flags->PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT){
            double max = params->MaxPositionVector->VecData[i] - 1e-10;
            double min = params->MinPositionVector->VecData[i] + 1e-10;
            params->TargetPositionVector->VecData[i] = std::max(std::min(max, pos), min);
        }
#endif
    }
}*/

const ReflexxesInputParameters& RMLCartesianPositionTask::fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out){
    out.fromRMLTypes((RMLPositionInputParameters&)in);
    return out;
}

const ReflexxesOutputParameters& RMLCartesianPositionTask::fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out){
    out.fromRMLTypes((RMLPositionOutputParameters&)in);
    return out;
}
