/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLPositionTask.hpp"
#include <base-logging/Logging.hpp>

using namespace trajectory_generation;

RMLPositionTask::RMLPositionTask(std::string const& name)
    : RMLPositionTaskBase(name){
    rml_flags = new RMLPositionFlags();
}

RMLPositionTask::RMLPositionTask(std::string const& name, RTT::ExecutionEngine* engine)
    : RMLPositionTaskBase(name, engine){
    rml_flags = new RMLPositionFlags();
}

RMLPositionTask::~RMLPositionTask(){
    delete rml_flags;
}

bool RMLPositionTask::configureHook(){
    rml_input_parameters = new RMLPositionInputParameters(_motion_constraints.get().size());
    rml_output_parameters = new RMLPositionOutputParameters(_motion_constraints.get().size());

    if (! RMLPositionTaskBase::configureHook())
        return false;
    return true;
}

void RMLPositionTask::updateHook(){
    RMLPositionTaskBase::updateHook();
}

void RMLPositionTask::errorHook(){
    RMLPositionTaskBase::errorHook();
}

void RMLPositionTask::stopHook(){
    RMLPositionTaskBase::stopHook();
}

bool RMLPositionTask::startHook(){
    if (! RMLPositionTaskBase::startHook())
        return false;
    return true;
}

void RMLPositionTask::cleanupHook(){
    RMLPositionTaskBase::cleanupHook();

    delete rml_input_parameters;
    delete rml_output_parameters;
}

void RMLPositionTask::updateMotionConstraints(const MotionConstraint& constraint,
                                              const size_t idx,
                                              RMLInputParameters* new_input_parameters){

    RMLTask::updateMotionConstraints(constraint, idx, new_input_parameters);
    ((RMLPositionInputParameters*)new_input_parameters)->MaxVelocityVector->VecData[idx] = constraint.max.speed;
}

ReflexxesResultValue RMLPositionTask::performOTG(RMLInputParameters* new_input_parameters,
                                                 RMLOutputParameters* new_output_parameters,
                                                 RMLFlags *rml_flags){

    int result = rml_api->RMLPosition( *(RMLPositionInputParameters*)new_input_parameters,
                                        (RMLPositionOutputParameters*)new_output_parameters,
                                       *(RMLPositionFlags*)rml_flags );

    // Always feed back the new state as the current state in Position based RML. This means that the current robot position
    // is completely ignored. However, on a real robot, using the current position as input in RML will NOT work!
    *new_input_parameters->CurrentPositionVector     = *new_output_parameters->NewPositionVector;
    *new_input_parameters->CurrentVelocityVector     = *new_output_parameters->NewVelocityVector;
    *new_input_parameters->CurrentAccelerationVector = *new_output_parameters->NewAccelerationVector;

    return (ReflexxesResultValue)result;
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

void RMLPositionTask::printParams(){
    ((RMLPositionInputParameters*)rml_input_parameters)->Echo();
    ((RMLPositionOutputParameters*)rml_output_parameters)->Echo();
}

void RMLPositionTask::updateTarget(const base::JointState &cmd,
                                   const size_t idx,
                                   RMLInputParameters* new_input_parameters){

    if(!cmd.hasPosition()){
        LOG_ERROR("Target position of element %i is invalid: %f", idx, cmd.position);
        throw std::invalid_argument("Invalid target position");
    }

    RMLPositionInputParameters* params = (RMLPositionInputParameters*)new_input_parameters;

    // Crop at limits if POSITIONAL_LIMITS_ACTIVELY_PREVENT is selected
    double pos = cmd.position;
#ifdef USING_REFLEXXES_TYPE_IV
    if(rml_flags->PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT){
        double max = rml_input_parameters->MaxPositionVector->VecData[idx] - 1e-10;
        double min = rml_input_parameters->MinPositionVector->VecData[idx] + 1e-10;
        pos = std::max(std::min(max, pos), min);
    }
#endif

    params->TargetPositionVector->VecData[idx] = pos;
    params->SelectionVector->VecData[idx]      = true;

    // Set target speed to zero only if not target speed is given
    params->TargetVelocityVector->VecData[idx] = 0;
    if(cmd.hasSpeed())
        params->TargetVelocityVector->VecData[idx] = cmd.speed;
}

const ReflexxesInputParameters& RMLPositionTask::fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out){
    RMLTask::fromRMLTypes(in, out);
    memcpy(out.max_velocity_vector.data(), ((RMLPositionInputParameters&)in).MaxVelocityVector->VecData, sizeof(double) * in.GetNumberOfDOFs());
    memcpy(out.target_position_vector.data(), ((RMLPositionInputParameters&)in).TargetPositionVector->VecData, sizeof(double) * in.GetNumberOfDOFs());
    return out;
}

const ReflexxesOutputParameters& RMLPositionTask::fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out){
    RMLTask::fromRMLTypes(in, out);
#ifdef USING_REFLEXXES_TYPE_IV
    out.trajectory_exceeds_target_position = ((RMLPositionOutputParameters&)in).TrajectoryExceedsTargetPosition;
#endif
    return out;
}
