/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLTask.hpp"
#include <base-logging/Logging.hpp>

using namespace trajectory_generation;

RMLTask::RMLTask(std::string const& name)
    : RMLTaskBase(name){
}

RMLTask::RMLTask(std::string const& name, RTT::ExecutionEngine* engine)
    : RMLTaskBase(name, engine){
}

RMLTask::~RMLTask(){
}

bool RMLTask::configureHook(){

    if (! RMLTaskBase::configureHook())
        return false;

    cycle_time = _cycle_time.get();
    if(cycle_time <= 0){
        LOG_ERROR("Cycle time should be > 0, but is %i", cycle_time);
        return false;
    }

    motion_constraints = _motion_constraints.get();
    for(size_t i = 0; i < motion_constraints.size(); i++)
        updateMotionConstraints(motion_constraints[i], i, rml_input_parameters);

    rml_flags->SynchronizationBehavior = _synchronization_behavior.get();
#ifdef USING_REFLEXXES_TYPE_IV
    rml_flags->PositionalLimitsBehavior = _positional_limits_behavior.get();
#endif

    rml_api = new ReflexxesAPI(motion_constraints.size(), cycle_time);
    rml_result_value = RML_NOT_INITIALIZED;

    input_parameters = ReflexxesInputParameters(rml_input_parameters->NumberOfDOFs);
    output_parameters = ReflexxesOutputParameters(rml_input_parameters->NumberOfDOFs);

    return true;
}

bool RMLTask::startHook(){
    if (! RMLTaskBase::startHook())
        return false;
    return true;
}

void RMLTask::updateHook(){

    base::Time start_time = base::Time::now();
    if(!timestamp.isNull())
        _actual_cycle_time.write((start_time - timestamp).toSeconds());
    timestamp = start_time;

    RMLTaskBase::updateHook();

    RTT::FlowStatus fs = getCurrentState(current_position);
    if(fs == RTT::NoData){
        if(state() != NO_CURRENT_STATE)
            state(NO_CURRENT_STATE);
        return;
    }
    else if (fs == RTT::NewData && rml_result_value == RML_NOT_INITIALIZED)
        setInitialState(current_position, rml_input_parameters);

    fs = getTarget(target_vector);
    if(fs == RTT::NoData){
        if(state() != NO_TARGET)
            state(NO_TARGET);
        return;
    }
    else if(fs == RTT::NewData)
        updateTarget(target_vector, rml_input_parameters);

    if(state() == NO_TARGET || state() == NO_CURRENT_STATE)
        state(RUNNING);

    rml_result_value = performOTG(rml_input_parameters, rml_output_parameters, rml_flags);
    handleResultValue(rml_result_value);

    writeCommand(*rml_output_parameters);

    // write debug data
    _rml_input_parameters.write(fromRMLTypes(*rml_input_parameters, input_parameters));
    _rml_output_parameters.write(fromRMLTypes(*rml_output_parameters, output_parameters));
    _computation_time.write((base::Time::now() - start_time).toSeconds());
}

void RMLTask::errorHook(){
    RMLTaskBase::errorHook();
}

void RMLTask::stopHook(){
    RMLTaskBase::stopHook();
}

void RMLTask::cleanupHook(){
    RMLTaskBase::cleanupHook();

    motion_constraints.clear();
    delete rml_api;
    delete rml_input_parameters;
    delete rml_output_parameters;
    delete rml_flags;
}

void RMLTask::updateMotionConstraints(const MotionConstraint& constraint,
                                      const size_t idx,
                                      RMLPositionInputParameters* new_input_parameters){
}

void RMLTask::updateMotionConstraints(const MotionConstraint& constraint,
                                      const size_t idx,
                                      RMLVelocityInputParameters* new_input_parameters){
    constraint.validate(); // Check if constraints are ok, e.g. max.speed > 0 etc

#ifdef USING_REFLEXXES_TYPE_IV
    new_input_parameters->MaxPositionVector->VecData[idx] = constraint.max.position;
    new_input_parameters->MinPositionVector->VecData[idx] = constraint.min.position;
#endif
    new_input_parameters->MaxAccelerationVector->VecData[idx] = constraint.max.acceleration;
    new_input_parameters->MaxJerkVector->VecData[idx] = constraint.max_jerk;
}

void RMLTask::setInitialState(const std::vector<double>& current_position,
                              RMLInputParameters* new_input_parameters){

    int n_dof = new_input_parameters->NumberOfDOFs;
    memcpy(new_input_parameters->CurrentPositionVector->VecData,     current_position.data(),  sizeof(double) * n_dof);
    memset(new_input_parameters->CurrentVelocityVector->VecData,     0,                        sizeof(double) * n_dof);
    memset(new_input_parameters->CurrentAccelerationVector->VecData, 0,                        sizeof(double) * n_dof);
}

void RMLTask::updateTarget(const TargetData& target_vector,
                           RMLPositionInputParameters* new_input_parameters){

    int n_dof = new_input_parameters->NumberOfDOFs;
    memcpy(new_input_parameters->TargetPositionVector->VecData, target_vector.position.data(),         sizeof(double) * n_dof);
    memcpy(new_input_parameters->TargetVelocityVector->VecData, target_vector.velocity.data(),         sizeof(double) * n_dof);
    memcpy(new_input_parameters->SelectionVector->VecData,      target_vector.selection_vector.data(), n_dof);

#ifdef USING_REFLEXXES_TYPE_IV
    for(size_t i = 0; i < n_dof; i++){
        // Crop at limits if POSITIONAL_LIMITS_ACTIVELY_PREVENT is selected
        double pos =  new_input_parameters->TargetPositionVector->VecData[i];
        if(rml_flags->PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT){
            double max = new_input_parameters->MaxPositionVector->VecData[i] - 1e-10;
            double min = new_input_parameters->MinPositionVector->VecData[i] + 1e-10;
            new_input_parameters->TargetPositionVector->VecData[i] = std::max(std::min(max, pos), min);
        }
    }
#endif

    for(size_t i = 0; i < target_vector.constraints.size(); i++){
        MotionConstraint constraint = target_vector.constraints[i];  // Get new motion constraint for target i
        constraint.applyDefaultIfUnset(motion_constraints[i]);    // Use default entry if new constraints entries are unset
        updateMotionConstraints(constraint, i, new_input_parameters);
    }
}

void RMLTask::updateTarget(const TargetData& target_vector,
                           RMLVelocityInputParameters* new_input_parameters){

    int n_dof = new_input_parameters->NumberOfDOFs;
    memcpy(new_input_parameters->TargetVelocityVector->VecData, target_vector.velocity.data(),         sizeof(double) * n_dof);
    memcpy(new_input_parameters->SelectionVector->VecData,      target_vector.selection_vector.data(), sizeof(bool) * n_dof);

#ifdef USING_REFLEXXES_TYPE_IV
    for(size_t i = 0; i < n_dof; i++){
        // If a joint is at a position limit, the target velocity is non-zero and pointing in direction of the limit, the sychronization time is
        // computed by RML as if the constrained joint could move freely. This might lead to incorrect synchronization time for all other joints.
        // Workaround: Set the target velocity to zero if (and only if) POSITIONAL_LIMITS_ACTIVELY_PREVENT is selected

        if(rml_flags->PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT){
            double cur_pos = new_input_parameters->CurrentPositionVector->VecData[i];
            double target_vel = new_input_parameters->TargetVelocityVector->VecData[i];
            double max_pos = new_input_parameters->MaxPositionVector->VecData[i];
            double min_pos = new_input_parameters->MinPositionVector->VecData[i];

            if( (target_vel*cycle_time + cur_pos > max_pos) || (target_vel*cycle_time + cur_pos < min_pos) )
                new_input_parameters->TargetVelocityVector->VecData[i] = 0;
        }
    }
#endif

    for(size_t i = 0; i < target_vector.constraints.size(); i++){
        MotionConstraint constraint = target_vector.constraints[i];  // Get new motion constraint for target i
        constraint.applyDefaultIfUnset(motion_constraints[i]);    // Use default entry if new constraints entries are unset
        updateMotionConstraints(constraint, i, new_input_parameters);
    }
}

ReflexxesResultValue RMLTask::performOTG(RMLPositionInputParameters* new_input_parameters,
                                         RMLPositionOutputParameters* new_output_parameters,
                                         RMLPositionFlags *rml_flags){

    int result = rml_api->RMLPosition( *new_input_parameters, new_output_parameters, *rml_flags );

    // Always feed back the new state as the current state. This means that the current robot position
    // is completely ignored. However, on a real robot, using the current position as input in RML will NOT work!
    *new_input_parameters->CurrentPositionVector     = *new_output_parameters->NewPositionVector;
    *new_input_parameters->CurrentVelocityVector     = *new_output_parameters->NewVelocityVector;
    *new_input_parameters->CurrentAccelerationVector = *new_output_parameters->NewAccelerationVector;

    return (ReflexxesResultValue)result;
}

ReflexxesResultValue RMLTask::performOTG(RMLVelocityInputParameters* new_input_parameters,
                                         RMLVelocityOutputParameters* new_output_parameters,
                                         RMLVelocityFlags *rml_flags){

    int result = rml_api->RMLVelocity( *new_input_parameters, new_output_parameters, *rml_flags );

    // Always feed back the new state as the current state. This means that the current robot position
    // is completely ignored. However, on a real robot, using the current position as input in RML will NOT work!
    *new_input_parameters->CurrentPositionVector     = *new_output_parameters->NewPositionVector;
    *new_input_parameters->CurrentVelocityVector     = *new_output_parameters->NewVelocityVector;
    *new_input_parameters->CurrentAccelerationVector = *new_output_parameters->NewAccelerationVector;

    return (ReflexxesResultValue)result;
}

void RMLTask::handleResultValue(ReflexxesResultValue result_value){

    _rml_result_value.write(result_value);

    switch(result_value){
    case RML_WORKING:{
        if(state() != FOLLOWING)
            state(FOLLOWING);
        break;
    }
    case RML_FINAL_STATE_REACHED:{
        if(state() != REACHED)
            state(REACHED);
        break;
    }
    case ReflexxesAPI::RML_ERROR_SYNCHRONIZATION:
        // Ignore this error. It occurs from time to time without having any effect
        break;
#ifdef USING_REFLEXXES_TYPE_IV
    case RML_ERROR_POSITIONAL_LIMITS:{

        if(rml_flags->PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ERROR_MSG_ONLY){
            LOG_ERROR("RML target position out of bounds. Modify your target position and/or positional limits or "
                      "choose POSITIONAL_LIMITS_IGNORE/POSITIONAL_LIMITS_ACTIVELY_PREVENT to avoid this error");
            error(RML_ERROR);
            printParams(*rml_input_parameters, *rml_output_parameters);
        }
        break;
    }
#endif
    default:{
#ifdef USING_REFLEXXES_TYPE_IV
        LOG_ERROR("Error in online trajectory generation algorithm: %s", rml_output_parameters->GetErrorString());
#endif
        printParams(*rml_input_parameters, *rml_output_parameters);
        error(RML_ERROR);
        break;
    }
    }
}
