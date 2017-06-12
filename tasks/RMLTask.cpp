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

    RTT::FlowStatus fs = updateCurrentState(motion_constraints.names, rml_input_parameters);
    if(fs == RTT::NoData){
        if(state() != NO_CURRENT_STATE)
            state(NO_CURRENT_STATE);
        return;
    }

    fs = updateTarget(motion_constraints, rml_input_parameters);
    if(fs == RTT::NoData){
        if(state() != NO_TARGET)
            state(NO_TARGET);
        return;
    }

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
}

void RMLTask::updateMotionConstraints(const MotionConstraint& constraint,
                                      const size_t idx,
                                      RMLInputParameters* new_input_parameters){

    constraint.validate(); // Check if constraints are ok, e.g. max.speed > 0 etc

#ifdef USING_REFLEXXES_TYPE_IV
    new_input_parameters->MaxPositionVector->VecData[idx] = constraint.max.position;
    new_input_parameters->MinPositionVector->VecData[idx] = constraint.min.position;
#endif
    new_input_parameters->MaxAccelerationVector->VecData[idx] = constraint.max.acceleration;
    new_input_parameters->MaxJerkVector->VecData[idx] = constraint.max_jerk;
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
            printParams();
        }
        break;
    }
#endif
    default:{
#ifdef USING_REFLEXXES_TYPE_IV
        LOG_ERROR("Error in online trajectory generation algorithm: %s", rml_output_parameters->GetErrorString());
#endif
        printParams();
        error(RML_ERROR);
        break;
    }
    }
}

const ReflexxesInputParameters& RMLTask::fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out){

    mempcpy(out.selection_vector.data(), in.SelectionVector->VecData, sizeof(bool) * in.GetNumberOfDOFs());
    memcpy(out.current_position_vector.data(), in.CurrentPositionVector->VecData, sizeof(double) * in.GetNumberOfDOFs());
    memcpy(out.current_velocity_vector.data(), in.CurrentVelocityVector->VecData, sizeof(double) * in.GetNumberOfDOFs());
    memcpy(out.current_acceleration_vector.data(), in.CurrentAccelerationVector->VecData, sizeof(double) * in.GetNumberOfDOFs());

    memcpy(out.max_acceleration_vector.data(), in.MaxAccelerationVector->VecData, sizeof(double) * in.GetNumberOfDOFs());
    memcpy(out.max_jerk_vector.data(), in.MaxJerkVector->VecData, sizeof(double) * in.GetNumberOfDOFs());

    out.min_synchronization_time = in.MinimumSynchronizationTime;

    mempcpy(out.target_velocity_vector.data(), in.TargetVelocityVector->VecData, sizeof(double) * in.GetNumberOfDOFs());

#ifdef USING_REFLEXXES_TYPE_IV
    memcpy(out.max_position_vector.data(), in.MaxPositionVector->VecData, sizeof(double) * in.GetNumberOfDOFs());
    memcpy(out.min_position_vector.data(), in.MinPositionVector->VecData, sizeof(double) * in.GetNumberOfDOFs());
    out.override_value = in.OverrideValue;
#endif

    return out;
}

const ReflexxesOutputParameters& RMLTask::fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out){

    memcpy(out.new_position_vector.data(), in.NewPositionVector->VecData, sizeof(double) * in.GetNumberOfDOFs());
    memcpy(out.new_velocity_vector.data(), in.NewVelocityVector->VecData, sizeof(double) * in.GetNumberOfDOFs());
    memcpy(out.new_acceleration_vector.data(), in.NewAccelerationVector->VecData, sizeof(double) * in.GetNumberOfDOFs());

    memcpy(out.execution_times.data(), in.ExecutionTimes->VecData, sizeof(double) * in.GetNumberOfDOFs());

    out.a_new_calculation_was_performed = in.ANewCalculationWasPerformed;
    out.trajectory_is_phase_synchronized = in.TrajectoryIsPhaseSynchronized;
    out.dof_with_greatest_execution_time = in.DOFWithTheGreatestExecutionTime;
    out.synchronization_time = in.SynchronizationTime;

#ifdef USING_REFLEXXES_TYPE_IV
    out.override_filter_is_active = in.OverrideFilterIsActive;
    out.current_override_value = in.CurrentOverrideValue;
#endif
    return out;
}

void RMLTask::checkVelocityTimeout(const base::Time time_last_reference, const double timeout){
    double t_diff = (base::Time::now() - time_last_reference).toSeconds();
    if(!time_last_reference.isNull() && t_diff > timeout){
        LOG_ERROR("Timeout: No new reference velocity arrived for %f seconds. Setting target velocity to zero.", t_diff);
        throw std::runtime_error("Velocity reference timeout");
    }
}
