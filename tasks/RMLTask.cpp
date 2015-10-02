/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLTask.hpp"
#include <base/Logging.hpp>

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

    double cycle_time = _cycle_time.get();
    if(cycle_time <= 0){
        LOG_ERROR("Cycle time should be > 0, but is %i", cycle_time);
        return false;
    }

    motion_constraints = _motion_constraints.get();
    if(motion_constraints.empty()){
        LOG_ERROR("Motion constraints are empty");
        return false;
    }

    rml_api = new ReflexxesAPI(motion_constraints.size(), cycle_time);

    rml_initialized = false;

    input_parameters = ReflexxesInputParameters(motion_constraints.size());
    output_parameters = ReflexxesOutputParameters(motion_constraints.size());

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

    RTT::FlowStatus joint_state_status = _joint_state.readNewest(joint_state);
    if(joint_state_status == RTT::NoData){
        if(state() != NO_JOINT_STATE)
            state(NO_JOINT_STATE);
        return;
    }
    else if(joint_state_status == RTT::NewData){
        handleNewJointState(joint_state);
        _current_sample.write(current_sample);
    }

    RTT::FlowStatus target_status = _target.readNewest(target);
    RTT::FlowStatus constrained_target_status = _constrained_target.readNewest(constrained_target);
    if(target_status == RTT::NoData && constrained_target_status == RTT::NoData){
        if(state() != NO_TARGET)
            state(NO_TARGET);
        return;
    }

    if(state() == NO_TARGET || state() == NO_JOINT_STATE)
        state(RUNNING);

    // Handle targets. Notice, that we have implicitly a priorization here: If there is data on target port, it will be preferred over the constrained target port
    if(target_status == RTT::NewData)
        handleNewTarget(target);
    else if(constrained_target_status == RTT::NewData)
        handleNewConstrainedTarget(constrained_target);

    handleResultValue(performOTG(command));
    command.time = base::Time::now();
    _command.write(command);

    // Write debug data
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

void RMLTask::handleNewJointState(const base::samples::Joints &joint_state){

    if(!rml_initialized){
        current_sample.resize(motion_constraints.size());
        current_sample.names = motion_constraints.names;
    }

    for(size_t i = 0; i < motion_constraints.size(); i++)
    {
        size_t idx;
        try{
            idx = joint_state.mapNameToIndex(motion_constraints.names[i]);
        }
        catch(std::exception e){
            LOG_ERROR("%s: Element %s has been configured in motion constraints, but is not available in joint state",
                      this->getName().c_str(), motion_constraints.names[i].c_str());
            throw e;
        }

        const base::JointState &state = joint_state[idx];

        if(!rml_initialized){ // Init with current joints state

            if(state.hasPosition())
                rml_input_parameters->CurrentPositionVector->VecData[i] = current_sample[i].position = state.position;
            else
                throw std::invalid_argument("Joint state contains NaN position elements");
            rml_input_parameters->CurrentVelocityVector->VecData[i] = current_sample[i].speed = 0;
            rml_input_parameters->CurrentAccelerationVector->VecData[i] = current_sample[i].acceleration = 0;
        }
        else
            setJointState(state, i);
    }
    rml_initialized = true;
}

void RMLTask::handleNewTarget(const base::commands::Joints &target){

    memset(rml_input_parameters->SelectionVector->VecData, false, motion_constraints.size());

    for(size_t i = 0; i < target.size(); i++){
        size_t idx;
        try{
            idx = motion_constraints.mapNameToIndex(target.names[i]);
        }
        catch(std::exception e){
            LOG_ERROR("%s: Element %s is given as target, but has not been configured in motion_constraints",
                      this->getName().c_str(), motion_constraints.names[i].c_str());
            throw e;
        }
        rml_input_parameters->SelectionVector->VecData[idx] = true;

        setTarget(target[i], idx);
    }
}

void RMLTask::handleNewConstrainedTarget(const trajectory_generation::ConstrainedJointsCmd &constrained_target){
    memset(rml_input_parameters->SelectionVector->VecData, false, motion_constraints.size());

    constrained_target.validate();

    for(size_t i = 0; i < constrained_target.size(); i++){
        size_t idx;
        try{
            idx = motion_constraints.mapNameToIndex(constrained_target.names[i]);
        }
        catch(std::exception e){
            LOG_ERROR("%s: Element %s is given as target, but has not been configured in motion_constraints",
                      this->getName().c_str(), motion_constraints.names[i].c_str());
            throw e;
        }
        rml_input_parameters->SelectionVector->VecData[idx] = true;

        // if a value of the new motion constraints is unset, use the default motion constraints instead:
        JointMotionConstraints constraints = constrained_target.motion_constraints[i];
        constraints.applyDefaultIfUnset(motion_constraints[idx]);

        setTarget(constrained_target[i], idx);
        setMotionConstraints(constraints, idx);
    }
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

        // Bugfix for reflexxes: in case POSITIONAL_LIMITS_ACTIVELY_PREVENT the result value will become RML_ERROR_POSITIONAL_LIMITS if
        // the target value is out of bounds. This behavior is somewhat not intuitive, since violating the joint limits will be prevented.
        // Thus, we check the states manually.
        if(rml_flags->PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT){
            if(rml_output_parameters->SynchronizationTime == 0){
                if(state() != REACHED)
                    state(REACHED);
            }else{
                if(state() != FOLLOWING)
                    state(FOLLOWING);
            }
        }

        if(rml_flags->PositionalLimitsBehavior == RMLFlags::POSITIONAL_LIMITS_ERROR_MSG_ONLY){
            LOG_ERROR("RML target position out of bounds. To prevent this error, either modify max/min position in your motion constraints,"
                      " or set positional_limits_behaviour to POSITIONAL_LIMITS_IGNORE or POSITIONAL_LIMITS_ACTIVELY_PREVENT");
            error(RML_ERROR);
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

void RMLTask::setOverrideValue(double override_value){
#ifdef USING_REFLEXXES_TYPE_IV
    assert(override_value <= 10.0 && override_value > 0);
    rml_input_parameters->OverrideValue = override_value;
#endif
}
