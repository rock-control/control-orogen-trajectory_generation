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
    if(motion_constraints.size() != motion_constraints.names.size()){
        LOG_ERROR("Number of elements in motion constraints must be same as size of the names vector");
        return false;
    }
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

    RTT::FlowStatus fs = updateCurrentState(rml_input_parameters);
    if(fs == RTT::NoData){
        if(state() != NO_CURRENT_STATE)
            state(NO_CURRENT_STATE);
        return;
    }

    fs = updateTarget(rml_input_parameters);
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
    _rml_input_parameters.write(convertRMLInputParams(*rml_input_parameters, input_parameters));
    _rml_output_parameters.write(convertRMLOutputParams(*rml_output_parameters, output_parameters));
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
