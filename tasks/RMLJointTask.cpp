/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLJointTask.hpp"
#include <base-logging/Logging.hpp>

using namespace trajectory_generation;

RMLJointTask::RMLJointTask(std::string const& name) : RMLJointTaskBase(name){
}

RMLJointTask::RMLJointTask(std::string const& name, RTT::ExecutionEngine* engine) : RMLJointTaskBase(name, engine){
}

RMLJointTask::~RMLJointTask(){
}

bool RMLJointTask::configureHook(){
    if (! RMLJointTaskBase::configureHook())
        return false;

    command.resize(_motion_constraints.get().size());
    command.names = _motion_constraints.get().names;

    current_sample.resize(_motion_constraints.get().size());
    current_sample.names = _motion_constraints.get().names;

    return true;
}

bool RMLJointTask::startHook(){
    if (! RMLJointTaskBase::startHook())
        return false;
    return true;
}

void RMLJointTask::updateHook(){
    RMLJointTaskBase::updateHook();
}

void RMLJointTask::errorHook(){
    RMLJointTaskBase::errorHook();
}

void RMLJointTask::stopHook(){
    RMLJointTaskBase::stopHook();
}

void RMLJointTask::cleanupHook(){
    RMLJointTaskBase::cleanupHook();

    current_sample.clear();
    joint_state.clear();
    target.clear();
    command.clear();
}

RTT::FlowStatus RMLJointTask::updateCurrentState(const std::vector<std::string> &names,
                                                 RMLInputParameters* new_input_parameters){

    RTT::FlowStatus fs = _joint_state.readNewest(joint_state);
    if(fs == RTT::NewData && rml_result_value == RML_NOT_INITIALIZED){

        for(size_t i = 0; i < names.size(); i++){
            try{
                const base::JointState &state = joint_state.getElementByName(names[i]);
                new_input_parameters->CurrentPositionVector->VecData[i]     = current_sample[i].position = state.position;
                new_input_parameters->CurrentVelocityVector->VecData[i]     = current_sample[i].speed = 0;
                new_input_parameters->CurrentAccelerationVector->VecData[i] = current_sample[i].acceleration = 0;
            }
            catch(std::exception e){
                LOG_ERROR("Element %s has been configured in motion constraints, but is not available in joint state", motion_constraints.names[i].c_str());
                throw e;
            }
        }
    }
    if(fs != RTT::NoData){
        current_sample.time = joint_state.time;
        _current_sample.write(current_sample);
    }
    return fs;
}

RTT::FlowStatus RMLJointTask::updateTarget(const MotionConstraints& default_constraints,
                                           RMLInputParameters* new_input_parameters){

    RTT::FlowStatus fs_target = _target.readNewest(target);
    RTT::FlowStatus fs_constr_target = _constrained_target.readNewest(target);
    if(fs_constr_target != RTT::NoData && fs_target != RTT::NoData)
        throw std::runtime_error("There is data on both, the target AND the constrained_target port. You should use only one of the two ports!");

    RTT::FlowStatus fs = RTT::NoData;
    if(fs_target != RTT::NoData)
        fs = fs_target;
    else if(fs_constr_target != RTT::NoData)
        fs = fs_constr_target;

    if(fs == RTT::NewData){

        target.validate();
        memset(new_input_parameters->SelectionVector->VecData, false, default_constraints.size());

        for(size_t i = 0; i < target.size(); i++){
            try{
                size_t idx = default_constraints.mapNameToIndex(target.names[i]);

                if(!target.motion_constraints.empty()){
                    MotionConstraint constraint = target.motion_constraints[i];  // Get new motion constraint for target i
                    constraint.applyDefaultIfUnset(default_constraints[idx]);    // Use default entry if new constraints entries are unset
                    updateMotionConstraints(constraint, idx, new_input_parameters);
                }

                const base::JointState& cmd = target[i];
                updateTarget(cmd, idx, new_input_parameters);
            }
            catch(std::exception e){
                LOG_ERROR("Element %s is in target vector but has not been configured in motion constraints", target.names[i].c_str());
                throw e;
            }
        } // for loop
    }
    return fs;
}
