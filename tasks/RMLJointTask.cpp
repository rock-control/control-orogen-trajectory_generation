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
}

RTT::FlowStatus RMLJointTask::updateCurrentState(const std::vector<std::string> &names,
                                                 RMLInputParameters* new_input_parameters){

    RTT::FlowStatus fs = _joint_state.readNewest(joint_state);
    if(fs == RTT::NewData){

        for(size_t i = 0; i < names.size(); i++)
            try{
            size_t idx = joint_state.mapNameToIndex(names[i]);
            setJointState(joint_state[idx], idx, new_input_parameters);
        }
        catch(std::exception e){
            LOG_ERROR("Element %s has been configured in motion constraints, but is not available in joint state", motion_constraints.names[i].c_str());
            throw e;
        }
    }
    return fs;
}

RTT::FlowStatus RMLJointTask::updateTarget(const MotionConstraints& default_constraints,
                                           RMLInputParameters* new_input_parameters){

    RTT::FlowStatus fs = _target.readNewest(target);
    if(fs != RTT::NewData)
        fs = _constrained_target.readNewest(target);

    if(fs == RTT::NewData){

        target.validate();
        memset(new_input_parameters->SelectionVector->VecData, false, default_constraints.size());

        for(size_t i = 0; i < target.size(); i++)
            try{
                size_t idx = default_constraints.mapNameToIndex(target.names[i]);

                MotionConstraint constraint = target.motion_constraints[i];  // Get new motion constraint for target i
                constraint.applyDefaultIfUnset(default_constraints[idx]);    // Use default entry if new constraints entries are unset
                setMotionConstraints(constraint, idx, new_input_parameters);

                const base::JointState& cmd = target[i];
                setJointTarget(cmd, idx, new_input_parameters);
            }
            catch(std::exception e){
                LOG_ERROR("Element %s is in target vector but has not been configured in motion constraints", target.names[i].c_str());
                throw e;
            }
    }
    return fs;
}
