/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLJointTask.hpp"
#include <base-logging/Logging.hpp>

using namespace trajectory_generation;

bool RMLJointTask::configureHook(){
    if (! RMLJointTaskBase::configureHook())
        return false;

    command.resize(_motion_constraints.get().size());
    command.names = _motion_constraints.get().names;

    current_sample.resize(_motion_constraints.get().size());
    current_sample.names = _motion_constraints.get().names;

    return true;
}

void RMLJointTask::cleanupHook(){
    RMLJointTaskBase::cleanupHook();

    current_sample.clear();
    joint_state.clear();
    target.clear();
    command.clear();
}

RTT::FlowStatus RMLJointTask::getCurrentPosition(std::vector<double> &current_position){

    RTT::FlowStatus fs = _joint_state.readNewest(joint_state);
    if(fs == RTT::NewData){
        current_position.resize(motion_constraints.size());
        for(size_t i = 0; i < motion_constraints.names.size(); i++){
            try{
                const base::JointState &state = joint_state.getElementByName(motion_constraints.names[i]);
                current_sample[i] = state;
                current_position[i] = state.position;
            }
            catch(std::exception e){
                LOG_ERROR("Element %s has been configured in motion constraints, but is not available in joint state", motion_constraints.names[i].c_str());
                throw e;
            }
        }
    }
    if(fs != RTT::NoData){
        current_sample.time = base::Time::now();
        _current_sample.write(current_sample);
    }
    return fs;
}


RTT::FlowStatus RMLJointTask::getTarget(TargetData& target_vector){
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
        target_vector.resize(motion_constraints.size());
        target_vector.constraints = target.motion_constraints;

        for(size_t i = 0; i < target.size(); i++){
            try{
                size_t idx = motion_constraints.mapNameToIndex(target.names[i]);

                target_vector.position[idx] = target[i].position;
                target_vector.velocity[idx] = 0;
                if(target[i].hasSpeed())
                    target_vector.velocity[idx] = target[i].speed;
                target_vector.selection_vector[idx] = true;
            }
            catch(std::exception e){
                LOG_ERROR("Element %s is in target vector but has not been configured in motion constraints", target.names[i].c_str());
                throw e;
            }
        } // for loop
    }

    return fs;
}

/*
RTT::FlowStatus RMLJointTask::updateTarget(RMLInputParameters* new_input_parameters){

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
        memset(new_input_parameters->SelectionVector->VecData, false, motion_constraints.size());

        for(size_t i = 0; i < target.size(); i++){
            try{
                size_t idx = motion_constraints.mapNameToIndex(target.names[i]);

                if(!target.motion_constraints.empty()){
                    MotionConstraint constraint = target.motion_constraints[i];  // Get new motion constraint for target i
                    constraint.applyDefaultIfUnset(motion_constraints[idx]);    // Use default entry if new constraints entries are unset
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
}*/
