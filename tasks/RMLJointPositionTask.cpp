/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLJointPositionTask.hpp"
#include <base-logging/Logging.hpp>

using namespace trajectory_generation;

RTT::FlowStatus RMLJointPositionTask::getCurrentState(CurrentStateData &current_state){

    RTT::FlowStatus fs = _joint_state.readNewest(joint_state);

    if(fs == RTT::NewData){

        current_state.resize(motion_constraints.size());
        for(size_t i = 0; i < motion_constraints.names.size(); i++){
            try{
                const base::JointState &state = joint_state.getElementByName(motion_constraints.names[i]);

                if(!state.hasPosition()){
                    LOG_ERROR("Element %s of joint state does not have a valid position entry", motion_constraints.names[i].c_str());
                    throw std::invalid_argument("Invalid joint state");
                }

                current_sample[i] = state;
                current_state.position[i]     = state.position;
                current_state.velocity[i]     = 0;
                current_state.acceleration[i] = 0;
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

RTT::FlowStatus RMLJointPositionTask::getTarget(TargetData& target_vector){

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
                if(!target[i].hasPosition()){
                    LOG_ERROR("Target element %s does not have a valid position value!", target.names[i].c_str());
                    throw std::invalid_argument("Invalid target");
                }

                size_t idx = motion_constraints.mapNameToIndex(target.names[i]);

                target_vector.selection_vector[idx] = true;
                target_vector.position[idx]         = target[i].position;
                target_vector.velocity[idx]         = 0;
                if(target[i].hasSpeed())
                    target_vector.velocity[idx] = target[i].speed;
            }
            catch(std::exception e){
                LOG_ERROR("Element %s is in target vector but has not been configured in motion constraints", target.names[i].c_str());
                throw e;
            }
        }
    }
    return fs;
}

void RMLJointPositionTask::writeCommand(const RMLOutputParameters& new_output_parameters){

    for(size_t i = 0; i < command.size(); i++){
        command[i].position     = new_output_parameters.NewPositionVector->VecData[i];
        command[i].speed        = new_output_parameters.NewVelocityVector->VecData[i];
        command[i].acceleration = new_output_parameters.NewAccelerationVector->VecData[i];
    }
    current_sample = command;
    command.time = base::Time::now();
    _command.write(command);
}
