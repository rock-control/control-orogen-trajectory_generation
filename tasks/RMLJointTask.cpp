/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLJointTask.hpp"

using namespace trajectory_generation;

bool RMLJointTask::configureHook()
{
    if (! RMLJointTaskBase::configureHook())
        return false;

    current_joint_sample.resize(_motion_constraints.get().size());
    current_joint_sample.names = _motion_constraints.get().names;

    return true;
}
bool RMLJointTask::startHook()
{
    if (! RMLJointTaskBase::startHook())
        return false;
    return true;
}

RTT::FlowStatus RMLJointTask::getCurrentState(CurrentStateVector& new_current_state){

    RTT::FlowStatus fs = _current_state.readNewest(current_joint_state);
    if(fs == RTT::NewData){

        new_current_state.resize(motion_constraints.size());
        new_current_state.names = motion_constraints.names;

        for(size_t i = 0; i < motion_constraints.size(); i++)
        {
            size_t idx;
            try{
                idx = current_state.mapNameToIndex(motion_constraints.names[i]);
            }
            catch(std::exception e){
                LOG_ERROR("%s: Element %s has been configured in motion constraints, but is not available in joint state",
                          this->getName().c_str(), motion_constraints.names[i].c_str());
                throw e;
            }

            // Init with current joint position and zero velocity/acceleration, if RML has not yet been called
            const base::JointState &state = current_joint_state[idx];
            new_current_state[i].position = state.position;
            new_current_state[i].velocity = state.speed;
            new_current_state[i].acceleration = state.acceleration;
        }
    }
    return fs;
}

RTT::FlowStatus RMLJointTask::getTarget(TargetVector& new_target){

    RTT::FlowStatus fs = _target.readNewest(joint_target);
    if(fs != RTT::NewData)
        fs = _constrained_target.readNewest(joint_target);

    if(fs == RTT::NewData){

        joint_target.validate();

        new_target.resize(motion_constraints.size());
        new_target.names = motion_constraints.names;
        new_target.reset();

        for(size_t i = 0; i < target.size(); i++){

            size_t idx;
            try{
                idx = motion_constraints.mapNameToIndex(target.names[i]);
            }
            catch(std::exception e){
                LOG_ERROR("%s: Element %s is in target vector but has not been configured in motion constraints",
                          this->getName().c_str(), target.names[i].c_str());
                throw e;
            }

            new_target[idx].selected = true;
            new_target[idx].position = joint_target[i].position;
            new_target[idx].velocity = joint_target[i].speed;
            new_target.motion_constraints = joint_target.motion_constraints;
        }

        new_target.motion_constraints = motion_constraints;

        for(size_t i = 0; i < joint_target.motion_constraints.size(); i++){

            MotionConstraint& constraint = joint_target.motion_constraints[i];
            uint idx = motion_constraints.mapNameToIndex(joint_target.names[i]);
            constraint.applyDefaultIfUnset(motion_constraints[idx]);
            new_target.motion_constraints[idx] = constraint;
        }
    }
    return fs;
}
