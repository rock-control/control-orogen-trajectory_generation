/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLCartesianVelocityTask.hpp"
#include <base-logging/Logging.hpp>

using namespace trajectory_generation;

RTT::FlowStatus RMLCartesianVelocityTask::getCurrentState(CurrentStateData &current_state){

    RTT::FlowStatus fs = _cartesian_state.readNewest(cartesian_state);
    if(fs == RTT::NewData){

        if(!cartesian_state.hasValidPosition() || !cartesian_state.hasValidOrientation()){
            LOG_ERROR("Cartesian state has invalid position and/or orientation.");
            throw std::invalid_argument("Invalid cartesian state");
        }

        current_state.resize(6);
        base::Vector3d euler = toEuler(cartesian_state.orientation);
        memcpy(current_state.position.data(),     cartesian_state.position.data(), sizeof(double)*3);
        memcpy(current_state.position.data()+3,   euler.data(),                    sizeof(double)*3);
        memset(current_state.velocity.data(),     0,                               sizeof(double)*6);
        memset(current_state.acceleration.data(), 0,                               sizeof(double)*6);

        current_sample = cartesian_state;
    }
    if(fs != RTT::NoData){
        current_sample.time = base::Time::now();
        _current_sample.write(current_sample);
    }
    return fs;
}

RTT::FlowStatus RMLCartesianVelocityTask::getTarget(TargetData& target_vector){

    RTT::FlowStatus fs = _target.readNewest(target);
    if(fs == RTT::NewData){
        memcpy(target_vector.velocity.data(),   target.velocity.data(),         sizeof(double) * 3);
        memcpy(target_vector.velocity.data()+3, target.angular_velocity.data(), sizeof(double) * 3);
        memset(target_vector.selection_vector.data(), true, sizeof(double) * 6);
    }
    return fs;
}

void RMLCartesianVelocityTask::writeCommand(const RMLOutputParameters& new_output_parameters){

    base::Vector3d pos, euler, vel, ang_vel;
    memcpy(pos.data(),     new_output_parameters.NewPositionVector->VecData,   sizeof(double)*3);
    memcpy(euler.data(),   new_output_parameters.NewPositionVector->VecData+3, sizeof(double)*3);
    memcpy(vel.data(),     new_output_parameters.NewVelocityVector->VecData,   sizeof(double)*3);
    memcpy(ang_vel.data(), new_output_parameters.NewVelocityVector->VecData+3, sizeof(double)*3);

    current_sample.position    = pos;
    current_sample.orientation = fromEuler(euler);
    command.velocity           = current_sample.velocity         = vel;
    command.angular_velocity   = current_sample.angular_velocity = ang_vel;

    if( convert_to_position ){
        command.position    = pos;
        command.orientation = fromEuler(euler);
    }

    command.time = base::Time::now();
    command.sourceFrame = target.sourceFrame;
    command.targetFrame = target.targetFrame;
    _command.write(command);
}
