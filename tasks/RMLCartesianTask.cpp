/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RMLCartesianTask.hpp"
#include <base-logging/Logging.hpp>

using namespace trajectory_generation;

bool RMLCartesianTask::configureHook(){

    if(_motion_constraints.get().size() != 6){
        LOG_ERROR("Size of motion constraints must be 6 for a Cartesian task, but is %i", _motion_constraints.get().size());
        return false;
    }
    if (! RMLCartesianTaskBase::configureHook())
        return false;
    return true;
}

RTT::FlowStatus RMLCartesianTask::getCurrentPosition(std::vector<double>& current_position){

    RTT::FlowStatus fs = _cartesian_state.readNewest(cartesian_state);
    if(fs == RTT::NewData){
        current_position.resize(motion_constraints.size());
        base::Vector3d euler = toEuler(cartesian_state.orientation);
        memcpy(current_position.data(),   cartesian_state.position.data(), sizeof(double)*3);
        memcpy(current_position.data()+3, euler.data(),                    sizeof(double)*3);

        current_sample = cartesian_state;
    }
    if(fs != RTT::NoData){
        current_sample.time = base::Time::now();
        _current_sample.write(current_sample);
    }
    return fs;
}

RTT::FlowStatus RMLCartesianTask::getTarget(TargetVector& target_vector){

    RTT::FlowStatus fs = _target.readNewest(target);
    if(fs == RTT::NewData){
        base::Vector3d euler = toEuler(target.orientation);
        memcpy(target_vector.position.data(),   target.position.data(),         sizeof(double) * 3);
        memcpy(target_vector.position.data()+3, euler.data(),                   sizeof(double) * 3);
        memcpy(target_vector.velocity.data(),   target.velocity.data(),         sizeof(double) * 3);
        memcpy(target_vector.velocity.data()+3, target.angular_velocity.data(), sizeof(double) * 3);
        memset(target_vector.selection_vector.data(), true, sizeof(double) * 6);
    }
    return fs;
}

base::Vector3d RMLCartesianTask::toEuler(const base::Orientation& orientation){
    // We use yaw-pitch-roll (ZYX wrt. rotated coordinate system) convention here
    return orientation.toRotationMatrix().eulerAngles(2,1,0);
}

base::Orientation RMLCartesianTask::fromEuler(const base::Vector3d& euler){
    base::Quaterniond q;
    // We use yaw-pitch-roll (ZYX wrt. rotated coordinate system) convention here
    q = Eigen::AngleAxisd(euler(0), base::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler(1), base::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler(2), base::Vector3d::UnitZ());
    return q;
}
