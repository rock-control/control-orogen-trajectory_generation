/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLCARTESIANVELOCITYTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLCARTESIANVELOCITYTASK_TASK_HPP

#include "trajectory_generation/RMLCartesianVelocityTaskBase.hpp"

namespace trajectory_generation{

class RMLCartesianVelocityTask : public RMLCartesianVelocityTaskBase
{
    friend class RMLCartesianVelocityTaskBase;

    base::samples::RigidBodyState cartesian_state; /** From input port: Current Cartesian state. Will only be used for initializing RML */
    base::samples::RigidBodyState current_sample;  /** From input port: Current Cartesian interpolator status (position/speed/acceleration)*/
    base::samples::RigidBodyState target;          /** From input port: Target Cartesian position or speed.  */
    base::samples::RigidBodyState command;         /** To output port: Commanded Cartesian position or speed.  */

protected:
    /** Read the current state from port and return position and flow status*/
    virtual RTT::FlowStatus getCurrentState(CurrentStateData& current_state);
    /** Read target vector and return the flow status*/
    virtual RTT::FlowStatus getTarget(TargetData& target_vector);
    /** Write the generated trajectory to port*/
    virtual void writeCommand(const RMLOutputParameters& new_output_parameters);

public:
    RMLCartesianVelocityTask(std::string const& name = "trajectory_generation::RMLCartesianVelocityTask") : RMLCartesianVelocityTaskBase(name){}
    RMLCartesianVelocityTask(std::string const& name, RTT::ExecutionEngine* engine) : RMLCartesianVelocityTaskBase(name, engine){}
    ~RMLCartesianVelocityTask(){}
    bool configureHook(){return RMLCartesianVelocityTaskBase::configureHook();}
    bool startHook(){return RMLCartesianVelocityTaskBase::startHook();}
    void updateHook(){RMLCartesianVelocityTaskBase::updateHook();}
    void errorHook(){RMLCartesianVelocityTaskBase::errorHook();}
    void stopHook(){RMLCartesianVelocityTaskBase::stopHook();}
    void cleanupHook(){RMLCartesianVelocityTaskBase::cleanupHook();}
};
}

#endif

