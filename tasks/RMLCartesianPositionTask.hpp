/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLCARTESIANPOSITIONTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLCARTESIANPOSITIONTASK_TASK_HPP

#include "trajectory_generation/RMLCartesianPositionTaskBase.hpp"

namespace trajectory_generation{

class RMLCartesianPositionTask : public RMLCartesianPositionTaskBase
{
    friend class RMLCartesianPositionTaskBase;

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
    RMLCartesianPositionTask(std::string const& name = "trajectory_generation::RMLCartesianPositionTask") : RMLCartesianPositionTaskBase(name){}
    RMLCartesianPositionTask(std::string const& name, RTT::ExecutionEngine* engine) : RMLCartesianPositionTaskBase(name, engine){}
    ~RMLCartesianPositionTask(){}
    bool configureHook(){return RMLCartesianPositionTaskBase::configureHook();}
    bool startHook(){return RMLCartesianPositionTaskBase::startHook();}
    void updateHook(){RMLCartesianPositionTaskBase::updateHook();}
    void errorHook(){RMLCartesianPositionTaskBase::errorHook();}
    void stopHook(){RMLCartesianPositionTaskBase::stopHook();}
    void cleanupHook(){RMLCartesianPositionTaskBase::cleanupHook();}
};
}

#endif

