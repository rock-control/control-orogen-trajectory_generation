/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLVELOCITYTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLVELOCITYTASK_TASK_HPP

#include "trajectory_generation/RMLVelocityTaskBase.hpp"

namespace trajectory_generation{

class RMLVelocityTask : public RMLVelocityTaskBase
{
    friend class RMLVelocityTaskBase;

    base::samples::Joints joint_state;    /** From input port: Current joint state. Will only be used for initializing RML */
    base::samples::Joints current_sample; /** From input port: Current joint interpolator status (position/speed/acceleration)*/
    ConstrainedJointsCmd target;          /** From input port: Target joint position or speed.  */
    base::commands::Joints command;       /** To output port: Commanded joint position or speed.  */

protected:
    /** Read the current state from port and return position and flow status*/
    virtual RTT::FlowStatus getCurrentState(CurrentStateData& current_state);
    /** Read target vector and return the flow status*/
    virtual RTT::FlowStatus getTarget(TargetData& target_vector);
    /** Write the generated trajectory to port*/
    virtual void writeCommand(const RMLOutputParameters& new_output_parameters);

public:
    RMLVelocityTask(std::string const& name = "trajectory_generation::RMLVelocityTask") : RMLVelocityTaskBase(name){}
    RMLVelocityTask(std::string const& name, RTT::ExecutionEngine* engine) : RMLVelocityTaskBase(name, engine){}
    ~RMLVelocityTask(){}
    bool configureHook(){return RMLVelocityTaskBase::configureHook();}
    bool startHook(){return RMLVelocityTaskBase::startHook();}
    void updateHook(){RMLVelocityTaskBase::updateHook();}
    void errorHook(){RMLVelocityTaskBase::errorHook();}
    void stopHook(){RMLVelocityTaskBase::stopHook();}
    void cleanupHook(){RMLVelocityTaskBase::cleanupHook();}
};
}

#endif

