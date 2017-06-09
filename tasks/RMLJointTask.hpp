/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLJOINTTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLJOINTTASK_TASK_HPP

#include "trajectory_generation/RMLJointTaskBase.hpp"

namespace trajectory_generation{

class RMLJointTask : public RMLJointTaskBase
{
    friend class RMLJointTaskBase;
protected:
    /** Current joint state. The joint names have to match the ones defined by the motion_constraints property. Joint indices
        will be mapped internally by their names. The current joint state will only be used for initialization */
    base::samples::Joints current_joint_state;
    /** Debug: Current joint interpolator status (position/speed/acceleration)*/
    base::samples::Joints current_joint_sample;
    /** Target joint position or speed. The component will generate a trajectory to that position/speed, which complies with the motion constraints given by the
        motion_constraints property. The joint names in target have to match the ones defined by the motion_constraints property. Joint indices
        will be mapped internally by their names. */
    ConstrainedJointsCmd joint_target;

    /** Read the current state from port and return the current state if available and the flow status.*/
    virtual RTT::FlowStatus getCurrentState(CurrentStateVector& new_current_state);
    /** Read a new target from port and return the target if available and the flow status.*/
    virtual RTT::FlowStatus getTarget(TargetVector& new_target);

    /** Perform one step of online trajectory generation (call the RML algorithm with the given parameters). Return the RML result value*/
    virtual ReflexxesResultValue performOTG(const CurrentStateVector& current_state, const TargetVector& target) = 0;
    /** Call echo() method for rml input and output parameters*/
    virtual void printParams() = 0;

public:
    RMLJointTask(std::string const& name = "trajectory_generation::RMLJointTask") : RMLJointTaskBase(name){}
    RMLJointTask(std::string const& name, RTT::ExecutionEngine* engine) : RMLJointTaskBase(name, engine){}
    ~RMLJointTask(){}
    bool configureHook();
    bool startHook();
    void updateHook(){RMLJointTaskBase::updateHook();}
    void errorHook(){RMLJointTaskBase::errorHook();}
    void stopHook(){RMLJointTaskBase::stopHook();}
    void cleanupHook(){RMLJointTaskBase::cleanupHook();}
};
}

#endif

