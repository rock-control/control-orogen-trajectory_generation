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
    base::samples::Joints current_state;
    /** Debug: Current interpolator status (position/speed/acceleration)*/
    base::samples::Joints current_sample;

public:
    RMLJointTask(std::string const& name = "trajectory_generation::RMLJointTask");
    RMLJointTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~RMLJointTask();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

