/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLPOSITIONTRAJECTORYTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLPOSITIONTRAJECTORYTASK_TASK_HPP

#include "trajectory_generation/RMLPositionTrajectoryTaskBase.hpp"

namespace trajectory_generation {

/*! A temporary solution to interpolate complete trajectories. IMPORTANT: Jerk and acceleration have to be set to high values (e.g. 1000 and 100)
     * here, because the component will interpolate linearly between two trajectory points. */
class RMLPositionTrajectoryTask : public RMLPositionTrajectoryTaskBase
{
    friend class RMLPositionTrajectoryTaskBase;
protected:

    ConstrainedJointsTrajectory trajectory;
    size_t timestep;

    void setNextPoint(size_t t);
    void computeSpeeds(const base::samples::Joints &joint_state,
                       trajectory_generation::ConstrainedJointsTrajectory& trajectory);
    void handleNewTrajectory(const trajectory_generation::ConstrainedJointsTrajectory& trajectory);

public:
    RMLPositionTrajectoryTask(std::string const& name = "trajectory_generation::RMLPositionTrajectoryTask");
    RMLPositionTrajectoryTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~RMLPositionTrajectoryTask();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

