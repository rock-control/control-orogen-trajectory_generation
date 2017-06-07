/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLVELOCITYTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLVELOCITYTASK_TASK_HPP

#include "trajectory_generation/RMLVelocityTaskBase.hpp"

namespace trajectory_generation{

class RMLVelocityTask : public RMLVelocityTaskBase
{
    friend class RMLVelocityTaskBase;
protected:
    double no_reference_timeout;
    base::Time time_of_last_reference;
    bool convert_to_position;

    /** Velocity watchdog: Set target velocity to zero if no new reference arrives for more than no_reference_timeout seconds */
    void checkVelocityTimeout();

public:
    RMLVelocityTask(std::string const& name = "trajectory_generation::RMLVelocityTask");
    RMLVelocityTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~RMLVelocityTask();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();


    /** Call position or velocity based OTG, depending on the implementation*/
    virtual ReflexxesResultValue performOTG(base::commands::Joints &current_command);
    /** Set appropriate joint state depending on whether using position or velocity based RML*/
    virtual void setJointState(const base::JointState& state, const size_t idx);
    /** Set appropriate target depending on whether using position or velocity based RML*/
    virtual void setTarget(const base::JointState& cmd, const size_t idx);
    /** Set appropriate constraints depending on whether using position or velocity based RML*/
    virtual void setMotionConstraints(const trajectory_generation::MotionConstraint& constraints, const size_t idx);
    /** Convert from RMLOutputParameters to orogen type*/
    virtual const ReflexxesOutputParameters& fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out);
    /** Call echo() method for rml input and output parameters*/
    virtual void printParams();

};
}

#endif

