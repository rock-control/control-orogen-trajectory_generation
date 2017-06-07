/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLPOSITIONTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLPOSITIONTASK_TASK_HPP

#include "trajectory_generation/RMLPositionTaskBase.hpp"

namespace trajectory_generation{

class RMLPositionTask : public RMLPositionTaskBase
{
    friend class RMLPositionTaskBase;
protected:

public:
    RMLPositionTask(std::string const& name = "trajectory_generation::RMLPositionTask");
    RMLPositionTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~RMLPositionTask();
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
    /** Convert from RMLInputParameters to orogen type*/
    virtual const ReflexxesInputParameters& fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out);
    /** Convert from RMLOutputParameters to orogen type*/
    virtual const ReflexxesOutputParameters& fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out);
    /** Call echo() method for rml input and output parameters*/
    virtual void printParams();

};
}

#endif

