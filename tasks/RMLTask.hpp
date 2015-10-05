/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLTASK_TASK_HPP

#include "trajectory_generation/RMLTaskBase.hpp"
#include <base/commands/Joints.hpp>
#include "trajectory_generationTypes.hpp"
#include <reflexxes/ReflexxesAPI.h>

namespace trajectory_generation{

class RMLTask : public RMLTaskBase
{
    friend class RMLTaskBase;
protected:

    /** Motion constraints that define the properties of the output trajectory (command-port). These include the maximum/minimum position,
      * maximum maximum speed, maximum acceleration and maximum jerk (derivative of acceleration).*/
    trajectory_generation::JointsMotionConstraints motion_constraints;
    /** Current joint state. The joint names have to match the ones defined by the motion_constraints property. Joint indices
      * will be mapped internally by their names. The current joint state will only be used for initialization.*/
    base::samples::Joints joint_state;
    /** Target joint position/speed + new constraints. The component will generate a trajectory to that position, which complies with the motion constraints given
      * together with this command. If a constraint value (e.g. max.position) is NaN, it will not be changed, so the default motion constraints given
      * by the motion_constraints property apply.*/
    trajectory_generation::ConstrainedJointsCmd target;
    /** Output trajectory. The samples of the generated trajectory will be sent one by one. Size and names of this command will
      * be the same as in the motion_constraints property */
    base::commands::Joints command;
    /** Interface to the Online Trajectory Generation algorithms of the Reflexxes Motion Libraries*/
    ReflexxesAPI* rml_api;
    /** Input parameters for the OTG algorithm (target, constraints, flags, ...).*/
    RMLInputParameters *rml_input_parameters;
    /** Output parameters of the OTG algorithm (new states, errors, ...).*/
    RMLOutputParameters *rml_output_parameters;
    /** Input flags for the OTG algorithm.*/
    RMLFlags* rml_flags;
    /** Has the rml algorithm been initialized with the current joint state*/
    bool rml_initialized;
    /** RMLInputParameters do not work with orogen, so use own type*/
    ReflexxesInputParameters input_parameters;
    /** RMLOutputParameters do not work with orogen, so use own type*/
    ReflexxesOutputParameters output_parameters;
    /** Timestamp if updateHook();*/
    base::Time timestamp;
    /** Current sample of the interpolator. Will equal the current joint state, if no target has been given yet*/
    base::samples::Joints current_sample;
    /** Cycle time for interpolation*/
    double cycle_time;

    /** Handle an incoming joint state. Set positions/speeds/accelerations.*/
    void handleNewJointState(const base::samples::Joints &joint_state);
    /** Handle a new target. Set target positions/speeds.*/
    void handleNewTarget(const trajectory_generation::ConstrainedJointsCmd &target);
    /** Handle result of the OTG algorithm. Handle errors.*/
    void handleResultValue(ReflexxesResultValue result_value);


    /** Call position or velocity based OTG, depending on the implementation*/
    virtual ReflexxesResultValue performOTG(base::commands::Joints &current_command) = 0;
    /** Set appropriate joint state depending on whether using position or velocity based RML*/
    virtual void setJointState(const base::JointState& state, const size_t idx) = 0;
    /** Set appropriate target depending on whether using position or velocity based RML*/
    virtual void setTarget(const base::JointState& cmd, const size_t idx) = 0;
    /** Set appropriate constraints depending on whether using position or velocity based RML*/
    virtual void setMotionConstraints(const trajectory_generation::JointMotionConstraints& constraints, const size_t idx) = 0;
    /** Convert from RMLInputParameters to orogen type*/
    virtual const ReflexxesInputParameters& fromRMLTypes(const RMLInputParameters &in, ReflexxesInputParameters& out);
    /** Convert from RMLOutputParameters to orogen type*/
    virtual const ReflexxesOutputParameters& fromRMLTypes(const RMLOutputParameters &in, ReflexxesOutputParameters& out);
    /** Call echo() method for rml input and output parameters*/
    virtual void printParams() = 0;
    /** operation: Set new override value to define the relative speed of the overall motion*/
    virtual void setOverrideValue(double override_value);

public:
    RMLTask(std::string const& name = "trajectory_generation::RMLTask");
    RMLTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~RMLTask();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

