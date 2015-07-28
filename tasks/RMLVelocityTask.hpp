/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TRAJECTORY_GENERATION_RMLVELOCITYTASK_TASK_HPP
#define TRAJECTORY_GENERATION_RMLVELOCITYTASK_TASK_HPP

#include "trajectory_generation/RMLVelocityTaskBase.hpp"

#include <ReflexxesAPI.h>
#include <RMLVelocityFlags.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <base/commands/joints.h>
#include <trajectory_generation/trajectory_generationTypes.hpp>
#include <trajectory_generation/ConstrainedJointsTrajectory.hpp>


namespace trajectory_generation {

class RMLVelocityTask : public RMLVelocityTaskBase
{
    friend class RMLVelocityTaskBase;
protected:

    ReflexxesAPI *RML_;
    RMLVelocityInputParameters  *Vel_IP_;
    RMLVelocityOutputParameters *Vel_OP_;
    RMLVelocityFlags Vel_Flags_;

    /** Forcefully set the acceleration of the output command to this value (only for the specified joints)*/
    base::samples::Joints override_output_acceleration_;
    /** Should an exception be thrown if input was infeasible, e.g. target speed too high? */
    bool throw_on_infeasible_input_;
    /** Set output position as input for next cycle */
    bool override_input_position_;
    /** Set output speed as input for next cycle */
    bool override_input_speed_;
    /** Set output acceleration as input for next cycle. */
    bool override_input_acceleration_;
    /** Set Reference velocity to zero if no new reference arrives for this amount of time (in seconds).*/
    double velocity_timeout_;

    bool wrote_velocity_timeout_warning_;

    trajectory_generation::ConstrainedJointsCmd constrained_cmd_in_;
    trajectory_generation::JointsMotionConstraints limits_;
    base::samples::Joints status_;
    base::commands::Joints command_out_, command_in_;
    bool has_rml_been_called_, has_target_;
    double cycle_time_;
    size_t nDOF_;
    base::samples::Joints output_sample_;

    RMLInputParams input_params_;
    RMLOutputParams output_params_;

    base::Time stamp_;
    base::Time prev_time_;

    void handleStatusInput(const base::samples::Joints &status);
    void handleCommandInput(const base::commands::Joints &command);
    void setActiveMotionConstraints(const trajectory_generation::JointsMotionConstraints& constraints);
    void handleRMLInterpolationResult(const int res);
    void writeDebug(const RMLVelocityInputParameters* in, const RMLVelocityOutputParameters* out);
    void writeOutputCommand(const RMLVelocityOutputParameters* output);
    void writeOutputSample(const base::samples::Joints& status, const RMLVelocityOutputParameters* output);

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
};
}

#endif

