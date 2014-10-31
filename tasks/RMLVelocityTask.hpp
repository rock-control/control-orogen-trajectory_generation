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

    bool throw_on_infeasible_input_;
    bool override_input_position_;   /** Set output position as input for next cycle */
    bool override_input_speed_;      /** Set output speed as input for next cycle */
    bool override_input_acceleration_;  /** Set output acceleration as input for next cycle. When treat_effort_as_acceleration == false, input acceleration
                                            is always overriden. In this case, acceleration is always assumed to zero at first sample. */

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

